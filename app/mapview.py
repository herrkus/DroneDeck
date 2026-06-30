"""mapview.py -- slippy-map widget (OpenStreetMap tiles) with vehicle overlay.

North-up, follows the vehicle, wheel to zoom, drag to pan. Tiles are cached to
disk and memory and fetched asynchronously; with no network it degrades to a
lat/lon graticule so the overlay (marker + flight trail) still works offline.
No 3D model -- the vehicle is a simple heading-rotated marker.
"""
from __future__ import annotations
import math
import os

from PySide6.QtCore import Qt, QRectF, QPointF, QUrl
from PySide6.QtGui import (QPainter, QColor, QPen, QBrush, QPixmap, QPolygonF,
                           QFont, QPixmapCache)
from PySide6.QtWidgets import QWidget

try:
    from PySide6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
    _HAVE_NET = True
except Exception:                       # pragma: no cover
    _HAVE_NET = False

TILE = 256
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CACHE = os.path.join(_ROOT, "tiles_cache")


def deg2num(lat, lon, z):
    lat = max(-85.05, min(85.05, lat))
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def num2deg(x, y, z):
    n = 2 ** z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


class MapView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 300)
        self.zoom = 16
        self.center = (54.6872, 25.2797)     # placeholder until first fix
        self.follow = True
        self.veh = None                       # (lat, lon, heading) or None
        self.home = None
        self.trail = []
        self._pending = set()
        self._drag = None
        QPixmapCache.setCacheLimit(40 * 1024)
        os.makedirs(_CACHE, exist_ok=True)
        self.net = QNetworkAccessManager(self) if _HAVE_NET else None
        if self.net:
            self.net.finished.connect(self._on_tile)

    # -- public API -----------------------------------------------------------
    def update_vehicle(self, lat, lon, heading, home, trail):
        self.veh = (lat, lon, heading)
        self.home = home
        self.trail = trail
        if self.follow:
            self.center = (lat, lon)
        self.update()

    def set_zoom(self, z):
        self.zoom = max(3, min(19, int(z)))
        self.update()

    # -- tile cache -----------------------------------------------------------
    def _tile_path(self, z, x, y):
        return os.path.join(_CACHE, str(z), str(x), f"{y}.png")

    def _tile_pixmap(self, z, x, y):
        key = f"{z}/{x}/{y}"
        pm = QPixmapCache.find(key)
        if pm:
            return pm
        path = self._tile_path(z, x, y)
        if os.path.exists(path):
            pm = QPixmap(path)
            if not pm.isNull():
                QPixmapCache.insert(key, pm)
                return pm
        self._request(z, x, y)
        return None

    def _request(self, z, x, y):
        if not self.net or (z, x, y) in self._pending:
            return
        n = 2 ** z
        if not (0 <= x < n and 0 <= y < n):
            return
        self._pending.add((z, x, y))
        url = QUrl(f"https://tile.openstreetmap.org/{z}/{x}/{y}.png")
        req = QNetworkRequest(url)
        req.setHeader(QNetworkRequest.KnownHeaders.UserAgentHeader, "DroneDeck/1.0 (local GCS demo)")
        req.setAttribute(QNetworkRequest.Attribute.User, f"{z}/{x}/{y}")
        self.net.get(req)

    def _on_tile(self, reply):
        key = reply.request().attribute(QNetworkRequest.Attribute.User)
        try:
            z, x, y = (int(v) for v in key.split("/"))
        except Exception:
            reply.deleteLater()
            return
        self._pending.discard((z, x, y))
        if reply.error() == QNetworkReply.NetworkError.NoError:
            data = bytes(reply.readAll())
            pm = QPixmap()
            if pm.loadFromData(data):
                QPixmapCache.insert(key, pm)
                try:
                    os.makedirs(os.path.dirname(self._tile_path(z, x, y)), exist_ok=True)
                    with open(self._tile_path(z, x, y), "wb") as fh:
                        fh.write(data)
                except OSError:
                    pass
                self.update()
        reply.deleteLater()

    # -- geometry helpers -----------------------------------------------------
    def _ll_to_px(self, lat, lon, cfx, cfy):
        fx, fy = deg2num(lat, lon, self.zoom)
        return QPointF((fx - cfx) * TILE + self.width() / 2.0,
                       (fy - cfy) * TILE + self.height() / 2.0)

    # -- paint ----------------------------------------------------------------
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(28, 30, 36))
        w, h = self.width(), self.height()
        cfx, cfy = deg2num(self.center[0], self.center[1], self.zoom)

        # visible tile range
        x0 = int(cfx - (w / 2) / TILE) - 1
        x1 = int(cfx + (w / 2) / TILE) + 1
        y0 = int(cfy - (h / 2) / TILE) - 1
        y1 = int(cfy + (h / 2) / TILE) + 1
        n = 2 ** self.zoom
        any_tile = False
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                wx = ((tx % n) + n) % n
                if not (0 <= ty < n):
                    continue
                px = (tx - cfx) * TILE + w / 2.0
                py = (ty - cfy) * TILE + h / 2.0
                pm = self._tile_pixmap(self.zoom, wx, ty)
                if pm:
                    p.drawPixmap(int(px), int(py), pm)
                    any_tile = True
                else:
                    p.fillRect(QRectF(px, py, TILE, TILE), QColor(40, 43, 50))
                    p.setPen(QPen(QColor(55, 58, 66), 1))
                    p.drawRect(QRectF(px, py, TILE, TILE))

        if not any_tile:
            self._draw_graticule(p, cfx, cfy)

        # flight trail
        if len(self.trail) > 1:
            p.setPen(QPen(QColor(0, 220, 255, 220), 2))
            poly = QPolygonF([self._ll_to_px(la, lo, cfx, cfy) for la, lo in self.trail])
            p.drawPolyline(poly)

        # home
        if self.home:
            hp = self._ll_to_px(self.home[0], self.home[1], cfx, cfy)
            p.setPen(QPen(QColor(255, 255, 255), 2))
            p.setBrush(QBrush(QColor(0, 160, 0)))
            p.drawEllipse(hp, 5, 5)
            p.drawText(QRectF(hp.x() + 8, hp.y() - 8, 50, 16), Qt.AlignVCenter, "H")

        # vehicle marker (heading-rotated triangle, no 3D model)
        if self.veh:
            vp = self._ll_to_px(self.veh[0], self.veh[1], cfx, cfy)
            p.save()
            p.translate(vp)
            p.rotate(self.veh[2])
            p.setPen(QPen(QColor(0, 0, 0), 1.5))
            p.setBrush(QBrush(QColor(255, 80, 60)))
            p.drawPolygon(QPolygonF([QPointF(0, -12), QPointF(8, 10),
                                     QPointF(0, 5), QPointF(-8, 10)]))
            p.restore()

        self._draw_hud(p, any_tile)
        p.end()

    def _draw_graticule(self, p, cfx, cfy):
        p.setPen(QPen(QColor(70, 74, 82), 1))
        step = 0.01 if self.zoom >= 13 else 0.1
        latc, lonc = self.center
        p.setFont(QFont("DejaVu Sans Mono", 7))
        for i in range(-6, 7):
            lat = round(latc / step) * step + i * step
            pt = self._ll_to_px(lat, lonc, cfx, cfy)
            p.drawLine(0, int(pt.y()), self.width(), int(pt.y()))
            p.setPen(QColor(120, 124, 132))
            p.drawText(2, int(pt.y()) - 2, f"{lat:.3f}")
            p.setPen(QPen(QColor(70, 74, 82), 1))
        for i in range(-6, 7):
            lon = round(lonc / step) * step + i * step
            pt = self._ll_to_px(latc, lon, cfx, cfy)
            p.drawLine(int(pt.x()), 0, int(pt.x()), self.height())

    def _draw_hud(self, p, online):
        p.setPen(QColor(180, 184, 192))
        p.setFont(QFont("DejaVu Sans Mono", 8))
        tag = "OSM" if online else "offline grid"
        p.drawText(8, self.height() - 8, f"z{self.zoom}  {tag}")

    # -- interaction ----------------------------------------------------------
    def wheelEvent(self, e):
        self.set_zoom(self.zoom + (1 if e.angleDelta().y() > 0 else -1))

    def mousePressEvent(self, e):
        self._drag = e.position()
        self.follow = False

    def mouseMoveEvent(self, e):
        if self._drag is None:
            return
        d = e.position() - self._drag
        self._drag = e.position()
        cfx, cfy = deg2num(self.center[0], self.center[1], self.zoom)
        cfx -= d.x() / TILE
        cfy -= d.y() / TILE
        self.center = num2deg(cfx, cfy, self.zoom)
        self.update()

    def mouseReleaseEvent(self, _):
        self._drag = None

    def mouseDoubleClickEvent(self, _):
        self.follow = True
        if self.veh:
            self.center = (self.veh[0], self.veh[1])
        self.update()
