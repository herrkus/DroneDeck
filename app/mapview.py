"""mapview.py -- slippy-map widget (OpenStreetMap tiles) with vehicle overlay.

North-up, follows the vehicle, wheel to zoom, drag to pan. Tiles are cached to
disk and memory and fetched asynchronously; with no network it degrades to a
lat/lon graticule so the overlay (marker + flight trail) still works offline.
No 3D model -- the vehicle is a simple heading-rotated marker.
"""
from __future__ import annotations
import math
import os

from PySide6.QtCore import Qt, QRectF, QPointF, QUrl, Signal
from PySide6.QtGui import (QPainter, QColor, QPen, QBrush, QPixmap, QPolygonF,
                           QFont, QPixmapCache)
from PySide6.QtWidgets import QWidget, QMenu

try:
    from PySide6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
    _HAVE_NET = True
except Exception:                       # pragma: no cover
    _HAVE_NET = False

TILE = 256
# Public tile sources (no API key). ESRI World Imagery uses {z}/{y}/{x} path order.
PROVIDERS = {
    "Street":    "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    "Satellite": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    "Topo":      "https://tile.opentopomap.org/{z}/{x}/{y}.png",
}
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
    clicked = Signal(float, float)            # map click -> (lat, lon)
    contextAction = Signal(str, float, float)  # right-click empty map -> (action, lat, lon)
    wpAction = Signal(str, int)               # right-click a waypoint -> (action, wp_index)
    waypoint_selected = Signal(int)           # a planned waypoint was clicked
    waypoint_moved = Signal(int, float, float)  # waypoint dragged -> (index, lat, lon)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 300)
        self.zoom = 16
        self.center = (54.6872, 25.2797)     # placeholder until first fix
        self.follow = True
        self.provider = "Street"
        self.veh = None                       # (lat, lon, heading) or None
        self.home = None
        self.trail = []
        self.mission = []                     # list of (lat, lon) planned waypoints
        self.fence = []                       # inclusion polygon vertices (lat, lon)
        self.fence_exc = []                   # exclusion polygon vertices
        self.fence_circles = []               # [{"lat","lon","radius","incl"}]
        self.rally = []                       # list of (lat, lon) rally points
        self.traffic = []                     # ADSB: [{"lat","lon","heading","callsign"}]
        self.others = []                      # other vehicles: [(lat, lon, heading)]
        self.selected_wp = -1
        self.current_wp = -1                  # live mission target (MISSION_CURRENT)
        self._wp_drag = None
        self._pending = set()
        self._drag = None
        self._press = None
        self._dragged = False
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

    def set_provider(self, name):
        if name in PROVIDERS and name != self.provider:
            self.provider = name
            self._pending.clear()      # re-fetch visible tiles from the new source
            self.update()

    def set_mission(self, pts):
        self.mission = list(pts)
        if self.selected_wp >= len(self.mission):
            self.selected_wp = -1
        self.update()

    def set_fence(self, pts):
        self.fence = list(pts)
        self.update()

    def set_fence_shapes(self, inc, exc, circles):
        self.fence = list(inc)
        self.fence_exc = list(exc)
        self.fence_circles = list(circles)
        self.update()

    def set_rally(self, pts):
        self.rally = list(pts)
        self.update()

    def set_traffic(self, traffic):
        self.traffic = list(traffic)
        self.update()

    def set_others(self, others):
        self.others = list(others)
        self.update()

    def set_selected(self, idx):
        self.selected_wp = idx
        self.update()

    def set_current_wp(self, seq):
        if seq != self.current_wp:
            self.current_wp = seq
            self.update()

    def _nearest_wp(self, pos):
        if not self.mission:
            return -1
        cfx, cfy = deg2num(self.center[0], self.center[1], self.zoom)
        best, bestd = -1, 14.0
        for i, (la, lo) in enumerate(self.mission):
            p = self._ll_to_px(la, lo, cfx, cfy)
            d = ((p.x() - pos.x()) ** 2 + (p.y() - pos.y()) ** 2) ** 0.5
            if d < bestd:
                best, bestd = i, d
        return best

    # -- tile cache -----------------------------------------------------------
    def _tile_path(self, prov, z, x, y):
        return os.path.join(_CACHE, prov, str(z), str(x), f"{y}.png")

    def _tile_pixmap(self, z, x, y):
        key = f"{self.provider}/{z}/{x}/{y}"
        pm = QPixmapCache.find(key)
        if pm:
            return pm
        path = self._tile_path(self.provider, z, x, y)
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
        url = QUrl(PROVIDERS[self.provider].format(z=z, x=x, y=y))
        req = QNetworkRequest(url)
        req.setHeader(QNetworkRequest.KnownHeaders.UserAgentHeader, "DroneDeck/1.0 (local GCS demo)")
        req.setAttribute(QNetworkRequest.Attribute.User, f"{self.provider}/{z}/{x}/{y}")
        self.net.get(req)

    def _on_tile(self, reply):
        key = reply.request().attribute(QNetworkRequest.Attribute.User)
        try:
            prov, zs, xs, ys = key.split("/")
            z, x, y = int(zs), int(xs), int(ys)
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
                    os.makedirs(os.path.dirname(self._tile_path(prov, z, x, y)), exist_ok=True)
                    with open(self._tile_path(prov, z, x, y), "wb") as fh:
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

        # inclusion geofence polygon (dashed red, translucent fill)
        if len(self.fence) >= 2:
            fp = [self._ll_to_px(la, lo, cfx, cfy) for la, lo in self.fence]
            p.setPen(QPen(QColor(255, 80, 80, 230), 2, Qt.DashLine))
            p.setBrush(QBrush(QColor(255, 80, 80, 30)))
            p.drawPolygon(QPolygonF(fp))

        # exclusion geofence polygon (dashed orange)
        if len(self.fence_exc) >= 2:
            ep = [self._ll_to_px(la, lo, cfx, cfy) for la, lo in self.fence_exc]
            p.setPen(QPen(QColor(255, 160, 40, 230), 2, Qt.DashLine))
            p.setBrush(QBrush(QColor(255, 160, 40, 40)))
            p.drawPolygon(QPolygonF(ep))

        # geofence circles (inclusion red, exclusion orange)
        for c in self.fence_circles:
            cp = self._ll_to_px(c["lat"], c["lon"], cfx, cfy)
            mpp = 156543.03392 * math.cos(math.radians(c["lat"])) / (2 ** self.zoom)
            r_px = float(c.get("radius", 0)) / mpp if mpp else 0
            col = QColor(255, 80, 80, 230) if c.get("incl", True) else QColor(255, 160, 40, 230)
            fill = QColor(255, 80, 80, 30) if c.get("incl", True) else QColor(255, 160, 40, 40)
            p.setPen(QPen(col, 2, Qt.DashLine))
            p.setBrush(QBrush(fill))
            p.drawEllipse(cp, r_px, r_px)

        # rally points (green diamonds)
        if self.rally:
            p.setPen(QPen(QColor(20, 60, 20), 1.5))
            p.setBrush(QBrush(QColor(90, 220, 130)))
            for la, lo in self.rally:
                rp = self._ll_to_px(la, lo, cfx, cfy)
                p.drawPolygon(QPolygonF([QPointF(rp.x(), rp.y() - 8), QPointF(rp.x() + 8, rp.y()),
                                         QPointF(rp.x(), rp.y() + 8), QPointF(rp.x() - 8, rp.y())]))

        # planned mission: path + numbered waypoints
        if self.mission:
            pts = [self._ll_to_px(la, lo, cfx, cfy) for la, lo in self.mission]
            if len(pts) > 1:
                p.setPen(QPen(QColor(255, 205, 0, 230), 2))
                p.drawPolyline(QPolygonF(pts))
            wpf = QFont("DejaVu Sans Mono", 8)
            wpf.setBold(True)
            for i, pt in enumerate(pts):
                if i == self.current_wp:                 # live target waypoint: green ring
                    p.setPen(QPen(QColor(0, 230, 90), 3))
                    p.setBrush(Qt.NoBrush)
                    p.drawEllipse(pt, 14, 14)
                if i == self.selected_wp:
                    p.setPen(QPen(QColor(255, 255, 255), 2))
                    p.setBrush(QBrush(QColor(255, 140, 0)))
                    p.drawEllipse(pt, 11, 11)
                else:
                    p.setPen(QPen(QColor(40, 30, 0), 1.5))
                    p.setBrush(QBrush(QColor(255, 190, 0)))
                    p.drawEllipse(pt, 9, 9)
                p.setPen(QColor(20, 20, 20))
                p.setFont(wpf)
                p.drawText(QRectF(pt.x() - 9, pt.y() - 8, 18, 16), Qt.AlignCenter, str(i))

        # ADSB traffic (amber chevrons + callsign)
        for tr in self.traffic:
            tp = self._ll_to_px(tr["lat"], tr["lon"], cfx, cfy)
            p.save()
            p.translate(tp)
            p.rotate(tr.get("heading", 0))
            p.setPen(QPen(QColor(20, 20, 20), 1.2))
            p.setBrush(QBrush(QColor(255, 190, 40)))
            p.drawPolygon(QPolygonF([QPointF(0, -9), QPointF(6, 8), QPointF(-6, 8)]))
            p.restore()
            cs = tr.get("callsign", "")
            if cs:
                p.setPen(QColor(255, 210, 90))
                p.setFont(QFont("DejaVu Sans Mono", 7))
                p.drawText(QRectF(tp.x() + 8, tp.y() - 8, 80, 14), Qt.AlignVCenter, cs)

        # other vehicles (dimmed cyan triangles)
        for la, lo, hdg in self.others:
            op = self._ll_to_px(la, lo, cfx, cfy)
            p.save()
            p.translate(op)
            p.rotate(hdg)
            p.setPen(QPen(QColor(0, 0, 0), 1.2))
            p.setBrush(QBrush(QColor(90, 200, 230)))
            p.drawPolygon(QPolygonF([QPointF(0, -10), QPointF(7, 9),
                                     QPointF(0, 4), QPointF(-7, 9)]))
            p.restore()

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
    def _px_to_ll(self, px, py):
        cfx, cfy = deg2num(self.center[0], self.center[1], self.zoom)
        fx = cfx + (px - self.width() / 2.0) / TILE
        fy = cfy + (py - self.height() / 2.0) / TILE
        return num2deg(fx, fy, self.zoom)

    def wheelEvent(self, e):
        self.set_zoom(self.zoom + (1 if e.angleDelta().y() > 0 else -1))

    def mousePressEvent(self, e):
        idx = self._nearest_wp(e.position())
        if idx >= 0:                           # grab a waypoint to drag
            self._wp_drag = idx
            self.selected_wp = idx
            self.waypoint_selected.emit(idx)
            self.update()
            return
        self._drag = e.position()
        self._press = e.position()
        self._dragged = False

    def mouseMoveEvent(self, e):
        if self._wp_drag is not None:
            la, lo = self._px_to_ll(e.position().x(), e.position().y())
            if 0 <= self._wp_drag < len(self.mission):
                self.mission[self._wp_drag] = (la, lo)
                self.waypoint_moved.emit(self._wp_drag, la, lo)
                self.update()
            return
        if self._drag is None:
            return
        d = e.position() - self._drag
        if abs(d.x()) + abs(d.y()) > 2:
            self._dragged = True
            self.follow = False        # panning detaches from the vehicle
        self._drag = e.position()
        cfx, cfy = deg2num(self.center[0], self.center[1], self.zoom)
        self.center = num2deg(cfx - d.x() / TILE, cfy - d.y() / TILE, self.zoom)
        self.update()

    def mouseReleaseEvent(self, e):
        if self._wp_drag is not None:          # finished moving a waypoint
            self._wp_drag = None
            return
        was_click = (self._drag is not None and not self._dragged and self._press is not None)
        self._drag = None
        if was_click:
            la, lo = self._px_to_ll(self._press.x(), self._press.y())
            self.clicked.emit(la, lo)

    def contextMenuEvent(self, e):
        menu = QMenu(self)
        idx = self._nearest_wp(e.pos())
        if idx >= 0:                           # right-clicked a planned waypoint -> edit it
            for label, key in (("Edit waypoint...", "edit"),
                               ("Insert waypoint before", "insert_before"),
                               ("Delete waypoint", "delete")):
                act = menu.addAction(label)
                act.triggered.connect(lambda _=False, k=key, i=idx: self.wpAction.emit(k, i))
        else:                                  # empty map -> guided/flight actions
            la, lo = self._px_to_ll(e.pos().x(), e.pos().y())
            for label, key in (("Go to here", "goto"), ("Orbit here", "orbit"),
                               ("Point camera here (ROI)", "roi"),
                               ("Add ROI waypoint here", "add_roi"), ("Set home here", "sethome")):
                act = menu.addAction(label)
                act.triggered.connect(lambda _=False, k=key: self.contextAction.emit(k, la, lo))
        menu.exec(e.globalPos())

    def mouseDoubleClickEvent(self, _):
        self.follow = True
        if self.veh:
            self.center = (self.veh[0], self.veh[1])
        self.update()
