#!/usr/bin/env bash
# Install DroneDeck desktop integration for the current user:
#   - the app-menu launchers (paths rewritten to this checkout)
#   - the .tlog MIME type, so double-clicking a telemetry log replays it in DroneDeck
# Re-runnable; no root needed.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APPS="$HOME/.local/share/applications"
MIME="$HOME/.local/share/mime"
mkdir -p "$APPS" "$MIME/packages"

# launchers, with Exec/Path/Icon pointed at this checkout (suffix like %f / demo kept)
for d in DroneDeck DroneDeck-Demo; do
    [ -f "$ROOT/$d.desktop" ] || continue
    sed -e "s|\"[^\"]*run.sh\"|\"$ROOT/run.sh\"|" \
        -e "s|^Path=.*|Path=$ROOT|" \
        -e "s|^Icon=.*|Icon=$ROOT/app/assets/icon.png|" \
        "$ROOT/$d.desktop" > "$APPS/$d.desktop"
done

# .tlog -> application/x-dronedeck-tlog -> DroneDeck
cp "$ROOT/packaging/dronedeck-tlog.xml" "$MIME/packages/"
update-mime-database "$MIME"
update-desktop-database "$APPS" 2>/dev/null || true
xdg-mime default DroneDeck.desktop application/x-dronedeck-tlog

echo "Installed launchers in $APPS and the .tlog association."
echo "Double-click a *.tlog to replay it in DroneDeck."
