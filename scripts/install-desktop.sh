#!/bin/bash
# Put the DroneLab apps (apps/*.desktop) into the KDE menu and onto the Desktop.
# Run once on the Deck as user deck:  ~/Desktop/dronelab/scripts/install-desktop.sh
# The .desktop files point to /home/deck/Desktop/dronelab; if the repo lives somewhere
# else, the installed copies are rewritten to the real location.
set -e
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
DEFAULT=/home/deck/Desktop/dronelab
APPS="$HOME/.local/share/applications"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$APPS" "$DESK"
chmod +x "$REPO"/scripts/*.sh "$REPO"/apps/*.desktop

for src in "$REPO"/apps/*.desktop; do
    name="$(basename "$src")"
    for dst in "$APPS/$name" "$DESK/$name"; do
        sed "s|$DEFAULT|$REPO|g" "$src" > "$dst"
        chmod +x "$dst"
    done
    # Plasma asks before running desktop files it does not trust; ours are trusted
    command -v gio >/dev/null && gio set "$DESK/$name" metadata::trusted true 2>/dev/null || true
    echo "installed $name"
done

# icons of the previous version of this repo -> one folder, so the Desktop is not cluttered
for old in connector.desktop drone_viz.desktop drone_viz_nomap.desktop start_container.desktop \
           lrs_fei_mavros.desktop gazebo_env.desktop ardupilot_sitl.desktop test_drone.desktop; do
    if [ -f "$DESK/$old" ]; then
        mkdir -p "$DESK/old-dronelab-icons"
        mv "$DESK/$old" "$DESK/old-dronelab-icons/"
    fi
done
command -v update-desktop-database >/dev/null && update-desktop-database "$APPS" 2>/dev/null || true
command -v kbuildsycoca5 >/dev/null && kbuildsycoca5 >/dev/null 2>&1 || true
command -v kbuildsycoca6 >/dev/null && kbuildsycoca6 >/dev/null 2>&1 || true

cat <<MSG

Done. The icons are on the Desktop and in the menu (Education).

Steam Deck controls in the Demo: Steam (desktop mode) -> Games -> Add a Non-Steam Game
to My Library -> Browse -> $REPO/scripts/dronelab-demo-steam.sh, start it from Steam and
choose the "Gamepad" controller layout. External USB/Bluetooth pads also work from the icon.
MSG
