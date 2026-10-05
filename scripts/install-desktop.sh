#!/bin/bash
# Install the DroneLab icons into the KDE menu and onto the Desktop (run once, on the Deck, as user deck).
set -e
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
APPS="$HOME/.local/share/applications"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$APPS" "$DESK"
chmod +x "$REPO"/scripts/*.sh

entry() {  # file name, title, comment, command, icon, terminal
    cat > "$APPS/$1" <<EOF
[Desktop Entry]
Type=Application
Name=$2
Comment=$3
Exec=$REPO/scripts/dronelab.sh $4
Icon=$REPO/resources/icons/$5
Terminal=$6
Categories=Education;Science;
EOF
    chmod +x "$APPS/$1"
    cp "$APPS/$1" "$DESK/$1"
    chmod +x "$DESK/$1"
    # Plasma 5.27+/6 asks before running untrusted desktop files; mark ours as trusted
    command -v gio >/dev/null && gio set "$DESK/$1" metadata::trusted true 2>/dev/null || true
}

entry dronelab-flight.desktop "DroneLab Flight" "Connect real drones through the safety gate" flight connector.png false
entry dronelab-demo.desktop "DroneLab Demo" "Simulation demo: fly with a gamepad" demo gazebo.png false
entry dronelab-shell.desktop "DroneLab Shell" "Terminal inside the DroneLab container" shell ros.png false

# old icons from the previous version of this repo
for old in connector.desktop drone_viz.desktop drone_viz_nomap.desktop start_container.desktop \
           lrs_fei_mavros.desktop gazebo_env.desktop ardupilot_sitl.desktop test_drone.desktop; do
    [ -f "$DESK/$old" ] && mkdir -p "$DESK/old-dronelab-icons" && mv "$DESK/$old" "$DESK/old-dronelab-icons/"
done
command -v update-desktop-database >/dev/null && update-desktop-database "$APPS" 2>/dev/null || true

cat <<EOF
Installed: DroneLab Flight, DroneLab Demo, DroneLab Shell (menu + Desktop).

Steam Deck controls in the Demo: in Steam (desktop mode) choose
  Games -> Add a Non-Steam Game to My Library -> Browse -> $REPO/scripts/dronelab-demo-steam.sh
then start "DroneLab Demo" from Steam. Steam then turns the Deck's sticks/buttons into a gamepad.
External USB/Bluetooth gamepads work from the desktop icon too.
EOF
