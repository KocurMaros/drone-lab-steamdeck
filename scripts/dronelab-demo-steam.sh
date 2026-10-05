#!/bin/bash
# Add THIS file to Steam as a "Non-Steam Game" so the Steam Deck's own sticks and
# buttons reach the Demo as a gamepad (Steam Input). Use controller layout "Gamepad".
exec "$(dirname "$(readlink -f "$0")")/dronelab.sh" demo "$@"
