#!/usr/bin/env python3
"""DroneLab - check a STUDENT PC's ROS 2 setup against the Steam Deck (run it on the student PC).

    python3 student_check.py                 # finds the drones by itself
    python3 student_check.py --deck 192.168.88.250 --drone 13

Needs only ROS 2 Humble (source /opt/ros/humble/setup.bash first). It does not move any drone: it
lists the /droneNN topics, counts messages on /droneNN/gate/status and calls the harmless
/droneNN/gate/ping service, then says exactly what to change.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
import subprocess
import sys
import time

OK, BAD, WARN = "\033[32mOK\033[0m  ", "\033[31mFAIL\033[0m", "\033[33mWARN\033[0m"


def interfaces():
    try:
        js = subprocess.run(["ip", "-j", "-4", "addr", "show"], capture_output=True, text=True, timeout=3).stdout
        out = []
        for i in json.loads(js or "[]"):
            for a in i.get("addr_info", []):
                if a.get("family") == "inet" and "UP" in i.get("flags", []) and not a["local"].startswith("127."):
                    out.append((i["ifname"], ipaddress.ip_interface(f"{a['local']}/{a['prefixlen']}")))
        return out
    except Exception:
        return []


def pin_line(ifname, deck_ip):
    peers = f'<Peers><Peer Address="{deck_ip}"/></Peers>' if deck_ip else ""
    return ("export CYCLONEDDS_URI='<CycloneDDS><Domain Id=\"any\"><General><Interfaces>"
            f"<NetworkInterface name=\"{ifname}\"/></Interfaces></General><Discovery>"
            f"<ParticipantIndex>auto</ParticipantIndex>{peers}</Discovery></Domain></CycloneDDS>'")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone", type=int, help="drone ID (default: the first one found)")
    ap.add_argument("--deck", help="the Deck's IP, shown in the Flight app's drone tab (\"via wlan0 <IP>\")")
    ap.add_argument("--wait", type=float, default=8.0, help="seconds to wait for discovery")
    a = ap.parse_args()
    fixes = []

    print("== environment")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    dom = os.environ.get("ROS_DOMAIN_ID", "")
    print(f"  ROS_DISTRO={os.environ.get('ROS_DISTRO', '?')}  RMW_IMPLEMENTATION={rmw or '(default = Fast DDS)'}  "
          f"ROS_DOMAIN_ID={dom or '(unset = 0)'}")
    if os.environ.get("ROS_DISTRO") not in (None, "humble"):
        print(f"  {WARN} the Deck runs Humble; other distros may not understand each other's messages")
    if rmw != "rmw_cyclonedds_cpp":
        print(f"  {BAD} services (set_mode, cmd/arming, ...) only work with CycloneDDS")
        fixes.append("sudo apt install ros-humble-rmw-cyclonedds-cpp\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    if dom not in ("", "0"):
        print(f"  {BAD} ROS_DOMAIN_ID={dom}: the drones are on domain 0")
        fixes.append("export ROS_DOMAIN_ID=0")
    if os.environ.get("ROS_LOCALHOST_ONLY", "0") not in ("", "0"):
        print(f"  {BAD} ROS_LOCALHOST_ONLY=1 hides every other computer")
        fixes.append("unset ROS_LOCALHOST_ONLY")
    if os.environ.get("ROS_DISCOVERY_SERVER"):
        print(f"  {BAD} ROS_DISCOVERY_SERVER is set (Fast DDS discovery server): the Deck does not use one")
        fixes.append("unset ROS_DISCOVERY_SERVER")
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if uri:
        print(f"  CYCLONEDDS_URI={uri[:110]}{'...' if len(uri) > 110 else ''}")

    print("== network interfaces (IPv4, up)")
    ifs = interfaces()
    for name, ip in ifs:
        print(f"  {name:12s} {ip}")
    deck_ip = a.deck

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node("dronelab_student_check")

    print(f"== discovery ({a.wait:.0f} s)")
    drones = set()
    t0 = time.time()
    while time.time() - t0 < a.wait:
        rclpy.spin_once(node, timeout_sec=0.2)
        for name, _ in node.get_topic_names_and_types():
            m = re.match(r"^/drone(\d+)/", name)
            if m:
                drones.add(int(m.group(1)))
    if not drones:
        print(f"  {BAD} no /droneNN topics: no drone is connected in the Flight app, or this PC cannot hear the Deck")
    else:
        print(f"  {OK} drones seen: {sorted(drones)}")
    drone = a.drone or (min(drones) if drones else None)

    got = []
    status = {}
    ping = None
    if drone is not None:
        ns = f"/drone{drone}"
        node.create_subscription(String, f"{ns}/gate/status", lambda m: got.append(m.data), qos_profile_sensor_data)
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if got:
            print(f"  {OK} {ns}/gate/status: {len(got)} messages in 5 s (data arrives)")
            try:
                status = json.loads(got[-1])
            except ValueError:
                pass
        else:
            print(f"  {BAD} {ns}/gate/status: no messages in 5 s - the topic is visible but no DATA arrives")
        cli = node.create_client(Trigger, f"{ns}/gate/ping")
        if cli.wait_for_service(timeout_sec=3.0):
            fut = cli.call_async(Trigger.Request())
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 5.0:
                rclpy.spin_once(node, timeout_sec=0.1)
            ping = fut.result() if fut.done() else None
            if ping and ping.success:
                print(f"  {OK} {ns}/gate/ping: {ping.message} (services work)")
            else:
                print(f"  {BAD} {ns}/gate/ping: no reply - services do not work from this PC")
        else:
            print(f"  {WARN} {ns}/gate/ping not found (older Deck software?)")

    if not deck_ip and status.get("student_iface"):
        deck_ip = str(status["student_iface"]).split()[-1]
    same = [(n, ip) for n, ip in ifs if deck_ip and _in_net(deck_ip, ip)]
    transport_broken = drone is None or not got
    cyclone = rmw == "rmw_cyclonedds_cpp"
    broken = transport_broken or (cyclone and not (ping and ping.success))
    pinned = "NetworkInterface" in uri
    if deck_ip:
        print(f"== the Deck is {deck_ip}; this PC's interface on that network: "
              f"{', '.join(n for n, _ in same) or 'NONE'}")
        if not same:
            print(f"  {BAD} this PC is not on the Deck's network")
            fixes.append(f"# connect this PC to the same network as the Deck ({deck_ip}), then run me again")
        elif len(ifs) > 1 and not pinned:
            print(f"  {BAD if broken else WARN} {len(ifs)} network interfaces ({', '.join(n for n, _ in ifs)}): "
                  f"DDS may announce an address the Deck cannot reach (Docker, VPN, second network card)")
            fixes.append(pin_line(same[0][0], deck_ip))
        elif broken and not pinned:
            fixes.append(pin_line(same[0][0], deck_ip))
    elif broken:
        print(f"  {WARN} run again with --deck <IP> (the Flight app's drone tab shows it: \"via wlan0 <IP>\")")
    if transport_broken:
        fixes.append("sudo ufw status        # if active: sudo ufw allow from " + (deck_ip or "<the Deck's IP>"))

    print("== verdict")
    if not fixes:
        print(f"  {OK} everything works - use /drone{drone}/... like MAVROS topics and services")
    else:
        print("  put this in ~/.bashrc (then open a new terminal, run `ros2 daemon stop` and this check again):\n")
        for f in fixes:
            print("    " + f.replace("\n", "\n    "))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if not fixes else 1


def _in_net(ip, iface):
    try:
        return ipaddress.ip_address(ip) in iface.network
    except ValueError:
        return False


if __name__ == "__main__":
    sys.exit(main())
