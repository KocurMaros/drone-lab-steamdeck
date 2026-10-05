import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from dronelab import mavlink_lite as ml
from dronelab.config import parse_ports, parse_targets
from dronelab.discovery import Discovery, Path, subnet_hosts_24

pymavlink = pytest.importorskip("pymavlink.dialects.v20.ardupilotmega")


def _pm_heartbeat(sysid, compid=1, version=2, custom_mode=4, armed=True):
    from pymavlink.dialects.v20 import ardupilotmega as m
    mav = m.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    if version == 1:
        from pymavlink.dialects.v10 import ardupilotmega as m1
        mav = m1.MAVLink(None, srcSystem=sysid, srcComponent=compid)
        m = m1
    msg = m.MAVLink_heartbeat_message(m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                      (128 if armed else 0) | 1, custom_mode, m.MAV_STATE_ACTIVE, 3)
    return msg.pack(mav)


@pytest.mark.parametrize("version", [1, 2])
def test_parse_pymavlink_heartbeat(version):
    buf = _pm_heartbeat(11, version=version)
    hbs = list(ml.heartbeats(buf))
    assert len(hbs) == 1
    hb = hbs[0]
    assert hb.sysid == 11 and hb.armed and hb.mode_name == "GUIDED" and hb.is_vehicle
    assert hb.version == version


def test_parse_multiple_frames_with_garbage_and_bad_crc():
    good = _pm_heartbeat(12, custom_mode=5)
    bad = bytearray(_pm_heartbeat(13))
    bad[-1] ^= 0xFF
    buf = b"\x00\x01junk" + good + bytes(bad) + _pm_heartbeat(14, version=1)
    ids = [hb.sysid for hb in ml.heartbeats(buf)]
    assert ids == [12, 14]


@pytest.mark.parametrize("version", [1, 2])
def test_our_heartbeat_is_valid_for_pymavlink(version):
    from pymavlink import mavutil
    pkt = ml.pack_heartbeat(seq=7, version=version)
    mav = mavutil.mavlink.MAVLink(None)
    if version == 1:
        from pymavlink.dialects.v10 import ardupilotmega as m1
        mav = m1.MAVLink(None)
    msgs = mav.parse_buffer(pkt)
    assert msgs and msgs[0].get_type() == "HEARTBEAT"
    assert msgs[0].type == ml.MAV_TYPE_GCS
    assert msgs[0].get_srcSystem() == 255


def test_gcs_heartbeats_are_not_vehicles():
    hb = list(ml.heartbeats(ml.pack_heartbeat()))[0]
    assert not hb.is_vehicle


def test_parse_ports_and_targets():
    assert parse_ports(["14510-14512", 14550, "14550"]) == [14510, 14511, 14512, 14550]
    assert parse_targets(["192.168.18.110-112", "10.42.0.1"]) == ["192.168.18.110", "192.168.18.111",
                                                                   "192.168.18.112", "10.42.0.1"]
    assert len(parse_targets(["10.0.0.0/30"])) == 2
    with pytest.raises(ValueError):
        parse_targets(["10.0.0.0/8"])


def test_subnet_hosts_skip_own_ip():
    import ipaddress
    hosts = subnet_hosts_24([ipaddress.IPv4Interface("192.168.18.100/24")])
    assert "192.168.18.100" not in hosts and "192.168.18.1" in hosts and len(hosts) == 253


def test_fcu_urls():
    assert Path("listen", "192.168.18.111", 14513).fcu_url() == "udp://0.0.0.0:14513@"
    assert Path("probe", "10.42.0.1", 14550).fcu_url(14611) == "udp://0.0.0.0:14611@10.42.0.1:14550"


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_discovery_listen_and_probe_end_to_end():
    listen_port = _free_port()
    server_port = _free_port()
    # fake drone A pushes to our listen port
    push = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # fake drone B is a mavlink-router "Server" endpoint: answers whoever sends to it
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", server_port))
    server.settimeout(0.2)
    stop = threading.Event()

    def server_loop():
        while not stop.is_set():
            try:
                data, addr = server.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                return
            if any(not hb.is_vehicle for hb in ml.heartbeats(data)):
                server.sendto(_pm_heartbeat(15, custom_mode=9, armed=False), addr)

    t = threading.Thread(target=server_loop, daemon=True)
    t.start()
    logs = []
    d = Discovery([listen_port], ["127.0.0.1"], probe_port=server_port, probe_local_subnets=False,
                  probe_interval=0.3, stale_after=2.0, log=logs.append)
    d.start()
    try:
        deadline = time.time() + 3
        while time.time() < deadline:
            push.sendto(_pm_heartbeat(11), ("127.0.0.1", listen_port))
            rows = {r["sysid"]: r for r in d.snapshot()}
            if 11 in rows and 15 in rows:
                break
            time.sleep(0.1)
        assert rows[11]["path"].via == "listen" and rows[11]["mode"] == "GUIDED" and rows[11]["armed"]
        assert rows[15]["path"].via == "probe" and rows[15]["path"].port == server_port
        assert rows[15]["mode"] == "LAND"
        # releasing the listen path frees the port so MAVROS can bind it
        d.release(rows[11]["path"])
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", listen_port))
        s.close()
        d.reclaim(rows[11]["path"])
    finally:
        stop.set()
        t.join(timeout=1)
        d.stop()
        push.close()
        server.close()
