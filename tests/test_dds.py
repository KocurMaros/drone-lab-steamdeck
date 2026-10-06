"""DDS configuration: students' interface choice and the generated CycloneDDS XML."""
from dronelab.dds import choose_student_interface, cyclone_xml


def ifc(name, ip, flags=("BROADCAST", "MULTICAST", "UP", "LOWER_UP"), state="UP"):
    return {"name": name, "ip": ip, "prefix": 24, "flags": list(flags), "state": state}


LO = ifc("lo", "127.0.0.1", ("LOOPBACK", "UP", "LOWER_UP"), "UNKNOWN")


def test_prefers_default_route_wifi_over_bridges():
    ifs = [LO, ifc("docker0", "172.17.0.1"), ifc("wlan0", "192.168.88.250")]
    assert choose_student_interface(ifs, "wlan0")["name"] == "wlan0"
    assert choose_student_interface(ifs, None)["name"] == "wlan0"          # no default route: skip bridges


def test_skips_vpn_down_and_link_local():
    ifs = [LO, ifc("tun0", "10.8.0.2", ("POINTOPOINT", "MULTICAST", "UP")), ifc("eth0", "169.254.3.3"),
           ifc("enp1", "192.168.1.5", state="DOWN"), ifc("wlan0", "192.168.88.250")]
    assert choose_student_interface(ifs, "tun0")["name"] == "wlan0"
    assert choose_student_interface([LO], None) is None


def test_xml_pins_interface_and_participant_index():
    x = cyclone_xml([81], 0, "wlan0")
    student = x.split('<Domain Id="0">')[1].split("</Domain>")[0]
    assert '<NetworkInterface name="wlan0"/>' in student
    assert "<ParticipantIndex>auto</ParticipantIndex>" in student       # unicast peers can find the Deck
    private = x.split('<Domain Id="81">')[1]
    assert 'address="127.0.0.1"' in private
    assert "<NetworkInterface" not in cyclone_xml([81], 0, None).split('<Domain Id="81">')[0]
