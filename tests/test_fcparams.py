"""ArduPilot parameter review for RC takeover / fail-safes (dronelab.fcparams)."""
import os

import yaml

from dronelab import fcparams
from dronelab.config import DEFAULT_CONFIG, Config

REPO = os.path.join(os.path.dirname(__file__), "..")
LAB_DRONE = os.path.join(REPO, "drones", "edu10", "drone10.params")


def cfg():
    return Config(yaml.safe_load(open(DEFAULT_CONFIG)))


def by_name(findings):
    return fcparams.fixes([f for f in findings if f.level == "warn"])


def test_lab_drone_params_outdoor():
    p = fcparams.load_param_file(LAB_DRONE)            # QGroundControl format
    assert p["RC_OPTIONS"] == 544 and p["FS_GCS_ENABLE"] == 0 and p["SYSID_MYGCS"] == 255
    ds = cfg().drone(10, "outdoor_field")
    fix = by_name(fcparams.review(p, ds))
    assert fix["RC_OPTIONS"] == 546                    # keeps the existing bits, adds "ignore MAVLink overrides"
    assert fix["FS_GCS_ENABLE"] == fcparams.FS_GCS_RTL
    assert fix["FS_OPTIONS"] == 16
    assert fix["FENCE_ENABLE"] == 1 and fix["FENCE_TYPE"] == 3
    assert fix["FENCE_ALT_MAX"] >= ds.fence.zmax + 10
    assert fix["FENCE_RADIUS"] >= fcparams.fence_extent(ds.fence)
    assert fix["WPNAV_SPEED"] == 300                   # 10 m/s position targets -> the gate's 3 m/s
    assert "SYSID_MYGCS" not in fix and "FS_THR_ENABLE" not in fix and "RTL_ALT" not in fix
    infos = [f.text for f in fcparams.review(p, ds) if f.level == "info"]
    assert any("channel 8" in t and "LOITER" in t for t in infos)


def test_lab_drone_params_indoor():
    p = fcparams.load_param_file(LAB_DRONE)
    ds = cfg().drone(10)                               # indoor_lab
    fix = by_name(fcparams.review(p, ds))
    assert fix["FS_GCS_ENABLE"] == fcparams.FS_GCS_LAND
    assert "FENCE_ENABLE" not in fix and "RTL_ALT" not in fix      # indoors: info only, never lower RTL_ALT
    assert fix["WPNAV_SPEED"] == 100


def test_fixed_params_are_clean():
    p = fcparams.load_param_file(LAB_DRONE)
    ds = cfg().drone(10, "outdoor_field")
    p.update(fcparams.fixes(fcparams.review(p, ds)))
    assert [f.text for f in fcparams.review(p, ds) if f.level == "warn"] == []


def test_no_pilot_mode_on_switch_and_wrong_gcs_id():
    ds = cfg().drone(10)
    p = {f"FLTMODE{i}": 4 for i in range(1, 7)}
    p.update(RC_OPTIONS=2, SYSID_MYGCS=254, FS_GCS_ENABLE=5, FS_OPTIONS=24)
    f = fcparams.review(p, ds)
    texts = " ".join(x.text for x in f if x.level == "warn")
    assert "cannot take over" in texts
    assert by_name(f) == {"SYSID_MYGCS": 255}


def test_param_files_match_review():
    """drones/params/*-rc-takeover.param (for Mission Planner) must give a clean review."""
    for name, profile in (("outdoor", "outdoor_field"), ("indoor", None)):
        p = fcparams.load_param_file(LAB_DRONE)
        p.update(fcparams.load_param_file(os.path.join(REPO, "drones", "params", f"{name}-rc-takeover.param")))
        ds = cfg().drone(10, profile)
        # every warning the file can fix is fixed (EKF height source is a manual, drone-specific change)
        assert [f.text for f in fcparams.review(p, ds) if f.level == "warn" and f.fix] == [], name
        assert int(p["RC_OPTIONS"]) & 2 and int(p["FS_OPTIONS"]) & 16


def test_indoor_barometer_height_is_flagged():
    p = fcparams.load_param_file(LAB_DRONE)            # EK3_SRC1_POSZ=1 (baro)
    warns = [f for f in fcparams.review(p, cfg().drone(10)) if f.level == "warn"]
    assert any("EK3_SRC1_POSZ=1" in f.text and not f.fix for f in warns)     # explained, never auto-changed
    assert not any("EK3_SRC1_POSZ" in f.text for f in fcparams.review(p, cfg().drone(10, "outdoor_field")))
