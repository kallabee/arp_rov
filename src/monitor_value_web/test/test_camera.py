from monitor_value_web.camera import (
    CameraError,
    LIVE_GROUPS_BY_BACKEND,
    RESTART_GROUPS_BY_BACKEND,
    build_patch,
    mediamtx_runtime_patch,
    merge_state,
    parse_request,
    summarize_conf,
    to_libcamera_controls,
    to_mediamtx_patch,
    zoom_id_from_conf,
)


def test_zoom_crop_restarts():
    patch, restart = build_patch({"zoom": "crop"})
    assert restart is True
    assert patch["rpiCameraMode"] == "1536:864"
    assert patch["rpiCameraWidth"] == 1536
    assert patch["rpiCameraHeight"] == 864


def test_exposure_and_wb_are_live_on_mediamtx():
    partial, groups = parse_request(
        {
            "exposure": {"mode": "me", "shutter_us": 8000, "gain": 2.5},
            "wb": {"mode": "manual", "gains": [2.0, 1.4]},
        }
    )
    assert groups.isdisjoint(RESTART_GROUPS_BY_BACKEND["mediamtx"])
    assert groups <= LIVE_GROUPS_BY_BACKEND["mediamtx"]
    patch = to_mediamtx_patch(partial)
    assert patch["rpiCameraShutter"] == 8000
    assert patch["rpiCameraGain"] == 2.5
    assert patch["rpiCameraAWB"] == "custom"
    assert patch["rpiCameraAWBGains"] == [2.0, 1.4]


def test_ae_clears_manual_exposure():
    patch = to_mediamtx_patch(
        parse_request({"exposure": {"mode": "ae", "ev": 0.5}})[0]
    )
    assert patch["rpiCameraShutter"] == 0
    assert patch["rpiCameraGain"] == 0
    assert patch["rpiCameraEV"] == 0.5


def test_ae_switch_restarts_mediamtx():
    partial, groups = parse_request({"exposure": {"mode": "ae"}})
    patch, restart = mediamtx_runtime_patch(
        partial,
        groups,
        {"sourceOnDemandStartTimeout": "10s", "rpiCameraMode": "2304:1296"},
    )
    assert restart is True
    assert patch["rpiCameraShutter"] == 0
    assert patch["rpiCameraGain"] == 0
    assert patch["sourceOnDemandStartTimeout"] == "11s"
    assert patch["rpiCameraMode"] == "2304:1296"
    assert patch["rpiCameraAfWindow"] == "0,0,1,1"


def test_focus_window_restarts():
    patch, restart = build_patch({"focus": {"mode": "continuous", "window": [0.3, 0.3, 0.4, 0.4]}})
    assert restart is True
    assert patch["rpiCameraAfMode"] == "continuous"
    assert patch["rpiCameraAfWindow"] == "0.3,0.3,0.4,0.4"


def test_full_af_window():
    patch, restart = build_patch({"focus": {"window": ""}})
    assert restart is True
    assert patch["rpiCameraAfWindow"] == "0,0,1,1"


def test_zoom_id_from_conf():
    assert zoom_id_from_conf({"rpiCameraMode": "4608:2592"}) == "full"
    assert zoom_id_from_conf({"rpiCameraMode": "1536:864"}) == "crop"
    assert zoom_id_from_conf({}) == "binned"


def test_summarize_me():
    summary = summarize_conf({"rpiCameraShutter": 1000, "rpiCameraGain": 0, "rpiCameraAWB": "auto"})
    assert summary["exposure"]["mode"] == "me"


def test_invalid_zoom():
    try:
        build_patch({"zoom": "digital"})
    except CameraError:
        return
    raise AssertionError("expected CameraError")


def test_momo_maps_libcamera_controls():
    partial, groups = parse_request(
        {
            "zoom": "crop",
            "focus": {"mode": "manual", "lens_position": 2.0},
            "exposure": {"mode": "me", "shutter_us": 8000, "gain": 2.5},
            "wb": {"mode": "manual", "gains": [2.0, 1.4]},
        }
    )
    assert groups == RESTART_GROUPS_BY_BACKEND["momo"]
    state = merge_state({}, partial)
    controls = dict(to_libcamera_controls(state))
    assert controls["AfMode"] == "Manual"
    assert controls["LensPosition"] == "2.0"
    assert controls["ExposureTimeMode"] == "Manual"
    assert controls["ExposureTime"] == "8000"
    assert controls["ColourGains"] == "2.0,1.4"
    assert state["width"] == 1536
    assert state["height"] == 864
