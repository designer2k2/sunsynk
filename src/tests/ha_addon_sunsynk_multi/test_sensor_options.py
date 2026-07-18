"""States."""

import logging

from ha_addon_sunsynk_multi.sensor_options import OPT, SOPT

_LOGGER = logging.getLogger(__name__)


def test_opt1() -> None:
    """Sensors."""
    SOPT.init_sensors()
    assert sorted(s.id for s in SOPT) == ["device_type", "protocol", "serial"]

    OPT.sensors = ["prog1_time"]
    OPT.sensors_first_inverter = []
    SOPT.init_sensors()
    assert sorted(s.id for s in SOPT) == [
        "device_type",
        "prog1_time",
        "prog2_time",
        "prog3_time",
        "prog4_time",
        "prog5_time",
        "prog6_time",
        "protocol",
        "serial",
    ]
    assert sorted(s.id for s in SOPT if SOPT[s].visible) == [
        "prog1_time",
    ]


def test_opt_1st() -> None:
    """Sensors."""
    OPT.sensors = ["serial"]
    OPT.sensors_first_inverter = ["prog1_time", "device_type"]
    SOPT.init_sensors()

    assert sorted(s.id for s in SOPT) == [
        "device_type",
        "prog1_time",
        "prog2_time",
        "prog3_time",
        "prog4_time",
        "prog5_time",
        "prog6_time",
        "protocol",
        "serial",
    ]
    assert sorted(s.id for s in SOPT if SOPT[s].visible) == [
        "device_type",
        "prog1_time",
        "serial",
    ]
    assert sorted(s.id for s in SOPT if SOPT[s].first) == [
        "prog1_time",
        "prog2_time",
        "prog3_time",
        "prog4_time",
        "prog5_time",
        "prog6_time",
    ]


def test_opt_1st_visible() -> None:
    """Sensors."""
    OPT.sensors = []
    OPT.sensors_first_inverter = ["device_type"]
    SOPT.init_sensors()

    assert sorted(s.id for s in SOPT) == [
        "device_type",
        "protocol",
        "serial",
    ]
    assert sorted(s.id for s in SOPT if SOPT[s].visible) == [
        "device_type",
    ]
    assert sorted(s.id for s in SOPT if SOPT[s].first) == []


def test_charge_limit_sensors_not_added_when_disabled() -> None:
    """The charge limit sensors are absent unless the feature is enabled."""
    OPT.sensors = []
    OPT.sensors_first_inverter = []
    OPT.battery_charge_limit_soc = 0
    SOPT.init_sensors()
    assert "battery_soc" not in [s.id for s in SOPT]
    assert "battery_max_charge_current" not in [s.id for s in SOPT]


def test_charge_limit_sensors_force_added() -> None:
    """Enabling the charge limit force-tracks its sensors, hidden."""
    OPT.sensors = []
    OPT.sensors_first_inverter = []
    OPT.battery_charge_limit_soc = 80
    try:
        SOPT.init_sensors()
        ids = [s.id for s in SOPT]
        assert "battery_soc" in ids
        assert "battery_max_charge_current" in ids
        for sen in SOPT:
            if sen.id in ("battery_soc", "battery_max_charge_current"):
                assert SOPT[sen].visible is False
    finally:
        OPT.battery_charge_limit_soc = 0
        SOPT.init_sensors()


def test_charge_limit_sensors_first_flag_cleared() -> None:
    """A sensor listed under SENSORS_FIRST_INVERTER must still be read on every inverter once the charge limit needs it (or it'd never be scheduled for inverters after the first)."""
    OPT.sensors = []
    OPT.sensors_first_inverter = ["battery_soc"]
    OPT.battery_charge_limit_soc = 80
    try:
        SOPT.init_sensors()
        sen = next(s for s in SOPT if s.id == "battery_soc")
        assert SOPT[sen].first is False
    finally:
        OPT.battery_charge_limit_soc = 0
        OPT.sensors_first_inverter = []
        SOPT.init_sensors()
