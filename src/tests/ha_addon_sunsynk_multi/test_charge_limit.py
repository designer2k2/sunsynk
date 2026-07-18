"""Test the battery charge limit feature."""

import logging
from collections.abc import Iterator
from datetime import date, timedelta
from pathlib import Path
from unittest.mock import Mock

import pytest

from ha_addon_sunsynk_multi import charge_limit
from ha_addon_sunsynk_multi.a_inverter import AInverter
from ha_addon_sunsynk_multi.charge_limit import build_charge_limit_callback
from ha_addon_sunsynk_multi.options import OPT
from ha_addon_sunsynk_multi.sensor_options import get_charge_limit_sensors, import_definitions


@pytest.fixture(autouse=True)
def _reset_state(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    """Isolate the state file and reset OPT/DEFS mutated by tests."""
    monkeypatch.setattr(charge_limit, "get_root", lambda create=False: tmp_path)
    orig_soc = OPT.battery_charge_limit_soc
    orig_days = OPT.battery_charge_limit_balance_days
    orig_defs = OPT.sensor_definitions
    OPT.sensor_definitions = "single-phase"
    import_definitions()
    yield
    OPT.battery_charge_limit_soc = orig_soc
    OPT.battery_charge_limit_balance_days = orig_days
    if OPT.sensor_definitions != orig_defs:
        OPT.sensor_definitions = orig_defs
        import_definitions()


def test_get_charge_limit_sensors_single_phase() -> None:
    """single-phase uses battery_soc."""
    OPT.sensor_definitions = "single-phase"
    import_definitions()
    soc, max_charge = get_charge_limit_sensors()
    assert soc is not None and soc.id == "battery_soc"
    assert max_charge is not None and max_charge.id == "battery_max_charge_current"


def test_get_charge_limit_sensors_three_phase_hv() -> None:
    """three-phase-hv uses battery_1_soc instead of battery_soc."""
    OPT.sensor_definitions = "three-phase-hv"
    import_definitions()
    soc, max_charge = get_charge_limit_sensors()
    assert soc is not None and soc.id == "battery_1_soc"
    assert max_charge is not None and max_charge.id == "battery_max_charge_current"


async def test_disabled_returns_none(ist: AInverter) -> None:
    """No callback when the SOC limit is 0."""
    OPT.battery_charge_limit_soc = 0
    OPT.battery_charge_limit_balance_days = 0
    assert build_charge_limit_callback(ist) is None


async def test_disabled_with_balance_days_warns(
    ist: AInverter, caplog: pytest.LogCaptureFixture
) -> None:
    """Warn if BALANCE_DAYS is set without the SOC limit."""
    OPT.battery_charge_limit_soc = 0
    OPT.battery_charge_limit_balance_days = 30
    with caplog.at_level(logging.WARNING):
        assert build_charge_limit_callback(ist) is None
    assert "BALANCE_DAYS" in caplog.text


async def test_limit_stops_charging_at_soc(ist: AInverter) -> None:
    """Crossing the limit while charging queues a 0 write and remembers the current."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 0
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 85, max_charge_sensor: 60}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {max_charge_sensor: 0}
    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] == 60


async def test_limit_resumes_after_restart(ist: AInverter) -> None:
    """A restored prev_charge_current (simulating a restart while limited) is used,
    and only cleared once a later tick confirms the write actually landed."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 0
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": 60, "last_balance": None}
    )

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 70, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {max_charge_sensor: 60}
    # Not confirmed yet (max_charge still reads 0) - must NOT be forgotten,
    # otherwise a dropped/delayed write would permanently strand the current.
    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] == 60

    # Next tick: the write landed, max_charge now reads back as restored.
    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 70, max_charge_sensor: 60}.get(s)
    )
    await cb.callback(5)

    assert ist.write_queue == {}  # nothing left to do
    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] is None


async def test_limit_resume_retried_until_confirmed(ist: AInverter) -> None:
    """A dropped/delayed restore write is retried every tick, not forgotten."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 0
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": 60, "last_balance": None}
    )

    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 70, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None

    for now in (0, 5, 10):
        ist.write_queue = {}
        await cb.callback(now)
        # max_charge still reads 0 every time (write never took effect) -
        # the addon must keep retrying, not give up after the first attempt.
        assert ist.write_queue == {max_charge_sensor: 60}
        assert charge_limit._load_state(ist.opt.ha_prefix)["prev_charge_current"] == 60


async def test_confirm_clear_skipped_while_write_pending(ist: AInverter) -> None:
    """A stale nonzero read must not clear `prev` while a write for that same
    sensor is still sitting in write_queue, unflushed - otherwise a stop-write
    queued this tick could be "confirmed away" before it's even applied."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 0
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": 60, "last_balance": None}
    )

    ist.write_queue = {max_charge_sensor: 0}  # a write is already in flight
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 50, max_charge_sensor: 60}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] == 60  # not cleared


async def test_balance_confirm_clear_skipped_while_write_pending(ist: AInverter) -> None:
    """Same guard as above, for the balance-charge unlock confirm path."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 30
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": 60, "last_balance": None}
    )

    ist.write_queue = {max_charge_sensor: 0}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 85, max_charge_sensor: 60}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] == 60  # not cleared


async def test_unknown_prev_warning_rate_limited(
    ist: AInverter, caplog: pytest.LogCaptureFixture
) -> None:
    """The 'no known previous value' warning must not spam every 5s tick."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 30
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": None, "last_balance": None}
    )

    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 50, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None

    with caplog.at_level(logging.WARNING):
        for now in (0, 5, 10):
            ist.write_queue = {}
            await cb.callback(now)

    assert caplog.text.lower().count("no previous value is known") == 1


async def test_low_limit_warns_resume_unreachable(
    ist: AInverter, caplog: pytest.LogCaptureFixture
) -> None:
    """A limit at/below the hysteresis margin can never resume - warn up front."""
    OPT.battery_charge_limit_soc = 2
    OPT.battery_charge_limit_balance_days = 0
    with caplog.at_level(logging.WARNING):
        cb = build_charge_limit_callback(ist)
    assert cb is not None
    assert "hysteresis" in caplog.text.lower()


async def test_hysteresis_dead_zone_no_resume(ist: AInverter) -> None:
    """SOC just below the limit but within the hysteresis band must not resume yet."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 0
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": 60, "last_balance": None}
    )

    ist.write_queue = {}
    # limit=80, RESUME_HYSTERESIS_SOC=2 -> resume only at soc<=78. SOC=79 is
    # below the limit but still inside the dead zone: must not resume yet.
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 79, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {}
    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] == 60  # still remembered, not lost


async def test_no_restore_without_known_previous(ist: AInverter) -> None:
    """max_charge==0 with no persisted prev value must not be touched (e.g. manual zero)."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 0
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 50, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {}


async def test_noop_when_sensors_not_read_yet(ist: AInverter) -> None:
    """No state read yet (None) must be a no-op."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 0

    ist.write_queue = {}
    ist.get_state = Mock(return_value=None)  # type: ignore[misc]
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {}


async def test_balance_charge_unlocks_limited_inverter(ist: AInverter) -> None:
    """Due for a balance charge: restore a known previous value even if SOC < limit,
    keeping it persisted until a later tick confirms the write landed."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 30
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": 60, "last_balance": None}
    )

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 85, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {max_charge_sensor: 60}
    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] == 60  # not confirmed yet
    assert state["last_balance"] is None  # not complete yet, SOC below target

    # Next tick: the write landed.
    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 85, max_charge_sensor: 60}.get(s)
    )
    await cb.callback(5)

    assert ist.write_queue == {}
    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["prev_charge_current"] is None


async def test_balance_charge_completes_at_target(ist: AInverter) -> None:
    """Reaching the balance target SOC persists today's date."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 30
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": None, "last_balance": None}
    )

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {
            soc_sensor: charge_limit.BALANCE_TARGET_SOC,
            max_charge_sensor: 60,
        }.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {}  # already unlocked, nothing to write
    state = charge_limit._load_state(ist.opt.ha_prefix)
    assert state["last_balance"] == date.today()


async def test_balance_not_due_uses_normal_limit(ist: AInverter) -> None:
    """A recent balance date means normal limiting still applies."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 30
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": None, "last_balance": date.today()}
    )

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 85, max_charge_sensor: 60}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {max_charge_sensor: 0}


async def test_balance_due_without_known_previous_warns(
    ist: AInverter, caplog: pytest.LogCaptureFixture
) -> None:
    """Due for balancing but the addon doesn't know a safe value: warn, don't guess."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 30
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix, {"prev_charge_current": None, "last_balance": None}
    )

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 50, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    with caplog.at_level(logging.WARNING):
        await cb.callback(0)

    assert ist.write_queue == {}
    assert "balance charge" in caplog.text.lower()


async def test_balance_days_old_enough_is_due(ist: AInverter) -> None:
    """A last_balance older than the interval is due again."""
    OPT.battery_charge_limit_soc = 80
    OPT.battery_charge_limit_balance_days = 30
    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    assert soc_sensor is not None and max_charge_sensor is not None

    charge_limit._save_state(
        ist.opt.ha_prefix,
        {"prev_charge_current": 60, "last_balance": date.today() - timedelta(days=31)},
    )

    ist.write_queue = {}
    ist.get_state = Mock(  # type: ignore[misc]
        side_effect=lambda s: {soc_sensor: 85, max_charge_sensor: 0}.get(s)
    )
    cb = build_charge_limit_callback(ist)
    assert cb is not None
    await cb.callback(0)

    assert ist.write_queue == {max_charge_sensor: 60}


def test_load_all_state_handles_missing_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Missing state file loads as empty, and a never-seen inverter is seeded
    as "just balanced today" (see test_first_load_seeds_last_balance_today)."""
    monkeypatch.setattr(charge_limit, "get_root", lambda create=False: tmp_path)
    assert charge_limit._load_all_state() == {}
    assert charge_limit._load_state("ss1") == {
        "prev_charge_current": None,
        "last_balance": date.today(),
    }


def test_first_load_seeds_last_balance_today(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A brand-new inverter must not trigger an immediate balance charge just
    because BATTERY_CHARGE_LIMIT_BALANCE_DAYS was enabled from day one - the
    first balance charge should only happen after the configured interval."""
    monkeypatch.setattr(charge_limit, "get_root", lambda create=False: tmp_path)
    state = charge_limit._load_state("ss1")
    assert state["last_balance"] == date.today()
    # and it's persisted, not just returned in memory
    assert charge_limit._load_all_state()["ss1"]["last_balance"] == date.today().isoformat()


def test_load_all_state_handles_corrupt_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Corrupt state file loads as empty, no crash."""
    monkeypatch.setattr(charge_limit, "get_root", lambda create=False: tmp_path)
    (tmp_path / charge_limit.STATE_FILE).write_text("not json", encoding="utf-8")
    assert charge_limit._load_all_state() == {}
