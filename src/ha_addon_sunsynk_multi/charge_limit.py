"""Battery charge limit: stop charging at a configured SOC, with periodic
balance charges to 100% for cell balancing."""

import json
import logging
import os
from datetime import date, datetime
from pathlib import Path
from typing import Any

from ha_addon_sunsynk_multi.a_inverter import AInverter
from ha_addon_sunsynk_multi.helpers import get_root
from ha_addon_sunsynk_multi.options import OPT
from ha_addon_sunsynk_multi.sensor_options import get_charge_limit_sensors
from ha_addon_sunsynk_multi.timer_callback import AsyncCallback
from sunsynk.helpers import as_num

_LOGGER = logging.getLogger(__name__)

STATE_FILE = "charge_limit_state.json"
BALANCE_TARGET_SOC = 99
"""Treat SOC>=this as "fully charged" - some BMS/voltage estimations never
quite report an exact 100."""
RESUME_HYSTERESIS_SOC = 2
"""Only resume charging once SOC drops this many percentage points below the
limit, to avoid rapidly toggling (and re-writing) the charge current if SOC
hovers right at the boundary. SOC is reported as a whole percentage, so this
must be >=2 to create an actual dead zone (a margin of 1 has no integer
value strictly between "stop" and "resume")."""


def _state_path() -> Path:
    return get_root(create=True) / STATE_FILE


def _load_all_state() -> dict[str, Any]:
    """Read the whole state file, tolerating a missing/corrupt file."""
    pth = _state_path()
    if not pth.exists():
        return {}
    try:
        return dict(json.loads(pth.read_text(encoding="utf-8")))
    except Exception as err:  # pylint:disable=broad-except
        _LOGGER.warning("Could not read %s, ignoring: %s", pth, err)
        return {}


def _save_all_state(all_state: dict[str, Any]) -> None:
    """Write the whole state file atomically (write to a temp file, then rename)."""
    pth = _state_path()
    tmp = pth.with_suffix(".tmp")
    tmp.write_text(json.dumps(all_state), encoding="utf-8")
    os.replace(tmp, pth)


def _load_state(ha_prefix: str) -> dict[str, Any]:
    """Load the charge limit state for a single inverter."""
    raw = _load_all_state().get(ha_prefix)
    if raw is None:
        # Brand new inverter: assume "just balanced" so enabling the feature
        # doesn't immediately force a full charge - the first balance charge
        # (if configured) only happens after the configured interval.
        state = {"prev_charge_current": None, "last_balance": date.today()}
        _save_state(ha_prefix, state)
        return state
    last_balance = raw.get("last_balance")
    return {
        "prev_charge_current": raw.get("prev_charge_current"),
        "last_balance": (
            datetime.strptime(last_balance, "%Y-%m-%d").date() if last_balance else None
        ),
    }


def _save_state(ha_prefix: str, state: dict[str, Any]) -> None:
    """Persist the charge limit state for a single inverter."""
    all_state = _load_all_state()
    last_balance = state["last_balance"]
    all_state[ha_prefix] = {
        "prev_charge_current": state["prev_charge_current"],
        "last_balance": last_balance.isoformat() if last_balance else None,
    }
    _save_all_state(all_state)


def build_charge_limit_callback(ist: AInverter) -> AsyncCallback | None:
    """Build the battery charge limit callback for an inverter, if enabled."""
    if not OPT.battery_charge_limit_soc:
        if OPT.battery_charge_limit_balance_days:
            _LOGGER.warning(
                "BATTERY_CHARGE_LIMIT_BALANCE_DAYS is set but "
                "BATTERY_CHARGE_LIMIT_SOC is 0 - the balance charge will not run"
            )
        return None

    soc_sensor, max_charge_sensor = get_charge_limit_sensors()
    if soc_sensor is None or max_charge_sensor is None:
        _LOGGER.error(
            "Battery charge limit enabled, but could not find the required "
            "sensors for definitions %s",
            OPT.sensor_definitions,
        )
        return None

    if OPT.battery_charge_limit_soc <= RESUME_HYSTERESIS_SOC:
        _LOGGER.warning(
            "%s: BATTERY_CHARGE_LIMIT_SOC (%s%%) is at or below the %s%% resume "
            "hysteresis margin - once charging stops it may never resume automatically",
            ist.opt.ha_prefix,
            OPT.battery_charge_limit_soc,
            RESUME_HYSTERESIS_SOC,
        )

    state = _load_state(ist.opt.ha_prefix)
    last_unknown_prev_warn: date | None = None

    async def callback_charge_limit(now: int) -> None:
        nonlocal last_unknown_prev_warn
        soc_raw = ist.get_state(soc_sensor)
        max_charge_raw = ist.get_state(max_charge_sensor)
        if soc_raw is None or max_charge_raw is None:
            return  # not read yet

        soc = as_num(soc_raw)
        max_charge = as_num(max_charge_raw)
        limit = OPT.battery_charge_limit_soc
        balance_days = OPT.battery_charge_limit_balance_days
        prev = state["prev_charge_current"]

        last_balance = state["last_balance"]
        balancing = bool(balance_days) and (
            last_balance is None or (date.today() - last_balance).days >= balance_days
        )

        if balancing:
            if max_charge == 0:
                if prev is None:
                    today = date.today()
                    if last_unknown_prev_warn != today:  # once per day, not every 5s
                        last_unknown_prev_warn = today
                        _LOGGER.warning(
                            "%s: due for a balance charge, but the charge current "
                            "is 0 and no previous value is known - not touching it",
                            ist.opt.ha_prefix,
                        )
                else:
                    # Keep `prev` persisted until we observe the write actually
                    # landed (max_charge > 0 on a later tick) - the flush queue
                    # can silently drop a failed write, and we must be able to
                    # retry rather than forget the value forever.
                    ist.write_queue[max_charge_sensor] = prev
                    _LOGGER.info(
                        "%s: balance charge started, charge limit suspended (%sA)",
                        ist.opt.ha_prefix,
                        prev,
                    )
            elif prev is not None and max_charge_sensor not in ist.write_queue:
                # Confirmed: the register is non-zero, and there's no write
                # still in flight for it, so this read isn't stale relative
                # to a pending change (e.g. a stop-write queued moments ago).
                state["prev_charge_current"] = None
                _save_state(ist.opt.ha_prefix, state)
            if soc >= BALANCE_TARGET_SOC:
                state["last_balance"] = date.today()
                _save_state(ist.opt.ha_prefix, state)
                _LOGGER.info(
                    "%s: balance charge to %s%% complete", ist.opt.ha_prefix, soc
                )
            return

        if soc >= limit and max_charge > 0:
            state["prev_charge_current"] = max_charge
            _save_state(ist.opt.ha_prefix, state)
            ist.write_queue[max_charge_sensor] = 0
            _LOGGER.info(
                "%s: SOC %s%% reached charge limit %s%%, stopping charge (was %sA)",
                ist.opt.ha_prefix,
                soc,
                limit,
                max_charge,
            )
        elif soc <= limit - RESUME_HYSTERESIS_SOC and max_charge == 0 and prev is not None:
            # Same reasoning as the balance-charge restore above: don't clear
            # `prev` until a later tick confirms max_charge > 0.
            ist.write_queue[max_charge_sensor] = prev
            _LOGGER.info(
                "%s: SOC %s%% below charge limit %s%%, resuming charge (%sA)",
                ist.opt.ha_prefix,
                soc,
                limit,
                prev,
            )
        elif max_charge > 0 and prev is not None and max_charge_sensor not in ist.write_queue:
            # Confirmed: the resume write (above, or a manual one) landed,
            # and nothing is still in flight that could make this read stale.
            state["prev_charge_current"] = None
            _save_state(ist.opt.ha_prefix, state)

    return AsyncCallback(
        name=f"charge_limit {ist.opt.ha_prefix}",
        every=5,
        callback=callback_charge_limit,
    )
