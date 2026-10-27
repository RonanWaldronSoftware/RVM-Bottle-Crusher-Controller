#!/usr/bin/env python3
"""Non-destructive simulation test for BB01 reed-gated bottle cycle."""
from __future__ import annotations

import tempfile
import time
from pathlib import Path

import controller_kiosk as app


def settle_reed(controller, value: int):
    app.GPIO.simulate_set_pin(app.PIN_REED, value)
    controller._update_reed_switch()
    time.sleep(app.DEBOUNCE_SECONDS + 0.03)
    controller._update_reed_switch()


def main() -> int:
    app.load_gpio(True)
    controller = app.MachineController(simulate=True)
    controller._simulate_reed_activity = lambda: None
    controller.setup()
    closed = controller.reed_closed_level
    opened = controller.reed_open_level
    assert closed is not None
    assert opened is not None
    try:
        settle_reed(controller, closed)

        # Idle/manual opening must never start the crusher.
        settle_reed(controller, opened)
        assert not controller.motor_on
        assert controller.flap_open_count == 1

        settle_reed(controller, closed)
        assert controller.arm_motor_for_next_flap_open()
        assert controller.motor_trigger_armed

        # The first armed opening starts one 15-second cycle and consumes arm.
        settle_reed(controller, opened)
        assert controller.motor_on
        assert not controller.motor_trigger_armed
        first_deadline = controller.motor_off_deadline
        assert first_deadline > time.monotonic()

        # Reopening without a new valid scan must not reset the deadline.
        settle_reed(controller, closed)
        settle_reed(controller, opened)
        assert controller.motor_on
        assert controller.motor_off_deadline == first_deadline

        # Prove the timed stop path without waiting 15 seconds.
        controller.motor_off_deadline = time.monotonic() + 0.05
        time.sleep(0.08)
        controller._update_motor_timer()
        assert not controller.motor_on
        assert app.GPIO.input(app.PIN_MOTOR_SSR) == app.MOTOR_OFF_LEVEL_VALUE

        controller.solenoid_unlock_pulse()
        time.sleep(app.SOLENOID_PULSE_SECONDS + 0.15)
        assert not controller.solenoid_busy
        assert app.GPIO.input(app.PIN_SOL_PWM) == app.GPIO.LOW
        assert app.GPIO.input(app.PIN_SOL_DIR) == app.SOLENOID_UNLOCK_DIR_VALUE
    finally:
        controller.cleanup()

    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        product_file = root / "products.csv"
        option_file = root / "options.csv"
        product_file.write_text(
            "barcode,product_name,brand,subbrand,capacity\n",
            encoding="utf-8",
        )
        option_file.write_text(
            "brand,subbrand,capacity,product_name\n"
            "Corona,Zero,330ml,Corona Zero 330ml\n",
            encoding="utf-8",
        )
        catalogue = app.ProductCatalogue(product_file)
        catalogue.save_mapping(
            "1234567890123",
            "Corona Zero 330ml",
            "Corona",
            "Zero",
            "330ml",
        )
        assert catalogue.find("1234567890123") == "Corona Zero 330ml"
        assert catalogue.find("9999999999999") is None
        options = app.ProductOptions(option_file)
        assert options.brands() == ["Corona"]
        transaction_file = root / "transactions.csv"
        transaction_log = app.BottleTransactionLog(transaction_file)
        transaction_id = transaction_log.append(
            "IMX219", "1234567890123", "EAN13", "Corona Zero 330ml"
        )
        assert transaction_id
        content = transaction_file.read_text(encoding="utf-8")
        assert "scan_recorded_before_unlock" in content
        assert "pending_ir_sensor" in content

    print(
        "PASS: unarmed flap is safe, valid-cycle reed trigger is one-shot, "
        "motor timer stops output, MD13S solenoid pulse and CSV logic pass"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
