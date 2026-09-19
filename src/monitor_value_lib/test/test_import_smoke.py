# Copyright 2026
# SPDX-License-Identifier: Apache-2.0


def test_import_monitor_value_lib() -> None:
    import monitor_value_lib  # noqa: F401

    assert monitor_value_lib.MonitorValueCollector is not None
