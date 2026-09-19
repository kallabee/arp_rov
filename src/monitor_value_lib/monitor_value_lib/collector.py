from __future__ import annotations

import time
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from device_registry import DeviceRegistry

from monitor_value_lib.config import MonitorValueConfig
from monitor_value_lib.logger.csv_rotating_logger import RotatingCsvLogger
from monitor_value_lib.values import MonitorValues


class MonitorValueCollector:
    def __init__(self, cfg: MonitorValueConfig):
        self.cfg = cfg
        self._seq = 0
        self._start_monotonic = time.monotonic()
        self._logger: Optional[RotatingCsvLogger] = None
        self._registry: Optional[DeviceRegistry] = None

        self._rpi = None
        self._bme = None
        self._battery = None
        self._ads1015 = None
        self._thermocouple = None
        self._water_leak = None
        self._ds18b20 = None

        self._init_sensors()

    @classmethod
    def from_yaml(cls, path: str | Path) -> "MonitorValueCollector":
        from monitor_value_lib.config import load_config

        return cls(load_config(path))

    def _init_sensors(self) -> None:
        sensors = self.cfg.sensors

        # RPi
        flags = self.cfg.sensor_flags("rpi")
        if flags.wants_init():
            try:
                from monitor_value_lib.sensors.rpi import RpiSensor

                self._rpi = RpiSensor()
            except Exception:
                if self.cfg.sensor_init_raises("rpi"):
                    raise

        # BME280
        flags = self.cfg.sensor_flags("bme280")
        if flags.wants_init():
            s_cfg = self.cfg.sensor_cfg("bme280")
            try:
                from monitor_value_lib.sensors.bme280 import Bme280Sensor

                bb = (s_cfg.get("i2c_bitbang") or {}) if isinstance(s_cfg, dict) else {}
                linux_bus = None
                address = int(s_cfg.get("address", 0x77))
                if bool(s_cfg.get("use_device_registry", True)):
                    self._registry = self._registry or DeviceRegistry.from_src_default()
                    dev_id = str(s_cfg.get("device_id", "bme280"))
                    i2c = self._registry.get_i2c(dev_id)
                    linux_bus = i2c.linux_bus
                    address = int(i2c.addr)
                self._bme = Bme280Sensor(
                    address=address,
                    linux_bus=linux_bus,
                    use_extended_bus=bool(s_cfg.get("use_extended_bus", True)),
                    bitbang_enabled=bool((bb or {}).get("enabled", False)),
                    bitbang_scl_pin=str((bb or {}).get("scl_pin", "D23")),
                    bitbang_sda_pin=str((bb or {}).get("sda_pin", "D22")),
                )
                self._bme.init()
            except Exception:
                if self.cfg.sensor_init_raises("bme280"):
                    raise
                self._bme = None

        # Current monitor (INA226 now)
        flags = self.cfg.sensor_flags("current_monitor")
        if flags.wants_init():
            s_cfg = self.cfg.sensor_cfg("current_monitor")
            typ = str(s_cfg.get("type", "ina226")).lower()
            if typ != "ina226":
                if self.cfg.sensor_init_raises("current_monitor"):
                    raise RuntimeError(f"current_monitor.type unsupported: {typ}")
            else:
                try:
                    self._registry = DeviceRegistry.from_src_default()
                    dev_id = str(s_cfg.get("device_id", "ina226_current"))
                    i2c = self._registry.get_i2c(dev_id)
                    if i2c.linux_bus is None:
                        raise ValueError(f"device {dev_id} requires 'bus' in device_i2c.yaml")

                    from monitor_value_lib.sensors.battery_manager import BatteryManager

                    cap = s_cfg.get("capacity_wh", None)
                    capacity_wh = float(cap) if cap is not None else None
                    self._battery = BatteryManager(
                        busnum=int(i2c.linux_bus),
                        address=int(i2c.addr),
                        shunt_ohms=float(s_cfg.get("shunt_ohms", 0.002)),
                        max_expected_amps=float(s_cfg.get("max_expected_amps", 25.0)),
                        capacity_wh=capacity_wh,
                    )
                    self._battery.init()
                except Exception:
                    if self.cfg.sensor_init_raises("current_monitor"):
                        raise
                    self._battery = None

        self._init_ads1015_sensors()

        flags = self.cfg.sensor_flags("ds18b20")
        if flags.wants_init():
            s_cfg = self.cfg.sensor_cfg("ds18b20")
            try:
                from monitor_value_lib.sensors.ds18b20 import Ds18b20Sensor

                gpios = s_cfg.get("gpios", [17, 27])
                if not isinstance(gpios, (list, tuple)):
                    gpios = [17, 27]
                rescan = float(s_cfg.get("rescan_interval_sec", 10.0))
                self._ds18b20 = Ds18b20Sensor(
                    gpios=tuple(int(g) for g in gpios),
                    rescan_interval_sec=rescan,
                )
                self._ds18b20.init()
            except Exception:
                if self.cfg.sensor_init_raises("ds18b20"):
                    raise
                self._ds18b20 = None

    def _resolve_ads1015_bus_addr(self, s_cfg: dict) -> tuple[int, int]:
        bus = s_cfg.get("bus")
        addr = s_cfg.get("addr", s_cfg.get("address", 0x49))
        if bool(s_cfg.get("use_device_registry", True)):
            self._registry = self._registry or DeviceRegistry.from_src_default()
            dev_id = str(s_cfg.get("device_id", "ads1015_hull"))
            i2c = self._registry.get_i2c(dev_id)
            if i2c.linux_bus is None:
                raise ValueError(f"device {dev_id} requires 'bus' in device_i2c.yaml")
            bus = i2c.linux_bus
            addr = i2c.addr
        if bus is None:
            raise ValueError("ADS1015 requires 'bus' or device_id with bus in device_i2c.yaml")
        return int(bus), int(addr, 0) if isinstance(addr, str) else int(addr)

    def _init_ads1015_sensors(self) -> None:
        needs_ads = (
            self.cfg.sensor_flags("thermocouple_adc").wants_init()
            or self.cfg.sensor_flags("water_leak").wants_init()
        )
        if not needs_ads:
            return

        ads_cfg: dict = {}
        for sid in ("thermocouple_adc", "water_leak"):
            if self.cfg.sensor_flags(sid).wants_init():
                ads_cfg = self.cfg.sensor_cfg(sid)
                break

        try:
            from monitor_value_lib.sensors.ads1015_reader import Ads1015Reader

            bus, addr = self._resolve_ads1015_bus_addr(ads_cfg)
            self._ads1015 = Ads1015Reader(linux_bus=bus, address=addr)
            self._ads1015.init()
        except Exception:
            if self.cfg.sensor_init_raises("thermocouple_adc") or self.cfg.sensor_init_raises(
                "water_leak"
            ):
                raise
            self._ads1015 = None
            return

        flags = self.cfg.sensor_flags("thermocouple_adc")
        if flags.wants_init() and self._ads1015 is not None:
            s_cfg = self.cfg.sensor_cfg("thermocouple_adc")
            try:
                from monitor_value_lib.sensors.thermocouple_adc import ThermocoupleAdcSensor

                chs = s_cfg.get("adc_channels", [2, 3])
                if not isinstance(chs, (list, tuple)):
                    chs = [0, 2]
                self._thermocouple = ThermocoupleAdcSensor(
                    reader=self._ads1015,
                    adc_channels=[int(c) for c in chs],
                    vref_v=float(s_cfg.get("vref_v", 1.25)),
                )
                self._thermocouple.init()
            except Exception:
                if self.cfg.sensor_init_raises("thermocouple_adc"):
                    raise
                self._thermocouple = None

        flags = self.cfg.sensor_flags("water_leak")
        if flags.wants_init() and self._ads1015 is not None:
            s_cfg = self.cfg.sensor_cfg("water_leak")
            try:
                from monitor_value_lib.sensors.water_leak import (
                    WaterLeakSensor,
                    parse_water_leak_channels,
                )

                self._water_leak = WaterLeakSensor(
                    reader=self._ads1015,
                    channels=parse_water_leak_channels(s_cfg),
                )
                self._water_leak.init()
            except Exception:
                if self.cfg.sensor_init_raises("water_leak"):
                    raise
                self._water_leak = None

    def get_values(self) -> MonitorValues:
        now = datetime.now(timezone.utc)
        t = time.monotonic()
        elapsed = t - self._start_monotonic
        v = MonitorValues(seq=int(self._seq), wall_time_utc=now, elapsed_since_start_sec=float(elapsed))

        if self._rpi is not None and self.cfg.sensor_flags("rpi").wants_samples():
            try:
                vdict = self._rpi.read()
                v = replace(v, **vdict)
            except Exception:
                pass

        if self._bme is not None and self.cfg.sensor_flags("bme280").wants_samples():
            try:
                vdict = self._bme.read()
                v = replace(v, **vdict)
            except Exception:
                pass

        if self._battery is not None and self.cfg.sensor_flags("current_monitor").wants_samples():
            try:
                vdict = self._battery.read()
                v = replace(v, **vdict)
            except Exception:
                pass

        if self._thermocouple is not None and self.cfg.sensor_flags("thermocouple_adc").wants_samples():
            try:
                v = replace(v, **self._thermocouple.read())
            except Exception:
                pass

        if self._water_leak is not None and self.cfg.sensor_flags("water_leak").wants_samples():
            try:
                v = replace(v, **self._water_leak.read())
            except Exception:
                pass

        if self._ds18b20 is not None and self.cfg.sensor_flags("ds18b20").wants_samples():
            try:
                v = replace(v, **self._ds18b20.read())
            except Exception:
                pass

        # 9DoF / depth / pose are produced by the ``imu_navigation`` package (ROS or embedded
        # runner) and merged into ``MonitorValues`` in ``monitor_value_pub``.

        self._seq += 1
        return v

    def get_values_and_append(self) -> tuple[MonitorValues, Path]:
        values = self.get_values()
        if self._logger is None:
            self._logger = RotatingCsvLogger(
                log_dir=Path(self.cfg.log_dir),
                rotate_seconds=float(self.cfg.log_rotate_seconds),
                flush_each_row=True,
            )
        path = self._logger.append(values)
        return values, path

    def close(self) -> None:
        if self._logger is not None:
            self._logger.close()
            self._logger = None

