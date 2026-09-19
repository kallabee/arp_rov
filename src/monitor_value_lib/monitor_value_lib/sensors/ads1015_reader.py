from __future__ import annotations

from typing import Any, Optional


class Ads1015Reader:
    """ADS1015 single-ended voltage reader (ExtendedI2C / Blinka)."""

    def __init__(self, linux_bus: int, address: int = 0x49):
        self.linux_bus = int(linux_bus)
        self.address = int(address)
        self._ads: Any = None
        self._channels: dict[int, Any] = {}

    def init(self) -> None:
        if self._ads is not None:
            return
        try:
            import adafruit_ads1x15.ads1015 as ADS  # type: ignore
            from adafruit_ads1x15.analog_in import AnalogIn  # type: ignore
            from adafruit_extended_bus import ExtendedI2C  # type: ignore

            i2c = ExtendedI2C(self.linux_bus)
            self._ads = ADS.ADS1015(i2c, address=self.address)
            self._AnalogIn = AnalogIn
        except Exception as e:
            raise RuntimeError(
                f"ADS1015 init failed (bus={self.linux_bus} addr=0x{self.address:02x}): {e}"
            ) from e

    def _channel(self, adc_ch: int) -> Any:
        if self._ads is None:
            self.init()
        ch = int(adc_ch)
        if ch not in self._channels:
            self._channels[ch] = self._AnalogIn(self._ads, ch)
        return self._channels[ch]

    def read_voltage(self, adc_ch: int) -> float:
        return float(self._channel(adc_ch).voltage)
