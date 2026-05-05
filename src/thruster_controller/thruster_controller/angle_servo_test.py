import angle_servo
from pathlib import Path

from device_registry import DeviceRegistry
from thruster_controller.cam_act_config import CamActPTConfig


def test():
    reg = DeviceRegistry.from_src_default()
    d = reg.get_i2c("cam_act")
    if d.linux_bus is None:
        raise ValueError("device cam_act requires 'bus' in device_i2c.yaml")

    cam_cls = getattr(angle_servo, "CamActControllerPT", None)
    if cam_cls is None:
        print(
            "CamActControllerPT is not available (see thruster_controller.cam_act_controllers). "
            f"Registry I2C: bus={d.linux_bus} addr=0x{d.addr:02x}"
        )
        return

    cfg_path = Path(__file__).resolve().parent / "cam_act_pt.yaml"
    cfg = CamActPTConfig.load(cfg_path)

    s = cam_cls(cfg, linux_bus=d.linux_bus, i2c_addr=d.addr)

    k = 0
    p = 0
    t = 0

    while k != 27:
        k = input(">> ")
        if k[0] == "p":
            p = float(k[1:])
        elif k[0] == "t":
            t = float(k[1:])

        s.move_pan(p)
        s.move_tilt(t)


if __name__ == "__main__":
    test()
