import angle_servo


def test():
    s = angle_servo.CamActControllerPT(
        i2c_channel=6,
        i2c_addr=0x63,
        pan_center=90,
        pan_gain=90 / 180,
        pan_min=-45,
        pan_max=45,
        tilt_center=56,
        tilt_gain=75 / 180,
        tilt_min=-45,
        tilt_max=45,
        use_cam_act=True,
    )

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
