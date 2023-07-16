from FocuserWrapper import FocuserWrapper
from Focuser import Focuser


def test():
    # focuser = Focuser(7)
    # focuser.set(Focuser.OPT_FOCUS, 0)
    # time.sleep(3)
    # focuser.set(Focuser.OPT_FOCUS, 0)
    # time.sleep(3)
    # focuser.reset(Focuser.OPT_FOCUS)

    focuser = FocuserWrapper(6, 5, 30, 120, -5, 10, 150)
    # focuser.reset(Focuser.OPT_MOTOR_X)
    # time.sleep(5)
    # focuser.set(Focuser.OPT_MOTOR_X, 90)

    # time.sleep(5)
    # focuser.set(Focuser.OPT_MOTOR_X, 100)

    # time.sleep(5)
    # focuser.set(Focuser.OPT_MOTOR_X, 80)

    k = 0
    while k != 27:
        k = input(">> ")
        t = int(k)
        # focuser.set(Focuser.OPT_MOTOR_X, t)
        # t_after0 = focuser.get(Focuser.OPT_MOTOR_X)
        # time.sleep(0.005)
        focuser.set(Focuser.OPT_MOTOR_X, t)
        t_after = focuser.get(Focuser.OPT_MOTOR_X)
        # print(f"{t_after0}, {t_after}")
        print(f"{t_after}")


if __name__ == "__main__":
    test()
