from scipy import *
from scipy import linalg
import numpy as np

import yaml


class thruster_rot_conv:
    def __init__(self, thruster_param):
        self.thruster_param = thruster_param

        a = thruster_param["z_angle"]  # [degree]
        self.ar = [np.radians(b) if b else 0 for b in a]  # [rad]

        self.sign = thruster_param["polar"]
        self.effi = thruster_param["efficiency"]

        self.prepare_xyty_matrix()
        self.prepare_er_matrix()

    def prepare_xyty_matrix(self):
        # 'a' vector, [x, y, wy, wz]
        # 'b' vector, [thruster 1's force, 2, 5, 6]
        # First, create 4 equations for 'a' and 'b'.
        # Second, chage the equations into matrix equation.
        #   b = C * a
        #   C is 4x4 matrix.
        # Finally, calc inv C = C^-1 matrix.
        #   a = C^-1 * b

        ar = self.ar

        self.C = np.array(
            [
                [-np.sin(ar[0]), np.sin(ar[1]), np.sin(ar[4]), -np.sin(ar[5])],
                [-np.cos(ar[0]), np.cos(ar[1]), np.cos(ar[4]), -np.cos(ar[5])],
                [-np.sin(ar[0]), np.sin(ar[1]), -np.sin(ar[4]), np.sin(ar[5])],
                [-1, 1, -1, 1],
            ]
        )
        # print(self.PT)

        self.xyty_mat = linalg.pinv(self.C)
        self.xyty_mat = self.xyty_mat / np.max(self.xyty_mat)

    def prepare_er_matrix(self):
        # Movement of elevation and rolling.
        # z=elevation: +Z-axis = upper translation.
        # wx=roll: +X-axis rotation.
        # [z, wx]^-1 = er * [P3, P4]

        er = np.array([[1, 1], [1, -1]])
        self.er_mat = np.linalg.inv(er)
        self.er_mat = self.er_mat / np.max(self.er_mat)

    def xyty_to_fr_thrusters(self, xyty: np.array) -> np.array:
        # Converts 4-rows vector for x, y tlanslation and tilt, yaw rotation, [x, y, wy, wz] into 4-rows vector, [thruster 1, 2, 5, 6] for front and rear thrusters' rotation.
        assert xyty.dtype == "float64" or xyty.dtype == "float32"
        assert np.max(np.abs(xyty)) <= 1.0

        # frt stands for front and rear thrusters.
        frt = self.xyty_mat @ xyty

        return frt

    def er_to_m_thrusters(self, er: np.array) -> np.array:
        """
        Converts 2-rows vector for elevation and roll, [e, r] into 2-rows vector, [thrusters 3, 4] for middle thrusters's rotation.
        -1 <= [e, r] <= 1
        -1 <= [thrusterd 3, 4] <= 1
        """
        assert er.dtype == "float64" or er.dtype == "float32"
        assert np.max(np.abs(er)) <= 1.0

        t = np.sum(np.abs(er))
        if t > 1:
            # Normalize |v| not to exceed 1.
            v_new = er / t
            # print(f't = {t}, v = {v}, new v = {v_new}')
            er = v_new

        ert = self.er_mat @ er.transpose()

        return ert.transpose()

    def to_all_thrusters(self, xyzwxwywz: np.array) -> np.array:
        # Convert 6 axis input (x, y, z parallel move and wx, wy, wz rotation) into 6 thrusters forces.
        # x, y, wy, wz -> front and back 4 thruster's forces
        # z, wx -> middle 2 thruster's forces

        # Calc forces for front and rear thrusters from x, y, wy, wz input.
        # xyty = np.concatenate([xyzwxwywz[:2], np.array([xyzwxwywz[4:5]])])
        xyty = np.concatenate([xyzwxwywz[:2], xyzwxwywz[4:6]])
        frt = self.xyty_to_fr_thrusters(xyty)

        # Calc forces for middle thrusters from z and wx input.
        er = np.array([xyzwxwywz[2:4]])
        ert = self.er_to_m_thrusters(er)

        allt = np.concatenate([frt[:2], ert[0, :], frt[2:]])
        # print(f'xyp = {xyp}, re = {roll_elev}, input = {v}, output = {p}')

        for i , a in enumerate(allt):
            allt[i]= self.sign[i]*self.effi[i]*a

        # Regulate thruster input value in [-1, 1].
        maxv = np.max(np.abs(allt))
        if maxv > 1.0:
            allt = allt / maxv

        return allt

    def get_num(self) -> int:
        return len(self.thruster_param["z_angle"])

    @staticmethod
    def load_param(yaml_path: str):
        with open(yaml_path, "r") as f:
            return yaml.safe_load(f)
