from thruster_rot_conv import *
import os

param_path = os.path.join(os.path.dirname(__file__), "thruster_param.yaml")

# tc = thruster_rot_conv(data)
tc = thruster_rot_conv(thruster_rot_conv.load_param(param_path))
print(tc.xyty_to_fr_thrusters(np.array([1.0, 0, 0, 0], dtype=np.float32)))
print(tc.xyty_to_fr_thrusters(np.array([0, 1.0, 0, 0], dtype=np.float32)))
print(tc.xyty_to_fr_thrusters(np.array([1.0, 1, 0, 0], dtype=np.float32)))
print(tc.xyty_to_fr_thrusters(np.array([0, 0, 1.0, 0], dtype=np.float32)))
print(tc.xyty_to_fr_thrusters(np.array([1.0, 1, 1, 0], dtype=np.float32)))
print(tc.xyty_to_fr_thrusters(np.array([0, 1.0, 1, 0], dtype=np.float32)))

print("...")
print(tc.xyty_to_fr_thrusters(np.array([0, 0, 1, 0], dtype=np.float32)))
print(tc.xyty_to_fr_thrusters(np.array([1, 0,1, 0], dtype=np.float32)))
print(tc.xyty_to_fr_thrusters(np.array([0, 1,1, 0], dtype=np.float32)))

# print(tc.roll_elev_move_to_thruster(np.array([1, 0])))
# print(tc.roll_elev_move_to_thruster(np.array([0, 1])))
# print(tc.roll_elev_move_to_thruster(np.array([0.5, 0.5])))
# print(tc.roll_elev_move_to_thruster(np.array([1, 0.5])))
# print(tc.roll_elev_move_to_thruster(np.array([1, 1])))

print('...')
print(tc.to_all_thrusters(np.array([1, 0,0,0,0,0], dtype=np.float32)))
print(tc.to_all_thrusters(np.array([0, 1,0,0,0,0], dtype=np.float32)))
print(tc.to_all_thrusters(np.array([0, 0,1,0,0,0], dtype=np.float32)))
print(tc.to_all_thrusters(np.array([1, 0,0,1,0,0], dtype=np.float32)))
print(tc.to_all_thrusters(np.array([0, 0,0,1,0,0], dtype=np.float32)))
print(tc.to_all_thrusters(np.array([0, 0,0,0,1,0], dtype=np.float32)))
print(tc.to_all_thrusters(np.array([0, 0,0,0,0,1], dtype=np.float32)))
