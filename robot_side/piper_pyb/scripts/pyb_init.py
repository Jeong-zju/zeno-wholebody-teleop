import pybullet_planning as pp
import time
from pipper import Pipper

robot_model_path = "/home/jeong/zeno/wholebody-teleop/piper_ros/src/piper_description/urdf/piper_description.urdf"
sever_id = pp.connect(use_gui=True)

pipper = Pipper(robot_model_path)
pipper.set_joint_positions([0, 0, 0, 0, 0, 0])
print(pipper.get_joint_positions())
pipper.set_joint_positions([1, 1, 1, 1, 1, 1])
print(pipper.get_joint_positions())
pipper.set_joint_positions([-1, -1, -1, -1, -1, -1])
print(pipper.get_joint_positions())

while True:
    # pp.stepSimulation(sever_id)
    time.sleep(0.01)
