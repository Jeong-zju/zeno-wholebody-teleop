import pybullet_planning as pp

robot_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

class Pipper:
    def __init__(self, robot_model_path):
        self.robot = pp.load_model(robot_model_path)
        self.robot_joint_ids = [pp.joint_from_name(self.robot, name) for name in robot_joint_names]

    def set_joint_positions(self, joint_positions: list[float]) -> None:
        pp.set_joint_positions(self.robot, self.robot_joint_ids, joint_positions)

    def get_joint_positions(self) -> list[float]:
        return list(pp.get_joint_positions(self.robot, self.robot_joint_ids))