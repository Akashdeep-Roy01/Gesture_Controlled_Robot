from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("lbr_iisy3_r760", package_name="iisy_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
