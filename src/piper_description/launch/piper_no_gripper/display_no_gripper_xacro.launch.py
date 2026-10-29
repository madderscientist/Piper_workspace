from piper_description.display import display


def generate_launch_description():
    return display('piper_no_gripper_description.xacro', 'piper_no_gripper.rviz')
