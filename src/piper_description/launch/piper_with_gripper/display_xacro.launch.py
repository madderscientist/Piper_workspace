from piper_description.display import display


def generate_launch_description():
    return display('piper_description.xacro', 'piper_ctrl.rviz', gui_rate=200.0)
