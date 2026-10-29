from piper_description.display import display


def generate_launch_description():
    return display('piper_description_with_teach.urdf', 'piper_ctrl.rviz')
