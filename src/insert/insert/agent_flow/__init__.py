"""Multi-camera ChArUco calibration, GPT-6.1-sol dual-view perception, and full-pipeline web server."""

from .geometry import (
    BoardSpec,
    calibrate_external_camera,
    compute_alignment_plan,
    detect_charuco_pose,
    infer_gripper_tip_offset,
    invert_se3,
    load_extrinsics_file,
    pixel_to_base_ray,
    project_base_points,
    render_gpt_crop_overlay,
    render_gpt_output_overlay,
    render_perception_overlay,
    save_extrinsics_file,
    solve_axis_from_two_views,
    triangulate_two_rays,
    wrist_camera_pose_in_base,
)
from .vision_agent import (
    DEFAULT_OBJECT_PROMPT,
    DEFAULT_OUTPUT_PROMPT,
    perceive_dual_views,
)

__all__ = [
    'BoardSpec',
    'DEFAULT_OBJECT_PROMPT',
    'DEFAULT_OUTPUT_PROMPT',
    'calibrate_external_camera',
    'compute_alignment_plan',
    'detect_charuco_pose',
    'infer_gripper_tip_offset',
    'invert_se3',
    'load_extrinsics_file',
    'perceive_dual_views',
    'pixel_to_base_ray',
    'project_base_points',
    'render_gpt_crop_overlay',
    'render_gpt_output_overlay',
    'render_perception_overlay',
    'save_extrinsics_file',
    'solve_axis_from_two_views',
    'triangulate_two_rays',
    'wrist_camera_pose_in_base',
]
