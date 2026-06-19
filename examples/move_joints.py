from soarm_sdk import SOARM, ensure_user_config


with SOARM.from_config(ensure_user_config()) as arm:
    arm.enable()
    arm.move_joints(
        {
            "shoulder_pan": 0.0,
            "shoulder_lift": -0.35,
            "elbow_flex": 0.7,
            "wrist_flex": 0.1,
        },
        duration=2.0,
    )
