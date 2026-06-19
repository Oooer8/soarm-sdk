from soarm_sdk import SOARM, ensure_user_config


with SOARM.from_config(ensure_user_config()) as arm:
    arm.enable()
    arm.move_pose("ready", duration=2.0)
