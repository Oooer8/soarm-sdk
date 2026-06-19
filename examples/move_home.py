from soarm_sdk import SOARM, ensure_user_config


with SOARM.from_config(ensure_user_config()) as arm:
    arm.enable()
    arm.move_home(duration=1.5)
