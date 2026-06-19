from soarm_sdk import SOARM, ensure_user_config


config_path = ensure_user_config()

with SOARM.from_config(config_path) as arm:
    arm.calibrate(output_path=config_path, announce=print)
    print("Saved zero ticks, directions, and soft limits.")
