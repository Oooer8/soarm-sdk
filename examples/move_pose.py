from soarm_sdk import SOARM


with SOARM.from_config("configs/soarm-sdk.yaml") as arm:
    arm.enable()
    arm.move_pose("ready", duration=2.0)
