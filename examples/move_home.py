from soarm_sdk import SOARM


with SOARM.from_config("configs/soarm-sdk.yaml") as arm:
    arm.enable()
    arm.move_home(duration=1.5)
