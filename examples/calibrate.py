from soarm_sdk import SOARM


with SOARM.from_config("configs/soarm-sdk.yaml") as arm:
    arm.calibrate(output_path="configs/soarm-sdk.yaml", announce=print)
    print("Saved zero ticks, directions, and soft limits.")
