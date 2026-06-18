from soarm import SOARM


with SOARM.from_config("configs/soarm.yaml") as arm:
    arm.calibrate(output_path="configs/soarm.yaml", announce=print)
    print("Saved zero ticks, directions, and soft limits.")
