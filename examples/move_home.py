from soarm import SOARM


with SOARM.from_config("configs/soarm.yaml") as arm:
    arm.enable()
    arm.move_home(duration=1.5)
