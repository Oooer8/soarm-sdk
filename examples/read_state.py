from soarm_sdk import SOARM


with SOARM.from_config("configs/soarm-sdk.yaml") as arm:
    for name, state in arm.get_joint_states().items():
        print(name, state)
