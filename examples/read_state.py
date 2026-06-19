from soarm_sdk import SOARM, ensure_user_config


with SOARM.from_config(ensure_user_config()) as arm:
    for name, state in arm.get_joint_states().items():
        print(name, state)
