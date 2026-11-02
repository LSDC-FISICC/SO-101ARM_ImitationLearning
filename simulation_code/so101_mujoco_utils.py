import time
import math
import mujoco

def convert_to_dictionary(qpos):
    return {
        'shoulder_pan': math.degrees(qpos[0]),
        'shoulder_lift': math.degrees(qpos[1]),
        'elbow_flex': math.degrees(qpos[2]),
        'wrist_flex': math.degrees(qpos[3]),
        'wrist_roll': math.degrees(qpos[4]),
        'gripper': qpos[5] * 100 / math.pi,
    }

def convert_to_list(dictionary):
    return [
        math.radians(dictionary['shoulder_pan']),
        math.radians(dictionary['shoulder_lift']),
        math.radians(dictionary['elbow_flex']),
        math.radians(dictionary['wrist_flex']),
        math.radians(dictionary['wrist_roll']),
        dictionary['gripper'] * math.pi / 100.0,
    ]

def set_initial_pose(d, position_dict):
    d.qpos = convert_to_list(position_dict)

def send_position_command(d, position_dict):
    d.ctrl = convert_to_list(position_dict)

def move_to_pose(m, d, viewer, desired_position, duration):
    start_time = time.time()
    starting_pose = convert_to_dictionary(d.qpos.copy())

    while True:
        t = time.time() - start_time
        if t > duration:
            break
        alpha = min(t / duration, 1)

        position_dict = {}
        for joint in desired_position:
            position_dict[joint] = (1 - alpha) * starting_pose[joint] + alpha * desired_position[joint]

        send_position_command(d, position_dict)
        mujoco.mj_step(m, d)
        viewer.sync()

def hold_position(m, d, viewer, duration):
    current_pos_dict = convert_to_dictionary(d.qpos.copy())
    start_time = time.time()
    while True:
        if time.time() - start_time > duration:
            break
        send_position_command(d, current_pos_dict)
        mujoco.mj_step(m, d)
        viewer.sync()