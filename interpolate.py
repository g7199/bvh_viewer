import numpy as np
from scipy.spatial.transform import Rotation as R
from Transforms import compute_forward_kinetics, translation_matrix, get_pelvis_virtual, inverse_matrix

def extract_pitch_roll_y(T):
        r = R.from_matrix(T[:3, :3])
        pitch, yaw, roll = r.as_euler('xyz', degrees=True)
        y = T[1, 3]
        return pitch, yaw, roll, y

def lerp(t, motion1, motion2):
    interp = (1 - t) * np.array(motion1) + t * np.array(motion2)
    return interp

def get_localized_motions(motion, root):
    motion = list(map(float, motion))
    root_motion = motion[:6]
    motion_kinetics = compute_forward_kinetics(root, root_motion[3:6])

    T_global_pelvis = translation_matrix(root_motion[0:3]) @ motion_kinetics

    #local에서 y좌표를 못잡는 것 같음 (0으로 죽여버림)
    T_local_pelvis  = get_pelvis_virtual(T_global_pelvis)

    pitch, yaw, roll, y = extract_pitch_roll_y(T_local_pelvis)

    print(pitch, yaw, roll)

    rotation_dict = {
        "Xrotation": pitch,
        "Yrotation": yaw,
        "Zrotation": roll,
    }
    position_dict = {
         "Xposition": 0,
         "Yposition": y,
         "Zposition": 0,
    }

    ordered_positions = [position_dict[channel] for channel in root.channels[:3]]
    ordered_angles = [rotation_dict[channel] for channel in root.channels[3:6]]

    k = ordered_positions + ordered_angles + motion[6:]

    motion_kinetics = compute_forward_kinetics(root, k[3:6])

    pitch, yaw, roll, y = extract_pitch_roll_y(motion_kinetics)

    return ordered_positions + ordered_angles + motion[6:]

def interpolation(root, prev_motions, next_motions, offset, blended_len = 3, mode='lerp'):
    prev_motions = prev_motions[-blended_len:]
    next_motions = next_motions[1:blended_len+1]

    for idx in range(len(prev_motions)):
        prev_motions[idx] = get_localized_motions(prev_motions[idx], root)

    for idx in range(len(next_motions)):
        next_motions[idx] = get_localized_motions(next_motions[idx], root)


    if mode == 'lerp':
        blended_motions = []
        for b in range(blended_len):
            blended_motions.append(lerp((b+1)/(blended_len+1), prev_motions[-1], next_motions[0]))

        return blended_motions
    else:
        print("error")
        exit(1)

    
