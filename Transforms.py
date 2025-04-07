from pyglm import glm

def updateJointTransform(joint, frame_data, parent_transform=None):
    """
    Recursively update each joint's transform attribute using the motion frame data.

    For the root joint (when parent_transform is None), the hip position is combined with the joint's offset.
    For child joints, only the joint's static offset is used.

    The local transformation is computed as:
       local_transform = T(offset) * R(rotation)

    The global transformation is:
       joint.transform = parent_transform * local_transform   (or just local_transform for root)
    """
    # For the root joint, combine the hip position and the joint's offset.
    if parent_transform is None:
        root_translation = glm.vec3(*frame_data.position[joint.name]) + glm.vec3(*joint.offset)
        translation = glm.translate(glm.mat4(1.0), root_translation)
    else:
        translation = glm.translate(glm.mat4(1.0), glm.vec3(*joint.offset))

    # Get the per-frame rotation if available; otherwise, use identity.
    if joint.name in frame_data.rotation:
        rotation = glm.mat4_cast(frame_data.rotation[joint.name])
    else:
        rotation = glm.mat4(1.0)

    joint.transform = translation * rotation

    # Recursively update all child joints.
    for child in joint.children:
        updateJointTransform(child, frame_data, joint.transform)


def get_pelvis_virtual_safe(ap: glm.vec3, ar: glm.quat,
                            fallback_forward=glm.vec3(0, 0, 1),
                            smooth_ratio=0.2,
                            prev_r_inv_ref=None):
    """
    안정적으로 virtual root를 추출하는 개선된 함수입니다.
    :param ap: pelvis 위치 (world 기준)
    :param ar: pelvis 회전 (world 기준)
    :param fallback_forward: 비정상 시 사용하는 전방 벡터
    :param smooth_ratio: slerp 보간 비율 (0~1, 높을수록 빠르게 변화)
    :param prev_r_inv_ref: 이전 프레임의 r_inv 값을 참조하고 갱신할 리스트 [quat]
    :return: (new_ap, new_ar)
    """
    up = glm.vec3(0, 1, 0)

    # 수직 성분 제거 (위치 기준 평면화)
    p = ap - get_projection(ap, up)

    # 현재 바라보는 방향 벡터 추출 후 수평화
    f = ar * glm.vec3(0, 0, 1)
    f_mod = f - get_projection(f, up)

    # 벡터 길이 체크 후 보정
    if glm.length(f_mod) < 1e-4:
        f_mod = fallback_forward

    # 회전 정렬 및 역변환
    r = lookrotation(f_mod, up)
    r_inv = glm.inverse(r)

    # low-pass filtering (이전 값과 slerp)
    if prev_r_inv_ref is not None:
        r_inv = glm.slerp(prev_r_inv_ref[0], r_inv, smooth_ratio)
        prev_r_inv_ref[0] = r_inv  # 업데이트

    # 회전 기준 정렬
    new_ap = r_inv * (ap - p)
    new_ar = r_inv * ar
    return new_ap, new_ar

def get_projection(v: glm.vec3, onto: glm.vec3):
    onto_norm = glm.normalize(onto)
    return glm.dot(v, onto_norm) * onto_norm


def lookrotation(v: glm.vec3, u: glm.vec3) -> glm.quat:
    v_hat = glm.normalize(v)
    u_hat = glm.normalize(u)
    t_hat = glm.normalize(glm.cross(u_hat, v_hat))
    up_corrected = glm.cross(v_hat, t_hat)
    rot_mat = glm.mat3(t_hat, up_corrected, v_hat)
    rot_q = glm.quat_cast(rot_mat)
    if rot_q.w < 0:
        rot_q = -rot_q
    return rot_q


def apply_virtual_root_offset(current_motion, next_motion):
    position_offset, rotation_offset = compute_offset(current_motion, next_motion)


    # Apply the offset to every frame of the next animation
    for frame in next_motion.frameData:
        # Update hip position by applying the position offset

        # Update virtual root position similarly
        vr_vec = glm.vec3(*frame.position["VirtualRoot"])
        vr_vec += position_offset
        frame.position["VirtualRoot"] = [vr_vec.x, vr_vec.y, vr_vec.z]

        # Update rotations by applying the rotation offset
        frame.rotation["hip"] = rotation_offset * frame.rotation["hip"]
        frame.rotation["VirtualRoot"] = rotation_offset * frame.rotation["VirtualRoot"]


def blend_new_animation_offset(next_motion, position_offset, rotation_offset, frame_index):
    """
    For the first few frames of the new animation, blend in the offset.
    Here, blend_duration is set to 10% of the total frame count.
    """
    total_frames = next_motion.frameCount
    blend_duration = int(0.1 * total_frames)  # Blend over first 10% of frames
    if frame_index < blend_duration:
        # Compute blend factor (0 at frame 0, 1 at frame blend_duration)
        t = frame_index / blend_duration
        # For rotation: SLERP from identity (no offset) to full rotation_offset
        blended_rot_offset = glm.slerp(glm.quat(1, 0, 0, 0), rotation_offset, t)
        # For position: linear interpolation from zero offset to full offset
        blended_pos_offset = position_offset * t
    else:
        # After the blend period, use the full offset.
        blended_rot_offset = rotation_offset
        blended_pos_offset = position_offset

    # Get the current frame of the new animation.
    frame = next_motion.frameData[frame_index]
    # Apply the blended position offset to the hip position.

    # Apply the blended rotation offset to the hip rotation.
    frame.rotation["hip"] = blended_rot_offset * frame.rotation["hip"]

def compute_offset(current_motion, next_motion):
    # Get the final frame of the current animation
    last_frame = current_motion.frameData[-1]
    prev_vr_pos = glm.vec3(*last_frame.position["VirtualRoot"])
    prev_vr_rot = last_frame.rotation["VirtualRoot"]

    # Get the first frame of the next animation
    first_next = next_motion.frameData[0]
    next_vr_pos = glm.vec3(*first_next.position["VirtualRoot"])
    next_vr_rot = first_next.rotation["VirtualRoot"]


    # Compute full offset transformation
    position_offset = prev_vr_pos - next_vr_pos
    rotation_offset = prev_vr_rot * glm.inverse(next_vr_rot)

    return position_offset, rotation_offset
