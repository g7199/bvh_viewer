from pyglm import glm
from Transforms import get_pelvis_virtual_safe

class Joint:
    """
    관절을 정의하는 Joint Class
    """
    def __init__(self, name: str, offset: tuple, channels: list):
        self.name = name
        self.channels = channels
        self.children = []
        self.offset = offset
        self.parent = None
        self.transform = glm.mat4(1.0)
    def addChild(self, child):
        child.parent = self
        self.children.append(child)


class FrameData:
    def __init__(self):
        self.rotation = {}
        self.position = {}

class Motion:
    def __init__(self, frame_Count: int, frame_time: float):
        self.frameData = []
        self.frameCount = frame_Count
        self.frameTime = frame_time

    def getFrame(self, index: int):
        """
        입력한 index의 motion data를 반환한다.
        :param index: frame index
        :return: 모션 데이터
        """
        if index < 0 or index >= self.frameCount:
            raise IndexError("Frame index out of range.")
        return self.frameData[index]
    def applyVirtualRoot(self):
        for frame in self.frameData:
            ap = frame.position['hip']
            ar = frame.rotation['hip']

            ap_local, ar_local = get_pelvis_virtual_safe(ap, ar)

            ap_global = ap - ap_local
            ar_global = ar * glm.conjugate(ar_local)


            frame.position['hip'] = [ap_local.x, ap_local.y, ap_local.z]
            frame.rotation['hip'] = ar_local

            frame.position["VirtualRoot"] = ap_global
            frame.rotation["VirtualRoot"] = ar_global


def parseJoint(tokens, index):
    """
    Recursive function to parse a joint definition from the tokenized BVH data.
    **Fixed:** Added recursive descent parsing for both 'ROOT', 'JOINT' and 'End Site'.
    """
    token = tokens[index]
    # Determine the joint type and name
    if token in ["ROOT", "JOINT"]:
        index += 1
        name = tokens[index]
        index += 1
    elif token == "End":
        index += 2  # Skip "End" and "Site"
        name = "End Site"
    # Skip the opening brace '{'
    index += 1

    offset = None
    channels = []
    children = []

    # Process the block until we hit the closing brace '}'
    while tokens[index] != "}":
        token = tokens[index]
        if token == "OFFSET":
            index += 1
            x = float(tokens[index]); index += 1
            y = float(tokens[index]); index += 1
            z = float(tokens[index]); index += 1
            offset = (x, y, z)
        elif token == "CHANNELS":
            index += 1
            num_channels = int(tokens[index])
            index += 1
            channels = tokens[index:index+num_channels]
            index += num_channels
        elif token in ["JOINT", "End"]:
            child, index = parseJoint(tokens, index)
            children.append(child)
        else:
            # Skip any tokens that do not match known keywords
            index += 1
    index += 1  # Skip the closing brace '}'

    if offset is None:
        offset = (0.0, 0.0, 0.0)
    joint = Joint(name, offset, channels)
    for child in children:
        joint.addChild(child)
    return joint, index


def BVHParser(file_path: str):
    """
    Parses the given BVH file and returns the root joint and motion data.
    """
    with open(file_path, 'r') as f:
        content = f.read()
    # Tokenize the file content by splitting on whitespace
    tokens = content.replace('\n', ' ').split()
    index = 0

    # Skip the "HIERARCHY" token if present
    if tokens[index] == "HIERARCHY":
        index += 1

    # Parse the root joint recursively
    root_joint, index = parseJoint(tokens, index)

    # Advance to the "MOTION" section
    while tokens[index] != "MOTION":
        index += 1
    index += 1  # Skip the "MOTION" token

    # Parse frame count
    if tokens[index] == "Frames:":
        index += 1
        frame_count = int(tokens[index])
        index += 1
    else:
        frame_count = 0

    # Parse frame time
    if tokens[index] == "Frame" and tokens[index+1] == "Time:":
        index += 2
        frame_time = float(tokens[index])
        index += 1
    else:
        frame_time = 0.0

    motion = Motion(frame_count, frame_time)

    # Collect joints with channels in depth-first order
    channel_order = []
    def collectChannels(joint):
        if joint.channels:
            channel_order.append(joint)
        for child in joint.children:
            collectChannels(child)
    collectChannels(root_joint)

    last_quat = {}

    for _ in range(frame_count):
        frame_data = FrameData()
        for joint in channel_order:
            values = []
            for _ in joint.channels:
                values.append(float(tokens[index]))
                index += 1

            if any("position" in ch.lower() for ch in joint.channels):
                pos = [values[i] for i, ch in enumerate(joint.channels) if "position" in ch.lower()]
                if joint.name.lower() in ["hip"]:
                    frame_data.position["hip"] = pos


            rotation = glm.vec3(0.0)
            for i, ch in enumerate(joint.channels):
                if "rotation" in ch.lower():
                    if ch.lower().startswith("x"):
                        rotation.x = glm.radians(values[i])
                    elif ch.lower().startswith("y"):
                        rotation.y = glm.radians(values[i])
                    elif ch.lower().startswith("z"):
                        rotation.z = glm.radians(values[i])

            qx = glm.angleAxis(rotation.x, glm.vec3(1, 0, 0))
            qy = glm.angleAxis(rotation.y, glm.vec3(0, 1, 0))
            qz = glm.angleAxis(rotation.z, glm.vec3(0, 0, 1))

            if joint.name.lower() in ["hip"]:
                rot_quat = qz * qy * qx
            else:
                rot_quat = qz * qx * qy

            frame_data.rotation[joint.name] = rot_quat


        motion.frameData.append(frame_data)

    virtualRoot = Joint("VirtualRoot", [0, 0, 0], ['Xposition', 'Yposition', 'Zposition', 'Zrotation', 'Yrotation', 'Xrotation'])
    motion.applyVirtualRoot()
    virtualRoot.addChild(root_joint)
    root_joint.parent = virtualRoot

    return virtualRoot, motion


# Example usage:
if __name__ == "__main__":
    # Provide the correct path to your BVH file
    bvh_file_path = r'C:\Users\admin\Downloads\cmuconvert-daz-113-128\114\114_05.bvh'
    root, motion = BVHParser(bvh_file_path)
    print("Parsed BVH file successfully!")
    print("Root Joint:", root.name)
    print(motion.frameData[0].position)
    print("Number of frames:", motion.frameCount)

