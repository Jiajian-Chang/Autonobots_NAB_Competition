from scipy.spatial.transform import Rotation

# Create a rotation object from Euler angles specifying axes of rotation
rot = Rotation.from_euler('xyz', [0, 0, 180], degrees=True)

# Convert to quaternions and print
rot_quat = rot.as_quat()
print(rot_quat)