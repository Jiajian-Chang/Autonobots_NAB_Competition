import csv
import math

def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    return angle_degrees

def get_reference_line(coordinates):
    x1_ref = coordinates['A1'][0]
    y1_ref = coordinates['A1'][1]
    x2_ref = coordinates['A2'][0]
    y2_ref = coordinates['A2'][1]

def get_checkpoints_angles(coordinates, reference_angle):
    angles = {}
    for checkpoint, (x, y) in coordinates.items():
        if checkpoint.startswith('J'):
            x_ref = (coordinates['A1'][0] + coordinates['A2'][0]) / 2
            y_ref = (coordinates['A1'][1] + coordinates['A2'][1]) / 2
            angle = calculate_angle(x_ref, y_ref, x, y) - reference_angle
            angles[checkpoint] = angle
    return angles

# Read checkpoint data from CSV file
def read_checkpoint_csv(file_path):
    coordinates = {}
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            name = row['NAME']
            x = float(row['EASTING'])
            y = float(row['NORTHING'])
            coordinates[name] = (x, y)
    return coordinates

# Example usage:
coordinates = read_checkpoint_csv('checkpoints_m.csv')
reference_angle = get_reference_angle(coordinates)
angles = get_checkpoints_angles(coordinates, reference_angle)
for checkpoint, angle in angles.items():
    print(f"Angle for moving to {checkpoint}: {angle} degrees")
