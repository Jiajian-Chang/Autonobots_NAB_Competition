from collections import defaultdict
from PIL import Image, ImageDraw
import csv

def read_csv(file_path):
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        data = [row for row in csv_reader]
    return data

def calculate_image_size(coordinates, margin):
    # Find the min and max coordinates
    x_values = [float(coord['EASTING']) for coord in coordinates]
    y_values = [float(coord['NORTHING']) for coord in coordinates]
    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)
    # Calculate the required image size
    width = max_x - min_x + 100 * margin
    height = max_y - min_y + 100 * margin
    return int(width), int(height)

def draw_environment_dets(coordinates, additional_points, margin):
    # Calculate the dynamic image size
    image_size = calculate_image_size(coordinates, margin)
    # Calculate the scaling factor to fit the rectangle within the image with margin
    corner_coordinates = [coord for coord in coordinates if coord['NAME'].startswith('EDGE')]
    x_values = [float(coord['EASTING']) for coord in corner_coordinates]
    y_values = [float(coord['NORTHING']) for coord in corner_coordinates]
    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)
    width = max_x - min_x
    height = max_y - min_y
    scaling_factor = min((image_size[0] - 2 * margin) / width, (image_size[1] - 2 * margin) / height)
    # Create a new image
    img = Image.new('RGBA', image_size, color=(255, 255, 255, 255))
    draw = ImageDraw.Draw(img)
    # Draw the scaled rectangle
    #scaled_coordinates = [((float(coord['EASTING']) - min_x) * scaling_factor + margin, (float(coord['NORTHING']) - min_y) * scaling_factor + margin) for coord in corner_coordinates]
    # Draw the scaled rectangle with a thicker boundary
    #boundary_thickness = 8  # Adjust thickness as needed
    #for i in range(boundary_thickness):
    #    draw.polygon([
    #        (coord[0] - i, coord[1] - i) for coord in scaled_coordinates
    #    ], outline=(0, 0, 0, 255))
    return scaling_factor, margin, min_x, min_y, draw, img

def extract_characters(coordinates):
    for coords in coordinates:
        name_parts = coords['NAME'].split('_')
        if len(name_parts) > 1:  # If underscore exists
            characters = name_parts[0]
        else:
            characters = coords['NAME']
        coords["NAME_char"] = characters
    return coordinates

# Function to extract alphabets from a string
def extract_alphabets(coordinates):
    for coords in coordinates:
        name_char = ''.join(filter(str.isalpha, coords['NAME']))
        coords["name_char"] = name_char
    return coordinates

def midpoint_euclidean(x1,y1,x2,y2):
    dist_x = abs(x1-x2) / 2.
    dist_y = abs(y1-y2) / 2.
    res_x = x1 - dist_x if x1 > x2 else x2 - dist_x
    res_y = y1 - dist_y if y1 > y2 else y2 - dist_y
    return res_x, res_y

def get_checkpoints_midpoint(coordinates):
    coordinates = extract_alphabets(coordinates)
    # Group coordinates by name
    grouped_coords = defaultdict(list)
    for coord in coordinates:
        grouped_coords[coord['name_char']].append(coord)
    
    midpoints = []
    checkpoints_name = []
    for name_char, coords_list in grouped_coords.items():
        if len(coords_list) < 2:
            print(f"Not enough coordinates for checkpoint '{name_char}'")
            continue
        
        x1 = float(coords_list[0]['EASTING'])
        x2 = float(coords_list[1]['EASTING'])
        y1 = float(coords_list[0]['NORTHING'])
        y2 = float(coords_list[1]['NORTHING'])
        
        mid_x, mid_y = midpoint_euclidean(x1, y1, x2, y2)
        midpoints.append([mid_x, mid_y])
        checkpoints_name.append(name_char)
    
    checkpoints_final = list(zip(checkpoints_name, midpoints))
    print(checkpoints_final)
    return checkpoints_final
    
def circle_to_rectangle(x, y, radius):
    min_x = x - radius
    max_x = x + radius
    min_y = y - radius
    max_y = y + radius
    return [min_x, max_x, min_y, max_y]

def intersects(box1, box2):
    return not (box1[2] < box2[0] or box1[0] > box2[2] or box1[1] > box2[3] or box1[3] < box2[1])

def merge_rectangles(rectangles):
    if not rectangles:
        return None
    # Extract all unique x and y coordinates
    x_coords = sorted(list(set([x1 for x1, y1, x2, y2 in rectangles] + [x2 for x1, y1, x2, y2 in rectangles])))
    y_coords = sorted(list(set([y1 for x1, y1, x2, y2 in rectangles] + [y2 for x1, y1, x2, y2 in rectangles])))   
    # Find the min and max coordinates
    x_min = x_coords[0]
    x_max = x_coords[-1]
    y_min = y_coords[0]
    y_max = y_coords[-1]  
    # Construct the merged rectangle
    merged_rectangle = (x_min, y_min, x_max, y_max)
    return merged_rectangle


def draw_obstacles_dets(coordinates, scaling_factor, margin, min_x, min_y, draw, img):
    coordinates = extract_characters(coordinates)
    # Group coordinates by NAME
    grouped_coords = defaultdict(list)
    for coord in coordinates:
        grouped_coords[coord['NAME_char']].append(coord)
    # Draw obstacles as red outline rectangles
    overlapping_coords = []
    for coords in grouped_coords.values():
        if len(coords) == 1:
            x = float(coords[0]['EASTING'])
            y = float(coords[0]['NORTHING'])
            radius = float(coords[0]['RADIUS'])  
            rect_coords = circle_to_rectangle(x, y, radius)  
            # Scale rectangle coordinates
            scaled_min_x = (rect_coords[0] - min_x) * scaling_factor + margin
            scaled_max_x = (rect_coords[1] - min_x) * scaling_factor + margin
            scaled_min_y = (rect_coords[2] - min_y) * scaling_factor + margin
            scaled_max_y = (rect_coords[3] - min_y) * scaling_factor + margin  
            # Draw the rectangle
            draw.rectangle([scaled_min_x, scaled_min_y, scaled_max_x, scaled_max_y], outline=(255, 0, 0, 255), fill=(150, 150, 160, 255))   
            #overlapping_coords.append([[scaled_min_x, scaled_min_y, scaled_max_x, scaled_max_y]]) 
        else:
            scaled_coords_list = []
            for coord in coords:
                x = float(coord['EASTING'])
                y = float(coord['NORTHING'])
                radius = float(coord['RADIUS'])           
                # Convert circle coordinates to rectangle coordinates
                rect_coords = circle_to_rectangle(x, y, radius)   
                # Scale rectangle coordinates
                scaled_min_x = (rect_coords[0] - min_x) * scaling_factor + margin
                scaled_max_x = (rect_coords[1] - min_x) * scaling_factor + margin
                scaled_min_y = (rect_coords[2] - min_y) * scaling_factor + margin
                scaled_max_y = (rect_coords[3] - min_y) * scaling_factor + margin 
                scaled_coords_list.append([scaled_min_x, scaled_min_y, scaled_max_x, scaled_max_y]) 
            overlap_list1, overlap_list2 = [], []
            overlap_list1.append(scaled_coords_list[0])
            for coords in scaled_coords_list[1:]:
                # List to store overlapping rectangles
                if intersects(scaled_coords_list[0], coords):
                    overlap_list1.append(coords)
                else:
                    overlap_list2.append(coords)
            overlapping_coords.append(overlap_list1)
            overlapping_coords.append(overlap_list2)
            
    for coords_list in overlapping_coords:
        rectangle = merge_rectangles(coords_list)  
        draw.rectangle(rectangle, outline=(255, 0, 0), fill=(150, 150, 160, 255))   
    return img

def save_image(img):
    # Save the image
    img.save('color_map_v3.png')
    img = img.convert('L')  # Convert to black and white
    img.save('gray_map_v3.pgm')

def create_yaml(): #hard coded for now
    yaml = open("map.yaml", "w")
    yaml.write("image: gray_map.pgm\n")
    yaml.write("resolution: 0.050000\n")
    yaml.write("origin: [0.0, 0.0, 0.0]\n")
    yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196")
    yaml.close()

if __name__ == "__main__":
    environment_coordinates = read_csv('environment_m.csv')
    additional_points = [point for point in environment_coordinates if point['NAME'] in ('ORIGIN', 'REF01', 'FINISH')]
    margin = 10
    scaling_factor, margin, min_x, min_y, draw, img = draw_environment_dets(environment_coordinates, additional_points, margin)
    obstacle_coordinates = read_csv('obstacles_m.csv')
    img = draw_obstacles_dets(obstacle_coordinates, scaling_factor, margin, min_x, min_y, draw, img)
    save_image(img)
    create_yaml() #hard coded for now
    checkpoints_coordinates = read_csv('checkpoints_m.csv')
    get_checkpoints_midpoint(checkpoints_coordinates)

    

