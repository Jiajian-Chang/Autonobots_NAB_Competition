from PIL import Image, ImageDraw
import csv

def read_csv(file_path):
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        data = [row for row in csv_reader]
    return data

def calculate_image_size(coordinates, margin):
    # Find the min and max coordinates
    x_values = [float(coord['easting']) for coord in coordinates]
    y_values = [float(coord['northing']) for coord in coordinates]
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
    corner_coordinates = [coord for coord in coordinates if coord['name'].startswith('corner')]
    x_values = [float(coord['easting']) for coord in corner_coordinates]
    y_values = [float(coord['northing']) for coord in corner_coordinates]
    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)
    width = max_x - min_x
    height = max_y - min_y
    scaling_factor = min((image_size[0] - 2 * margin) / width, (image_size[1] - 2 * margin) / height)
    # Create a new image
    img = Image.new('RGBA', image_size, color=(255, 255, 255, 255))
    draw = ImageDraw.Draw(img)
    # Draw the scaled rectangle
    scaled_coordinates = [((float(coord['easting']) - min_x) * scaling_factor + margin, (float(coord['northing']) - min_y) * scaling_factor + margin) for coord in corner_coordinates]
    draw.polygon([scaled_coordinates[0], scaled_coordinates[1], scaled_coordinates[3], scaled_coordinates[2]], outline=(255, 204, 0, 255))
    # Draw additional points with black color
    for point_coord in additional_points:
        x, y = (float(point_coord['easting']) - min_x) * scaling_factor + margin, (float(point_coord['northing']) - min_y) * scaling_factor + margin
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=(0, 0, 0, 255))  # Draw a big black dot
    return scaling_factor, margin, min_x, min_y, draw, img

def draw_checkpoints_dets(coordinates, scaling_factor, margin, min_x, min_y, draw, img):
    # Draw checkpoint triangles
    for coord in coordinates:
        x, y = (float(coord['easting']) - min_x) * scaling_factor + margin, (float(coord['northing']) - min_y) * scaling_factor + margin
        triangle_points = [
            (x - 5, y - 5),
            (x + 5, y),
            (x - 5, y + 5)
        ]
        draw.polygon(triangle_points, outline=(0, 100, 0), fill=(0, 100, 0, 153)) # 80% opacity 255*0.8 = 204 for alpha value
    return scaling_factor, margin, min_x, min_y, draw, img

def draw_obstacles_dets(coordinates, scaling_factor, margin, min_x, min_y, draw, img):
    # Draw obstacles as red outline circles
    for coord in coordinates:
        x, y, radius = (float(coord['easting']) - min_x) * scaling_factor + margin, (float(coord['northing']) - min_y) * scaling_factor + margin, float(coord['boundingRadius']) * scaling_factor
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline=(255, 0, 0), fill=((150, 150, 160, 255)))
    return img

def save_image(img):
    # Save the image
    img.save('color_map.png')
    img = img.convert('L')  # Convert to black and white
    img.save('gray_map.pgm')

def create_yaml(): # hard coded for now
    yaml = open("map.yaml", "w")
    yaml.write("image: gray_map.pgm\n")
    yaml.write("resolution: 0.050000\n")
    yaml.write("origin: [0.0, 0.0, 0.0]\n")
    yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196")
    yaml.close()

if __name__ == "__main__":
    environment_coordinates = read_csv('environment.csv')
    additional_points = [point for point in environment_coordinates if point['name'] in ('origin', 'ref01', 'finish')]
    margin = 10
    scaling_factor, margin, min_x, min_y, draw, img = draw_environment_dets(environment_coordinates, additional_points, margin)

    checkpoint_coordinates = read_csv('checkpoints.csv')
    scaling_factor, margin, min_x, min_y, draw, img = draw_checkpoints_dets(checkpoint_coordinates, scaling_factor, margin, min_x, min_y, draw, img)

    obstacle_coordinates = read_csv('obstacles.csv')
    img = draw_obstacles_dets(obstacle_coordinates, scaling_factor, margin, min_x, min_y, draw, img)

    save_image(img)

    create_yaml() # hard coded for now

    

