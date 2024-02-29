from PIL import Image, ImageDraw

# Create an empty image
width, height = 50, 50  # adjust based on your map size and resolution
img = Image.new('L', (width, height), color=255)  # 'L' for grayscale mode

draw = ImageDraw.Draw(img)

# Draw the boundaries
draw.polygon([(-15, -1), (15, -1), (15, 35), (-15, 35)], outline=0)  # adjust based on your map resolution

# Mark the starting and finishing points
draw.point((0, 0), fill=128)  # origin
draw.point((13.826, 28.960), fill=128)  # finish

# Mark the reference point
draw.point((-2.1, 0), fill=128)  # ref01

# Draw the gates
draw.line((4, -0.5, 4, 1.5), fill=0)  # gate01
draw.line((4.8, 3.8, 5.6, 3.8), fill=0)  # gate02

# Draw the finishing line
draw.line((10.851, 24.96, 13.826, 28.96), fill=0)  # finish line
draw.point((10.851, 24.96), fill=0) 
draw.point((13.826, 28.96), fill=0) 

# Draw the obstacles
obstacles = [
    ('garyTheSnail01', 5.104, 5.34, 0.015, 0.8),
    ('theStrip01', 5.8, 15.0, 0.261, 1),
    ('theStrip02', 5.8, 15.2, 0.263, 1),
    ('theStrip03', 5.8, 15.4, 0.261, 1),
    ('theStrip04', 8.8, 15.6, 0.263, 1),
    ('theStrip05', 8.8, 15.8, 0.265, 1),
    ('theStrip06', 8.8, 16, 0.264, 1),
    ('tunnel01', -8.654, 20.06, 1.9, 0.5),
    ('tunnel02', -7.089, 19.8, 1.9, 0.5),
    ('tunnel03', -8.675, 19.26, 1.9, 0.5)
]
for name, easting, northing, elevation, boundingRadius in obstacles:
    upper_left = (easting - boundingRadius, northing - boundingRadius)
    lower_right = (easting + boundingRadius, northing + boundingRadius)
    draw.ellipse([upper_left, lower_right], outline=0)


# Save the image in PGM format
img.save('map.pgm')
