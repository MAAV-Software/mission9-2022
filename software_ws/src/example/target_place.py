import random
import math
import numpy as np
# import pyproj
import os
import shutil
import chevron
# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
# generate the random locations of the 5 images

# make sure a drop point is at least 40 ft away from every other drop point
def validDist(new_point, points, r_threshold):
    for point in points:
        dist = math.sqrt((math.pow(new_point[0]-point[0], 2) + math.pow(new_point[1]-point[1], 2)))
        if dist < r_threshold:
            return False
    return True


dRadius = 6.096 # the radius of a drop point

# bonudary for drop zone, 70 by 360 ft, converted to meters
# Note: this should be shifted 10 meters away from (0, 0, 0)
drop_boundary = [(10, 0, 0), (31.366, 0, 0), (31.366, 109.728, 0), (10, 109.728, 0)]

drop_points = [(10, 10, 0)]
# drop_points.append((random.uniform(10 + dRadius, 31.366 - dRadius), random.uniform(dRadius, 109.728 - dRadius), 0))

# This while loop runs until 5 points are generated, which don't overlap
while len(drop_points) < 6: # generates 5 random points within boundary
    new_point = (random.uniform(10 + dRadius, 31.366 - dRadius), random.uniform(dRadius, 109.728 - dRadius), 0)
    # check the distance between each of the existing drop points
    # if it doesn't overlap with any, append it to drop_points
    if (validDist(new_point, drop_points, ((2 * dRadius) + 1))):
        drop_points.append(new_point)

# print(drop_points)


#generate the 5 materials
random.seed(42) 
for i in range(0, 5):
    target_number = str(i+1)
    directory_path = '/root/.gazebo/models/target_' + target_number + '/'
    os.makedirs(directory_path + '/materials/scripts', exist_ok=True)
    os.makedirs(directory_path + '/materials/textures', exist_ok=True)
    with open('templates/material_template.mustache', 'r') as f:
        rendered_content = chevron.render(f, {'target_number': target_number, 'image_name': "target_" + target_number })
        file_path = '/materials/scripts/target_' + target_number + '.material'
        with open(directory_path + file_path, 'w') as output_file:
            output_file.write(rendered_content)

    with open('templates/model_template.mustache', 'r') as f:
        rendered_content = chevron.render(f, {'x_pose': drop_points[i][0], 'y_pose': drop_points[i][1], 'target_number': target_number, 'name': "target_" + target_number })
        file_path = 'target_' + target_number + '.sdf'
        with open(directory_path + file_path, 'w') as output_file:
            output_file.write(rendered_content)

    with open('templates/target_model_conflict.mustache', 'r') as f:
            rendered_content = chevron.render(f, {'target_number': target_number, 'name': "target_" + target_number })
            file_path = 'model.config'
            with open(directory_path + file_path, 'w') as output_file:
                output_file.write(rendered_content)

    shape = random.choice(os.listdir("/mission9-2022/software_ws/src/example/.data/training_images"))
    source_path = os.path.join('/mission9-2022/software_ws/src/example/.data/training_images', shape)
    filepath = os.path.join(directory_path + '/materials/textures', 'target_image_' + str(i+1) + '.png')
    shutil.copy2(source_path, filepath)