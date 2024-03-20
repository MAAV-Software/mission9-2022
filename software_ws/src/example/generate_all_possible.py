import random
import os
import shutil
from PIL import Image, ImageDraw, ImageFont
from ruleset import shapes, color_options, symbols

# Random generator for Shape, Color and Symbol


def save_random_color_image(filename, img_h, img_w):
    selected_shape = random.choice(shapes)
    selected_color = random.choice(color_options)
    while True:
        color2 = random.choice(color_options)
        if color2 != selected_color:
            break

    selected_symbol = random.choice(symbols)
    # selected_shape = "Quarter_circle"

    print(selected_shape)
    print(selected_color)

    gray = (128, 128, 128)

    # ('RGB', (100, 100), selected_color)
    img = Image.new('RGB', (img_h, img_w), gray)

    draw = ImageDraw.Draw(img)
    center = (0.5*img_w, 0.5*img_h)
    radius = 0.4*img_w

    if selected_shape == "Circle":
        draw.ellipse([center[0]-radius, center[1]-radius,
                     center[0]+radius, center[1]+radius], fill=selected_color)
    elif selected_shape == "Semi-Circle":
        center = (0.5*img_w, 0.35*img_h)
        draw.pieslice([center[0]-radius, center[1]-radius, center[0] +
                      radius, center[1]+radius], 0, 180, fill=selected_color)
    elif selected_shape == "Quarter_circle":
        center = (0.325*img_w, 0.35*img_h)
        draw.pieslice([center[0]-radius, center[1]-radius, center[0] +
                      radius, center[1]+radius], 0, 90, fill=selected_color)
    elif selected_shape == "Star":
        # Define points for a star
        points = [
            (0.5*img_w, 0.1*img_h),
            (0.61*img_w, 0.5*img_h),
            (1.0*img_w, 0.5*img_h),
            (0.7*img_w, 0.7*img_h),
            (0.8*img_w, 1.0*img_h),
            (0.5*img_w, 0.8*img_h),
            (0.2*img_w, 1.0*img_h),
            (0.3*img_w, 0.7*img_h),
            (0, 0.5*img_h),
            (0.39*img_w, 0.5*img_h)
        ]
        draw.polygon(points, fill=selected_color)
    elif selected_shape == "Pentagon":
        # Define points for a regular pentagon
        points = [
            (.50 * img_w, img_h * .05),
            (img_w * 0.95, img_h * .45),
            (.80 * img_w, img_h * 0.95),
            (.20 * img_w, img_h * 0.95),
            (img_w * .05, img_h * .45),
        ]
        draw.polygon(points, fill=selected_color)
    elif selected_shape == "Triangle":
        # Define points for an equilateral triangle
        triangle = [
            (img_w * .50, img_h * .10),
            (img_w * .90, img_h * .90),
            (img_w * .10, img_h * .90),
        ]
        draw.polygon(triangle, fill=selected_color)
    elif selected_shape == "Rectangle":
        # Define points for a rectangle
        draw.rectangle([.2 * img_w, .2 * img_h, .80 *
                       img_w, .80 * img_h], fill=selected_color)
    elif selected_shape == "Cross":
        # Define points for a cross
        draw.rectangle([img_w*.20, img_h*.40, img_w*.80,
                       img_h*.60], fill=selected_color)
        draw.rectangle([img_w*.40, img_h*.20, img_w*.60,
                       img_h*.80], fill=selected_color)

    font = ImageFont.truetype("Roboto/Roboto-Black.ttf", size=50)
    draw.text((img_h * 0.465, img_w * 0.465),
              selected_symbol, color2, font=font)

    img.save(filename)

def generate_img(shape, image_color, symbol, symbol_color):
    img_h = 500
    img_w = 500
    selected_shape = shape
    selected_color = image_color
    color2 = symbol_color
    selected_symbol = symbol

    gray = (128, 128, 128)
    # ('RGB', (100, 100), selected_color)
    img = Image.new('RGB', (img_h, img_w), gray)

    draw = ImageDraw.Draw(img)
    center = (0.5*img_w, 0.5*img_h)
    radius = 0.4*img_w

    if selected_shape == "Circle":
        draw.ellipse([center[0]-radius, center[1]-radius,
                     center[0]+radius, center[1]+radius], fill=selected_color)
    elif selected_shape == "SemiCircle":
        center = (0.5*img_w, 0.35*img_h)
        draw.pieslice([center[0]-radius, center[1]-radius, center[0] +
                      radius, center[1]+radius], 0, 180, fill=selected_color)
    elif selected_shape == "QuarterCircle":
        center = (0.325*img_w, 0.35*img_h)
        draw.pieslice([center[0]-radius, center[1]-radius, center[0] +
                      radius, center[1]+radius], 0, 90, fill=selected_color)
    elif selected_shape == "Star":
        # Define points for a star
        points = [
            (0.5*img_w, 0.1*img_h),
            (0.61*img_w, 0.5*img_h),
            (1.0*img_w, 0.5*img_h),
            (0.7*img_w, 0.7*img_h),
            (0.8*img_w, 1.0*img_h),
            (0.5*img_w, 0.8*img_h),
            (0.2*img_w, 1.0*img_h),
            (0.3*img_w, 0.7*img_h),
            (0, 0.5*img_h),
            (0.39*img_w, 0.5*img_h)
        ]
        draw.polygon(points, fill=selected_color)
    elif selected_shape == "Pentagon":
        # Define points for a regular pentagon
        points = [
            (.50 * img_w, img_h * .05),
            (img_w * 0.95, img_h * .45),
            (.80 * img_w, img_h * 0.95),
            (.20 * img_w, img_h * 0.95),
            (img_w * .05, img_h * .45),
        ]
        draw.polygon(points, fill=selected_color)
    elif selected_shape == "Triangle":
        # Define points for an equilateral triangle
        triangle = [
            (img_w * .50, img_h * .10),
            (img_w * .90, img_h * .90),
            (img_w * .10, img_h * .90),
        ]
        draw.polygon(triangle, fill=selected_color)
    elif selected_shape == "Rectangle":
        # Define points for a rectangle
        draw.rectangle([.2 * img_w, .2 * img_h, .80 *
                       img_w, .80 * img_h], fill=selected_color)
    elif selected_shape == "Cross":
        # Define points for a cross
        draw.rectangle([img_w*.20, img_h*.40, img_w*.80,
                       img_h*.60], fill=selected_color)
        draw.rectangle([img_w*.40, img_h*.20, img_w*.60,
                       img_h*.80], fill=selected_color)

    font = ImageFont.truetype("Roboto-Black.ttf", size=50)
    draw.text((img_h * 0.465, img_w * 0.465),
              selected_symbol, color2, font=font)

    return img

# to get the names for lableing
color_dict = {
    (255, 255, 255): 'white',
    (0, 0, 0): 'black',
    (255, 0, 0): 'red',
    (0, 0, 255): 'blue',
    (0, 255, 0): 'green',
    (127, 0, 255): 'purple',
    (102, 51, 0): 'brown',
    (255, 128, 0): 'orange',
}

def get_color_label(rgb_tuple):
    return color_dict.get(rgb_tuple, 'unknown')

def save_all_images(folder_name, img_h, img_w, labels_file):
    image_number = 1
    for selected_shape in shapes:
        for selected_color in color_options:
            for color2 in color_options:
                if color2 == selected_color:
                    continue
                for selected_symbol in symbols:
                    # Format the labels for shape, color, symbol, and character color
                    shape_label = selected_shape.lower()
                    color_label = get_color_label(selected_color)
                    char_color_label = get_color_label(color2)
                    symbol_label = selected_symbol.upper() if selected_symbol.isalpha() else selected_symbol

                    char_label = ''.join(c for c in char_color_label if c.isalnum())
                    char_label = char_label if char_label else 'x'  # If empty, use 'x'

                    filename = os.path.join(
                        folder_name, f"test_{shape_label}_{color_label}_{symbol_label}_{char_label}.png")

                    label = f"{selected_shape}, {color_label}, {selected_symbol}, {char_color_label}\n"
                    labels_file.write(label)

                    img = generate_img(selected_shape, selected_color, selected_symbol, color2)
                    img.save(filename)
                    image_number += 1

if __name__ == "__main__":
    if os.path.exists(".data/"):
        shutil.rmtree(".data/")
    os.mkdir(".data/")
    folder_name = ".data/training_images"
    if os.path.exists(folder_name):
        shutil.rmtree(folder_name)
    os.mkdir(folder_name)

    labels_output_fname = "labels.txt"
    if os.path.exists(labels_output_fname):
        os.remove(labels_output_fname)
    labels = open(labels_output_fname, "w")
    save_all_images(folder_name, 500, 500, labels)
    labels.close()