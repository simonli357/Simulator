from PIL import Image

# Define the canvas size and sign size based on the original image analysis
canvas_size = (1152, 648)
sign_size = (229, 229)
sign_position = (0, 10)  # Approximate top-left corner of original sign

# Load the blue sign image and resize it to match the original sign size
blue_sign = Image.open("a.png").resize(sign_size, Image.Resampling.LANCZOS)

# Create a blank canvas with the same size as the original image (gray background)
canvas = Image.new("RGBA", canvas_size, (128, 128, 128, 255))  # Gray background

# Paste the resized blue sign onto the canvas at the same position as the original
canvas.paste(blue_sign, sign_position, blue_sign)

# Save the final image
final_image_path = "roundabout_sign.png"
canvas.save("/home/slsecret/Simulator/src/models_pkg/roundabout_sign/materials/textures" + final_image_path)

print("saved: ", final_image_path)
