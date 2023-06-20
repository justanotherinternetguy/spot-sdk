from PIL import Image

def upscale(filepath):
    original_image = Image.open(filepath)
    
    upscale_factor = 100  # Replace with the desired value
    
    new_width = original_image.width * upscale_factor
    new_height = original_image.height * upscale_factor
    
    upscaled_image = original_image.resize((new_width, new_height), resample=Image.NEAREST)
    
    upscaled_image.save('upscaled_image.png')


upscale("tag36_11_00000.png")
