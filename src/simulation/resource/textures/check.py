from PIL import Image
import numpy as np
import os

def calculate_apriltag_black_size(image_path, physical_total_width_meters):
    # 1. Load the image
    img = Image.open(image_path).convert('L') # Convert to grayscale
    img_data = np.array(img)
    
    # 2. Threshold to binary (0 for black, 255 for white) to handle any anti-aliasing
    # Using 128 as a safe middle ground
    binary_img = (img_data > 128).astype(int)
    
    # 3. Find the bounds of the black region
    # We look for rows and cols that have at least one black pixel (0)
    rows_with_black = np.where(np.any(binary_img == 0, axis=1))[0]
    cols_with_black = np.where(np.any(binary_img == 0, axis=0))[0]
    
    if len(rows_with_black) == 0:
        return "Error: No black pixels found in image."

    # 4. Calculate dimensions in pixels
    # We add 1 because if pixels 10 to 20 are black, the width is (20-10) + 1 = 11 pixels
    black_pixel_width = (cols_with_black[-1] - cols_with_black[0]) + 1
    total_pixel_width = img_data.shape[1]
    
    # 5. Calculate the ratio
    ratio = black_pixel_width / total_pixel_width
    
    # 6. Calculate physical size
    physical_black_size = ratio * physical_total_width_meters
    
    return {
        "total_pixels": total_pixel_width,
        "black_pixels": black_pixel_width,
        "ratio": ratio,
        "calculated_black_size_meters": physical_black_size
    }

# --- USER INPUT ---
filename = os.path.join(os.path.dirname(__file__), "0.png")
board_size = 0.325437 

# Run calculation
result = calculate_apriltag_black_size(filename, board_size)

print(f"--- Results for {filename} ---")
print(f"Total Image Width (px): {result['total_pixels']}")
print(f"Black Region Width (px): {result['black_pixels']}")
print(f"Calculated Ratio: {result['ratio']:.4f}")
print(f"---------------------------")
print(f"REAL SIZE FOR CONFIG: {result['calculated_black_size_meters']:.6f} meters")