import sys
import os
import glob
import json
import numpy as np

def read_pgm16(filename):
    """Read a 16-bit PGM file and return a numpy array."""
    with open(filename, 'rb') as f:
        # Read header
        assert f.readline() == b'P5\n'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width_height = line
        width, height = [int(i) for i in width_height.split()]
        maxval = int(f.readline())
        assert maxval == 65535
        # Read image data
        img = np.frombuffer(f.read(), dtype='>u2').reshape((height, width))
        return img

def find_percentiles(img, lower, upper):
    """Return the lower and upper percentiles of the image."""
    flat = img.flatten()
    return (np.percentile(flat, lower), np.percentile(flat, upper))

def main():
    if len(sys.argv) != 5:
        print("Usage: python update_config.py <config.json> <image_dir> <lower_percentile,upper_percentile> <output.json>")
        sys.exit(1)

    config_file = sys.argv[1]
    image_dir = sys.argv[2]
    percentiles = sys.argv[3].split(',')
    lower = float(percentiles[0])
    upper = float(percentiles[1])
    output_file = sys.argv[4]

    # Load config
    with open(config_file, 'r') as f:
        config = json.load(f)

    camera_ids = [cam['id'] for cam in config['cameras']]
    images = []
    percentiles_per_camera = []

    # Read images and compute percentiles
    for cam_id in camera_ids:
        pattern = os.path.join(image_dir, f"*_{cam_id}_1.pgm")
        files = glob.glob(pattern)
        if not files:
            print(f"Warning: No image found for camera id {cam_id} with pattern {pattern}")
            images.append(None)
            percentiles_per_camera.append((None, None))
            continue
        img = read_pgm16(files[0])
        images.append(img)
        p_lo, p_hi = find_percentiles(img, lower, upper)
        percentiles_per_camera.append((p_lo, p_hi))

    # Use the first camera as reference
    ref_lo, ref_hi = percentiles_per_camera[0]
    ref_range = ref_hi - ref_lo

    # Update config to scale and offset all cameras to match the first camera
    for i, cam in enumerate(config['cameras']):
        p_lo, p_hi = percentiles_per_camera[i]
        if p_lo is None or p_hi is None:
            continue
        cam_range = p_hi - p_lo
        cam['color']['gain'] = ref_range / cam_range if cam_range != 0 else 1.0
        cam['color']['offset'] = ref_hi - p_hi
        print(f"Camera {cam['id']}: gain={cam['color']['gain']}, offset={cam['color']['offset']}, hi={p_hi}, range={cam_range}")

    # Write updated config
    with open(output_file, 'w') as f:
        json.dump(config, f, indent=2)

if __name__ == "__main__":
    main()

