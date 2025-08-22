import os
import numpy as np
import imageio
import re

def read_pgm16(filename):
    """Read a 16-bit PGM file and return a numpy array."""
    return imageio.imread(filename)

def write_pgm16(filename, array):
    """Write a 16-bit numpy array to a PGM file."""
    imageio.imwrite(filename, array, format='pgm')

def main(directory):
    # Find all .pgm files in the directory
    pgm_files = [f for f in os.listdir(directory) if f.lower().endswith('.pgm')]
    if not pgm_files:
        print("No .pgm files found in directory.")
        return

    # For each camera index 1..21, find matching files and compute max
    for cam_idx in range(1, 22):
        pattern = re.compile(rf".*_{cam_idx}_.*\.pgm$", re.IGNORECASE)
        cam_files = [f for f in pgm_files if pattern.match(f)]
        if not cam_files:
            print(f"No files found for camera {cam_idx}")
            continue

        max_image = None
        for fname in cam_files:
            img = read_pgm16(os.path.join(directory, fname))
            if max_image is None:
                max_image = img.copy()
            else:
                max_image = np.maximum(max_image, img)

        outname = os.path.join(directory, f"max_{cam_idx}.pgm")
        write_pgm16(outname, max_image)
        print(f"{outname} written ({len(cam_files)} images)")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python max.py <directory>")
    else:
        main(sys.argv[1])