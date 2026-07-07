import os
import numpy as np
from pathlib import Path
import imageio

def read_pgm_16bit(path):
    return imageio.imread(path)

def write_pgm_16bit(path, array):
    imageio.imwrite(path, array, format='pgm')

def compute_min_image(directory):
    pgm_files = list(Path(directory).glob('*.pgm'))
    if not pgm_files:
        raise RuntimeError("No PGM files found in the directory.")

    # Initialize with the first image
    base_img = imageio.imread(pgm_files[0])
    min_img = base_img.copy()

    for pgm_path in pgm_files[1:]:
        img = imageio.imread(pgm_path)
        min_img = np.minimum(min_img, img)

    write_pgm_16bit('min.pgm', min_img)
    print(f"min.pgm written with shape {min_img.shape}")

# Example usage
if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print("Usage: python pgm_min.py <directory>")
    else:
        compute_min_image(sys.argv[1])

