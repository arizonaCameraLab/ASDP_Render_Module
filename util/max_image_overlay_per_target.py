import os
import numpy as np
import imageio
import re
import csv

def read_pgm16(filename):
    """Read a 16-bit PGM file and return a numpy array."""
    return imageio.imread(filename)

def write_pgm16(filename, array):
    """Write a 16-bit numpy array to a PGM file."""
    imageio.imwrite(filename, array, format='pgm')

def main(csv_file, directory):
    # Parse the CSV file to get information about which frame is using which target and camera.
    # Also get the list of available targets.
    target_frames = {}
    cameras = set()
    targets = set()
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(row['FrameIndex'])
            target = row['TargetID']
            targets.add(target)
            camera = int(row['Camera'])
            cameras.add(camera)
            if target not in target_frames:
                target_frames[target] = []
            target_frames[target].append((frame, camera))

    # Sort the cameras and targets for consistent processing order
    cameras = sorted(cameras)
    targets = sorted(targets)

    # Find all .pgm files in the directory
    pgm_files = [f for f in os.listdir(directory) if f.lower().endswith('.pgm')]
    if not pgm_files:
        print("No .pgm files found in directory.")
        return

    # For each target and each camera index, find matching files and compute max
    for target in targets:
        for cam_idx in cameras:
            # Find all of the frames that use this target and camera
            frames = [f for f, c in target_frames[target] if c == cam_idx]
            cam_files = []
            print('Finding frames for target', target, 'camera', cam_idx)
            for frame in frames:
                pattern = re.compile(rf"{frame}_{cam_idx}_.*\.pgm$", re.IGNORECASE)
                # Add all of the new files that match this frame and camera to the list of files to process
                cam_files.extend([f for f in pgm_files if pattern.match(f)])
            if len(cam_files) == 0:
                print(f"No files found for camera {cam_idx}, target {target}")
                continue

            max_image = None
            for fname in cam_files:
                img = read_pgm16(os.path.join(directory, fname))
                if max_image is None:
                    max_image = img.copy()
                else:
                    max_image = np.maximum(max_image, img)

            outname = os.path.join(directory, f"max_{cam_idx}_{target}.pgm")
            write_pgm16(outname, max_image)
            print(f"{outname} written ({len(cam_files)} images)")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python max.py <poses.csv file> <directory>")
    else:
        main(sys.argv[1], sys.argv[2])