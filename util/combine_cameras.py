import json
import argparse
import sys

def merge_camera_configs(input_file1, input_file2, output_file):
    """
    Merge camera entries from two JSON configuration files.
    
    Args:
        input_file1: Path to first input JSON configuration file
        input_file2: Path to second input JSON configuration file
        output_file: Path to output JSON file
    """
    try:
        # Read first input JSON file
        with open(input_file1, 'r') as f:
            config1 = json.load(f)
        
        # Read second input JSON file
        with open(input_file2, 'r') as f:
            config2 = json.load(f)
        
        # Validate that both configs have serialNumber
        if 'serialNumber' not in config1:
            print(f"Error: First input file does not contain 'serialNumber' field", file=sys.stderr)
            sys.exit(1)
        
        if 'serialNumber' not in config2:
            print(f"Error: Second input file does not contain 'serialNumber' field", file=sys.stderr)
            sys.exit(1)
        
        # Verify serialNumbers match
        if config1['serialNumber'] != config2['serialNumber']:
            print(f"Error: Serial numbers do not match:", file=sys.stderr)
            print(f"  File 1: {config1['serialNumber']}", file=sys.stderr)
            print(f"  File 2: {config2['serialNumber']}", file=sys.stderr)
            sys.exit(1)
        
        # Validate that both configs have cameras array
        if 'cameras' not in config1:
            print(f"Error: First input file does not contain 'cameras' array", file=sys.stderr)
            sys.exit(1)
        
        if 'cameras' not in config2:
            print(f"Error: Second input file does not contain 'cameras' array", file=sys.stderr)
            sys.exit(1)
        
        # Start with first config as base
        output_config = config1.copy()
        
        # Merge cameras from both files
        # Use first file's cameras as base, then add cameras from second file
        merged_cameras = config1['cameras'].copy()
        merged_cameras.extend(config2['cameras'])
        
        output_config['cameras'] = merged_cameras
        
        # Write output JSON file with proper formatting
        with open(output_file, 'w') as f:
            json.dump(output_config, f, indent=2)
        
        print(f"Successfully merged configurations:")
        print(f"  Serial Number: {config1['serialNumber']}")
        print(f"  Cameras from file 1: {len(config1['cameras'])}")
        print(f"  Cameras from file 2: {len(config2['cameras'])}")
        print(f"  Total cameras: {len(merged_cameras)}")
        print(f"Output written to: {output_file}")
        
    except FileNotFoundError as e:
        print(f"Error: Input file not found: {e}", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in input file: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(
        description='Merge camera entries from two JSON configuration files with matching serial numbers'
    )
    parser.add_argument('input_file1', type=str, help='First input JSON configuration file')
    parser.add_argument('input_file2', type=str, help='Second input JSON configuration file')
    parser.add_argument('output_file', type=str, help='Output JSON file')
    
    args = parser.parse_args()
    
    merge_camera_configs(args.input_file1, args.input_file2, args.output_file)

if __name__ == '__main__':
    main()

