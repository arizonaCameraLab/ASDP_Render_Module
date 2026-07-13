import json
import argparse
import sys

def filter_cameras(input_file, lower_id, upper_id, output_file):
    """
    Filter cameras from input JSON config file based on ID range.
    
    Args:
        input_file: Path to input JSON configuration file
        lower_id: Lower bound of camera ID range (inclusive)
        upper_id: Upper bound of camera ID range (inclusive)
        output_file: Path to output JSON file
    """
    try:
        # Read input JSON file
        with open(input_file, 'r') as f:
            config = json.load(f)
        
        # Validate that the config has a cameras array
        if 'cameras' not in config:
            print(f"Error: Input file does not contain 'cameras' array", file=sys.stderr)
            sys.exit(1)
        
        # Filter cameras based on ID range
        filtered_cameras = [
            camera for camera in config['cameras']
            if 'id' in camera and lower_id <= camera['id'] <= upper_id
        ]
        
        # Create output configuration with filtered cameras
        output_config = {
            key: value for key, value in config.items()
            if key != 'cameras'
        }
        output_config['cameras'] = filtered_cameras
        
        # Write output JSON file with proper formatting
        with open(output_file, 'w') as f:
            json.dump(output_config, f, indent=2)
        
        print(f"Successfully filtered {len(filtered_cameras)} camera(s) with IDs in range [{lower_id}, {upper_id}]")
        print(f"Output written to: {output_file}")
        
    except FileNotFoundError:
        print(f"Error: Input file '{input_file}' not found", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in input file: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(
        description='Filter camera entries from JSON config file based on ID range'
    )
    parser.add_argument('input_file', type=str, help='Input JSON configuration file')
    parser.add_argument('lower_id', type=int, help='Lower bound of camera ID range (inclusive)')
    parser.add_argument('upper_id', type=int, help='Upper bound of camera ID range (inclusive)')
    parser.add_argument('output_file', type=str, help='Output JSON file')
    
    args = parser.parse_args()
    
    # Validate ID range
    if args.lower_id > args.upper_id:
        print(f"Error: Lower ID ({args.lower_id}) cannot be greater than upper ID ({args.upper_id})", file=sys.stderr)
        sys.exit(1)
    
    filter_cameras(args.input_file, args.lower_id, args.upper_id, args.output_file)

if __name__ == '__main__':
    main()

