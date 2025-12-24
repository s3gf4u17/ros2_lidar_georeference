import struct
import sys

def read_binary_points(input_file):
    """
    Read binary file containing point data.
    Each point: x, y, z, intensity, timestamp (5 floats = 20 bytes)
    """
    points = []
    
    with open(input_file, 'rb') as f:
        while True:
            # Read 5 floats (20 bytes)
            chunk = f.read(20)
            if len(chunk) < 20:
                break
            
            # Unpack as 5 floats (little-endian)
            x, y, z, intensity, timestamp = struct.unpack('<5f', chunk)
            points.append((x, y, z, intensity, timestamp))
    
    return points

def write_obj_file(points, output_file):
    """
    Write points to OBJ file format.
    OBJ format uses 'v' for vertex positions.
    """
    with open(output_file, 'w') as f:
        f.write("# Point cloud OBJ file\n")
        f.write(f"# Total points: {len(points)}\n\n")
        
        for x, y, z, intensity, timestamp in points:
            f.write(f"v {x} {y} {z}\n")
    
    print(f"Wrote {len(points)} points to {output_file}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python convert_to_obj.py <input_binary_file> [output_obj_file]")
        print("Example: python convert_to_obj.py points.bin points.obj")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.replace('.bin', '.obj')
    
    print(f"Reading binary file: {input_file}")
    points = read_binary_points(input_file)
    
    if not points:
        print("Error: No points read from file!")
        sys.exit(1)
    
    print(f"Read {len(points)} points")
    print(f"First point: x={points[0][0]:.3f}, y={points[0][1]:.3f}, z={points[0][2]:.3f}, i={points[0][3]:.3f}, t={points[0][4]:.3f}")
    
    write_obj_file(points, output_file)
    print(f"\nDone! Open '{output_file}' in Blender:")
    print("1. File > Import > Wavefront (.obj)")
    print("2. Select your .obj file")
    print("3. In the 3D viewport, the points will appear as vertices")

if __name__ == "__main__":
    main()