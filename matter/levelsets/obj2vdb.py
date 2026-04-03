import sys
sys.path.append('/usr/lib/python3/dist-packages')

import pyopenvdb as vdb
import trimesh # Možná budeš muset doinstalovat: pip install trimesh
import numpy as np

def convert_obj_to_vdb(input_file, output_file, voxel_size=0.03):
    # Load mesh using trimesh (handles OBJ well)
    mesh = trimesh.load(input_file)
    
    # Extract vertices and faces
    vertices = np.array(mesh.vertices, dtype=np.float64)
    faces = np.array(mesh.faces, dtype=np.int32)

    # Create a LevelSet (SDF) from polygons
    # The 'width' is the narrow band around the surface
    transform = vdb.createLinearTransform(voxelSize=voxel_size)
    
    grid = vdb.FloatGrid.createLevelSetFromPolygons(
        vertices,      # points
        faces,         # triangles
        None,          # quads
        transform,     # transform
        3.0            # halfWidth
    )
    
    grid.name = 'surface'
    vdb.write(output_file, grids=[grid])
    print(f"Successfully converted {input_file} to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 obj2vdb.py <filename_without_extension>")
        sys.exit(1)
    
    base_name = sys.argv[1]
    convert_obj_to_vdb(f"{base_name}.obj", f"{base_name}.vdb")