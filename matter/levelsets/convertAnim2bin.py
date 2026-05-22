import bpy
import struct
import os

# --- SETTINGS ---
FILENAME = "spatula_motion.bin"
OBJECT_NAME = "Cube_scrape"  # Change to the name of your object
TARGET_FPS = 60

def export_animation():
    obj = bpy.data.objects.get(OBJECT_NAME)
    if not obj:
        print(f"Error: Object {OBJECT_NAME} not found!")
        return

    # File path (will be saved next to the .blend file)
    filepath = os.path.join(bpy.path.abspath("//"), FILENAME)
    
    scene = bpy.context.scene
    # Step calculation for 60 FPS (if the scene is e.g. 24 FPS)
    # To be safe, it's recommended to set the scene FPS directly to 60 in Blender
    frame_start = scene.frame_start
    frame_end = scene.frame_end

    print(f"Exporting to: {filepath}")

    with open(filepath, "wb") as f:
        for frame in range(frame_start, frame_end + 1):
            scene.frame_set(frame)
            
            # World matrix transformation
            matrix = obj.matrix_world
            
            # 1. POSITION (X, Y, Z) -> Y-up transformation
            # Blender (x, y, z) -> OpenGL (x, z, -y)
            pos = matrix.to_translation()
            gl_pos = (pos.x, pos.z, -pos.y)
            
            # 2. ROTATION (Quaternion) -> Y-up transformation
            # We also need to transform the quaternion's orientation
            quat = matrix.to_quaternion()
            # Simple rotation conversion for Y-up (swapping components)
            gl_quat = (quat.x, quat.z, -quat.y, quat.w) # (x, y, z, w) order
            
            # Pack into binary format: 7x float (fffffff)
            # pos[3] + quat[4]
            data = struct.pack('7f', 
                               gl_pos[0], gl_pos[1], gl_pos[2],
                               gl_quat[0], gl_quat[1], gl_quat[2], gl_quat[3])
            f.write(data)

    print("Export finished.")

if __name__ == "__main__":
    export_animation()