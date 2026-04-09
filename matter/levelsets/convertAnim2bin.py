import bpy
import struct
import os

# --- NASTAVENÍ ---
FILENAME = "spatula_motion.bin"
OBJECT_NAME = "Cube_scrape"  # Změň na název tvého objektu
TARGET_FPS = 60

def export_animation():
    obj = bpy.data.objects.get(OBJECT_NAME)
    if not obj:
        print(f"Chyba: Objekt {OBJECT_NAME} nebyl nalezen!")
        return

    # Cesta k souboru (uloží se vedle .blend souboru)
    filepath = os.path.join(bpy.path.abspath("//"), FILENAME)
    
    scene = bpy.context.scene
    # Výpočet kroku pro 60 FPS (pokud má scéna např. 24 FPS)
    # Pro jistotu doporučuji nastavit FPS scény přímo na 60 v Blenderu
    frame_start = scene.frame_start
    frame_end = scene.frame_end

    print(f"Exportuji do: {filepath}")

    with open(filepath, "wb") as f:
        for frame in range(frame_start, frame_end + 1):
            scene.frame_set(frame)
            
            # World matrix transformace
            matrix = obj.matrix_world
            
            # 1. POZICE (X, Y, Z) -> Transformace na Y-up
            # Blender (x, y, z) -> OpenGL (x, z, -y)
            pos = matrix.to_translation()
            gl_pos = (pos.x, pos.z, -pos.y)
            
            # 2. ROTACE (Quaternion) -> Transformace na Y-up
            # Musíme transformovat i orientaci kvasternionu
            quat = matrix.to_quaternion()
            # Jednoduchý převod rotace pro Y-up (prohození komponent)
            gl_quat = (quat.x, quat.z, -quat.y, quat.w) # (x, y, z, w) order
            
            # Zabalení do binárního formátu: 7x float (fffffff)
            # pos[3] + quat[4]
            data = struct.pack('7f', 
                               gl_pos[0], gl_pos[1], gl_pos[2],
                               gl_quat[0], gl_quat[1], gl_quat[2], gl_quat[3])
            f.write(data)

    print("Export dokončen.")

if __name__ == "__main__":
    export_animation()