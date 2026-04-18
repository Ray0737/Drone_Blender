import bpy
import math

# --- Configuration ---
START_FRAME = 100
END_FRAME = 200
AMPLITUDE = 1.2    # Lowered height for a "subtle" wave
FREQUENCY = 0.4    # Controls how many drones make up one wave crest
SPEED = 0.05       # Lowered V: Very slow movement per frame
OFFSET_Z = 1.0     # Baseline altitude

def animate_gentle_wave():
    # Filter for your drones (Empties)
    drones = [obj for obj in bpy.context.scene.objects if "Empty" in obj.name]
    
    # Sort by X position
    drones.sort(key=lambda o: o.location.x)

    for frame in range(START_FRAME, END_FRAME + 1):
        bpy.context.scene.frame_set(frame)
        
        for i, drone in enumerate(drones):
            # The calculation: sin((Index * Freq) - (Frame * Speed))
            # Lowering SPEED makes the (Frame * Speed) change very little each step
            z_pos = OFFSET_Z + (AMPLITUDE * math.sin((i * FREQUENCY) - (frame * SPEED)))
            
            drone.location.z = z_pos
            
            # Keyframe only the Z location
            drone.keyframe_insert(data_path="location", index=2)

    print(f"Gentle wave animation baked from {START_FRAME} to {END_FRAME}.")

if __name__ == "__main__":
    animate_gentle_wave()
