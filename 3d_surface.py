import bpy
from mathutils import Vector

# --- SETTINGS ---
TARGET_NAME = "  " # Enter file name no need for format | Copy file name from outliner 
DRONE_NAME = "Drone"
EDGE_GAP = 2 # Proximity between drone to drone
MAX_COUNT = 1100  # Drone QTY
GAP_INCREMENT = 0.5  # How much to widen the gap if over limit
NORMAL_SIGN = 1.0
OUTPUT_COLLECTION = "Placed_Drones"

# --- INITIAL CHECKS ---
target = bpy.data.objects.get(TARGET_NAME)
drone_src = bpy.data.objects.get(DRONE_NAME)

if not target or not drone_src:
    raise ValueError("Check target or drone object names.")

out_coll = bpy.data.collections.get(OUTPUT_COLLECTION)
if out_coll is None:
    out_coll = bpy.data.collections.new(OUTPUT_COLLECTION)
    bpy.context.scene.collection.children.link(out_coll)

# --- PRE-CALCULATIONS ---
bbox_local = [Vector(corner) for corner in drone_src.bound_box]
M = drone_src.matrix_world.copy()
M.translation = Vector((0.0, 0.0, 0.0))
bbox_world_flat = [M @ c for c in bbox_local]

base_rot = drone_src.matrix_world.to_quaternion()
base_up = (base_rot @ Vector((0, 0, 1))).normalized()
base_right = (base_rot @ Vector((1, 0, 0))).normalized()
base_forward = (base_rot @ Vector((0, 1, 0))).normalized()

min_proj = min(v.dot(base_up) for v in bbox_world_flat)
size_right = max(v.dot(base_right) for v in bbox_world_flat) - min(v.dot(base_right) for v in bbox_world_flat)
size_forward = max(v.dot(base_forward) for v in bbox_world_flat) - min(v.dot(base_forward) for v in bbox_world_flat)
footprint_diameter = max(size_right, size_forward)

# Prepare Mesh Data
depsgraph = bpy.context.evaluated_depsgraph_get()
target_eval = target.evaluated_get(depsgraph)
mesh = target_eval.to_mesh()
normal_matrix = target.matrix_world.to_3x3().inverted().transposed()

candidates = []
for v in mesh.vertices:
    p_world = target.matrix_world @ v.co
    n_world = (normal_matrix @ v.normal).normalized() * NORMAL_SIGN
    candidates.append((p_world, n_world))
candidates.sort(key=lambda item: item[0].z, reverse=True)

# --- PLACEMENT LOOP WITH AUTO-GAP ADJUSTMENT ---
placing_finished = False
current_gap = EDGE_GAP

while not placing_finished:
    # Clear previous attempt
    for obj in list(out_coll.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    
    placed_centers = []
    temp_objects = []
    min_dist = footprint_diameter + current_gap
    
    print(f"Attempting placement with Gap: {current_gap:.2f}...")

    for point, normal in candidates:
        too_close = any((point - c).length < min_dist for c in placed_centers)
        
        if not too_close:
            # Check if this placement would exceed MAX_COUNT
            if len(placed_centers) >= MAX_COUNT:
                print(f"Limit exceeded ({MAX_COUNT}). Widening gap...")
                current_gap += GAP_INCREMENT
                break # Break inner loop to restart with wider gap
            
            # Create instance
            q_align = base_up.rotation_difference(normal)
            new_obj = drone_src.copy()
            new_obj.rotation_mode = 'QUATERNION'
            new_obj.rotation_quaternion = q_align @ base_rot
            new_obj.location = point - (min_proj * normal)
            
            temp_objects.append(new_obj)
            placed_centers.append(point)
    else:
        # This block executes if the inner loop finishes WITHOUT hitting 'break'
        # Meaning the count is within MAX_COUNT
        for obj in temp_objects:
            out_coll.objects.link(obj)
        placing_finished = True

print(f"Final Result: {len(placed_centers)} drones placed with a gap of {current_gap:.2f}")
target_eval.to_mesh_clear()
