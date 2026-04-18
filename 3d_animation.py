import bpy
from mathutils import Vector

# --- SETTINGS ---
TARGET_NAME = " " # File name 
DRONE_NAME = "Drone"
EDGE_GAP = 2.0 
MAX_COUNT = 1100 
GAP_INCREMENT = 0.5 
NORMAL_SIGN = 1.0
OUTPUT_COLLECTION = "Placed_Drones"

# --- INITIAL CHECKS ---
target = bpy.data.objects.get(TARGET_NAME)
drone_src = bpy.data.objects.get(DRONE_NAME)

if not target or not drone_src:
    raise ValueError(f"Check names: {TARGET_NAME} or {DRONE_NAME} not found.")

out_coll = bpy.data.collections.get(OUTPUT_COLLECTION)
if out_coll is None:
    out_coll = bpy.data.collections.new(OUTPUT_COLLECTION)
    bpy.context.scene.collection.children.link(out_coll)

# --- PRE-CALCULATIONS ---
bbox_local = [Vector(corner) for corner in drone_src.bound_box]
base_rot = drone_src.matrix_world.to_quaternion()
base_up = (base_rot @ Vector((0, 0, 1))).normalized()

size_right = max(v.x for v in bbox_local) - min(v.x for v in bbox_local)
size_forward = max(v.y for v in bbox_local) - min(v.y for v in bbox_local)
footprint_diameter = max(size_right, size_forward)

# Get evaluated mesh (handles rig deformation)
depsgraph = bpy.context.evaluated_depsgraph_get()
target_eval = target.evaluated_get(depsgraph)
mesh = target_eval.to_mesh()

world_mat = target.matrix_world
inv_world_mat = world_mat.inverted() # Used to bring world points to local space
normal_matrix = world_mat.to_3x3().inverted().transposed()

candidates = []
for i, v in enumerate(mesh.vertices):
    p_world = world_mat @ v.co
    n_world = (normal_matrix @ v.normal).normalized() * NORMAL_SIGN
    candidates.append((i, p_world, n_world))
candidates.sort(key=lambda item: item[1].z, reverse=True)

# --- PLACEMENT & STICKY LOGIC ---
placing_finished = False
current_gap = EDGE_GAP

while not placing_finished:
    # Cleanup previous attempt
    for obj in list(out_coll.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    
    placed_centers = []
    temp_data = [] 
    min_dist = footprint_diameter + current_gap
    
    for v_idx, point_world, normal_world in candidates:
        too_close = any((point_world - c).length < min_dist for c in placed_centers)
        
        if not too_close:
            if len(placed_centers) >= MAX_COUNT:
                current_gap += GAP_INCREMENT
                break 
            
            # Create instance
            new_drone = drone_src.copy()
            out_coll.objects.link(new_drone)
            
            # 1. SET PARENT FIRST
            new_drone.parent = target
            new_drone.parent_type = 'VERTEX'
            new_drone.parent_vertices = [v_idx, v_idx, v_idx]
            
            # 2. CALCULATE LOCAL POSITION
            # This is the "Fix": Placing the drone relative to the vertex, not the world
            new_drone.location = (0, 0, 0) 
            
            # 3. FIX ROTATION
            new_drone.rotation_mode = 'QUATERNION'
            # Convert world normal direction to local rotation
            local_normal = inv_world_mat.to_3x3() @ normal_world
            new_drone.rotation_quaternion = Vector((0,0,1)).rotation_difference(local_normal)
            
            placed_centers.append(point_world)
    else:
        placing_finished = True

target_eval.to_mesh_clear()
print(f"Fixed! {len(placed_centers)} drones locked to skin.")
