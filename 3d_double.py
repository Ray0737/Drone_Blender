import bpy
from mathutils import Vector

# --- SETTINGS ---
TARGET_NAME = "rp_nathan_animated_003_walking_geo" # Your animated character
DRONE_NAME = "Drone"
EDGE_GAP = 2.0 
MAX_COUNT = 1100 
GAP_INCREMENT = 0.5 
NORMAL_SIGN = 1.0
FORWARD_OFFSET = 20.0 # Distance in meters to move formation forward

# Collections for the two sets
REF_COLLECTION = "Reference_Anchors"  # Invisible set (Section 1)
REAL_COLLECTION = "Placed_Drones"     # Visible formation set (Section 2)

# --- INITIAL CHECKS ---
target = bpy.data.objects.get(TARGET_NAME)
drone_src = bpy.data.objects.get(DRONE_NAME)

if not target or not drone_src:
    raise ValueError(f"Check names: {TARGET_NAME} or {DRONE_NAME} not found.")

def get_or_create_collection(coll_name):
    coll = bpy.data.collections.get(coll_name)
    if not coll:
        coll = bpy.data.collections.new(coll_name)
        bpy.context.scene.collection.children.link(coll)
    return coll

ref_coll = get_or_create_collection(REF_COLLECTION)
real_coll = get_or_create_collection(REAL_COLLECTION)

# --- PRE-CALCULATIONS ---
bbox_local = [Vector(corner) for corner in drone_src.bound_box]
base_rot = drone_src.matrix_world.to_quaternion()
base_up = (base_rot @ Vector((0, 0, 1))).normalized()

# Distance calculation for spacing
size_right = max(v.x for v in bbox_local) - min(v.x for v in bbox_local)
size_forward = max(v.y for v in bbox_local) - min(v.y for v in bbox_local)
footprint_diameter = max(size_right, size_forward)

# Get mesh as it looks during the walk cycle
depsgraph = bpy.context.evaluated_depsgraph_get()
target_eval = target.evaluated_get(depsgraph)
mesh = target_eval.to_mesh()

world_mat = target.matrix_world
inv_world_mat = world_mat.inverted()
normal_matrix = world_mat.to_3x3().inverted().transposed()

candidates = []
for i, v in enumerate(mesh.vertices):
    p_world = world_mat @ v.co
    n_world = (normal_matrix @ v.normal).normalized() * NORMAL_SIGN
    candidates.append((i, p_world, n_world))
candidates.sort(key=lambda item: item[1].z, reverse=True)

# --- PLACEMENT LOOP ---
placing_finished = False
current_gap = EDGE_GAP

while not placing_finished:
    # Cleanup previous attempts
    for coll in [ref_coll, real_coll]:
        for obj in list(coll.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
    
    placed_centers = []
    min_dist = footprint_diameter + current_gap
    
    for v_idx, point_world, normal_world in candidates:
        too_close = any((point_world - c).length < min_dist for c in placed_centers)
        
        if not too_close:
            if len(placed_centers) >= MAX_COUNT:
                current_gap += GAP_INCREMENT
                break 
            
            # --- SECTION 1: INVISIBLE REFERENCE ANCHOR ---
            # Glued to the model's skin using Vertex Parenting
            anchor = bpy.data.objects.new(f"Anchor_{v_idx}", None)
            ref_coll.objects.link(anchor)
            anchor.parent = target
            anchor.parent_type = 'VERTEX'
            anchor.parent_vertices = [v_idx, v_idx, v_idx]
            anchor.location = (0, 0, 0) # Snap exactly to the vertex
            anchor.hide_viewport = True 
            anchor.hide_render = True
            
            # --- SECTION 2: REAL FORMATION DRONE ---
            # Uses real X,Y,Z coords but maintains the formation shape
            real_drone = drone_src.copy()
            real_coll.objects.link(real_drone)
            
            # Position logic: Match the anchor, then move 20m forward
            # Assumes 'forward' is along the Y-axis. Change to .x if needed.
            real_drone.location = (0, FORWARD_OFFSET, 0) 
            
            # Parent the drone to the anchor so it moves with the walk cycle
            real_drone.parent = anchor 
            
            # Set Rotation based on the model's normals
            real_drone.rotation_mode = 'QUATERNION'
            local_normal = inv_world_mat.to_3x3() @ normal_world
            real_drone.rotation_quaternion = Vector((0,0,1)).rotation_difference(local_normal)
            
            placed_centers.append(point_world)
    else:
        placing_finished = True

target_eval.to_mesh_clear()
print(f"System Ready: {len(placed_centers)} drones offset by {FORWARD_OFFSET}m.")
