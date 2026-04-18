import bpy
from mathutils import Vector

TARGET_NAME = "ชื่อไฟล์"
DRONE_NAME = "Drone"
EDGE_GAP = 1.8
MAX_COUNT = 100000
NORMAL_SIGN = 1.0
OUTPUT_COLLECTION = "Drones"

target = bpy.data.objects.get(TARGET_NAME)
drone_src = bpy.data.objects.get(DRONE_NAME)

if target is None:
    raise ValueError(f"ไม่เจอ target object: {TARGET_NAME}")

if drone_src is None:
    raise ValueError(f"ไม่เจอ drone object: {DRONE_NAME}")

if target.type != 'MESH':
    raise TypeError(f"{TARGET_NAME} ไม่ใช่ MESH")

if drone_src.type != 'MESH':
    raise TypeError(f"{DRONE_NAME} ไม่ใช่ MESH")

out_coll = bpy.data.collections.get(OUTPUT_COLLECTION)
if out_coll is None:
    out_coll = bpy.data.collections.new(OUTPUT_COLLECTION)
    bpy.context.scene.collection.children.link(out_coll)

for obj in list(out_coll.objects):
    bpy.data.objects.remove(obj, do_unlink=True)

bbox_local = [Vector(corner) for corner in drone_src.bound_box]

M = drone_src.matrix_world.copy()
M.translation = Vector((0.0, 0.0, 0.0))
bbox_world_flat = [M @ c for c in bbox_local]

base_rot = drone_src.matrix_world.to_quaternion()
base_up = (base_rot @ Vector((0, 0, 1))).normalized()
base_right = (base_rot @ Vector((1, 0, 0))).normalized()
base_forward = (base_rot @ Vector((0, 1, 0))).normalized()

min_proj = min(v.dot(base_up) for v in bbox_world_flat)

right_vals = [v.dot(base_right) for v in bbox_world_flat]
forward_vals = [v.dot(base_forward) for v in bbox_world_flat]

size_right = max(right_vals) - min(right_vals)
size_forward = max(forward_vals) - min(forward_vals)

footprint_diameter = max(size_right, size_forward)
min_center_distance = footprint_diameter + EDGE_GAP

print("==== DRONE INFO ====")
print("footprint_diameter =", footprint_diameter)
print("min_center_distance =", min_center_distance)
print("min_proj =", min_proj)

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

placed_centers = []
placed_count = 0

for point, normal in candidates:
    if placed_count >= MAX_COUNT:
        break

    too_close = False
    for c in placed_centers:
        if (point - c).length < min_center_distance:
            too_close = True
            break

    if too_close:
        continue

    q_align = base_up.rotation_difference(normal)
    final_rot = q_align @ base_rot

    new_obj = drone_src.copy()
    new_obj.data = drone_src.data
    new_obj.animation_data_clear()

    new_obj.rotation_mode = 'QUATERNION'
    new_obj.rotation_quaternion = final_rot
    new_obj.location = point - (min_proj * normal)

    out_coll.objects.link(new_obj)

    placed_centers.append(point)
    placed_count += 1

print(f"Placed {placed_count} drones on {TARGET_NAME}")

target_eval.to_mesh_clear()
