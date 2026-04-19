import bpy
from mathutils import Vector, Matrix

# ---------------- SETTINGS ----------------
TARGET_NAME = " "          # ชื่อ mesh เป้าหมาย
DRONE_NAME = "Drone"       # ชื่อโดรนต้นฉบับ
EDGE_GAP = 2.0
MAX_COUNT = 1100
GAP_INCREMENT = 0.5
NORMAL_SIGN = 1.0

OUTPUT_COLLECTION = "Placed_Drones"
ANCHOR_COLLECTION = "Drone_Anchors"

# ---------------- HELPERS ----------------
def get_or_create_collection(name):
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(coll)
    return coll

def clear_collection(coll):
    for obj in list(coll.objects):
        bpy.data.objects.remove(obj, do_unlink=True)

# ---------------- INITIAL CHECKS ----------------
target = bpy.data.objects.get(TARGET_NAME)
drone_src = bpy.data.objects.get(DRONE_NAME)

if not target or not drone_src:
    raise ValueError(f"Check names: {TARGET_NAME} or {DRONE_NAME} not found.")

out_coll = get_or_create_collection(OUTPUT_COLLECTION)
anchor_coll = get_or_create_collection(ANCHOR_COLLECTION)

# ล้างของเก่า
clear_collection(out_coll)
clear_collection(anchor_coll)

# ---------------- PRE-CALCULATIONS ----------------
bbox_local = [Vector(corner) for corner in drone_src.bound_box]
base_rot = drone_src.matrix_world.to_quaternion()
base_up = (base_rot @ Vector((0, 0, 1))).normalized()

size_right = max(v.x for v in bbox_local) - min(v.x for v in bbox_local)
size_forward = max(v.y for v in bbox_local) - min(v.y for v in bbox_local)
footprint_diameter = max(size_right, size_forward)

# evaluated mesh เพื่อรองรับ rig deformation
depsgraph = bpy.context.evaluated_depsgraph_get()
target_eval = target.evaluated_get(depsgraph)
mesh = target_eval.to_mesh()

world_mat = target.matrix_world
normal_matrix = world_mat.to_3x3().inverted().transposed()

# เก็บ candidate จาก vertex
candidates = []
for i, v in enumerate(mesh.vertices):
    p_world = world_mat @ v.co
    n_world = (normal_matrix @ v.normal).normalized() * NORMAL_SIGN
    candidates.append((i, p_world, n_world))

# เรียงจากสูงลงต่ำ
candidates.sort(key=lambda item: item[1].z, reverse=True)

# ---------------- PLACEMENT ----------------
placing_finished = False
current_gap = EDGE_GAP

while not placing_finished:
    clear_collection(out_coll)
    clear_collection(anchor_coll)

    placed_centers = []
    min_dist = footprint_diameter + current_gap
    placed_count = 0

    for v_idx, point_world, normal_world in candidates:
        too_close = any((point_world - c).length < min_dist for c in placed_centers)
        if too_close:
            continue

        if placed_count >= MAX_COUNT:
            current_gap += GAP_INCREMENT
            break

        # ---------- Anchor ----------
        anchor = bpy.data.objects.new(f"DroneAnchor_{placed_count:04d}", None)
        anchor.empty_display_type = 'PLAIN_AXES'
        anchor.empty_display_size = max(0.1, footprint_diameter * 0.15)
        anchor_coll.objects.link(anchor)

        # parent anchor กับ vertex
        anchor.parent = target
        anchor.parent_type = 'VERTEX'
        anchor.parent_vertices = [v_idx, v_idx, v_idx]
        anchor.location = (0, 0, 0)
        anchor.rotation_mode = 'QUATERNION'
        anchor.rotation_quaternion = Matrix.Identity(4).to_quaternion()

        # เก็บพิกัดเริ่มต้น
        anchor["vertex_index"] = int(v_idx)
        anchor["spawn_x"] = float(point_world.x)
        anchor["spawn_y"] = float(point_world.y)
        anchor["spawn_z"] = float(point_world.z)

        # ---------- Drone ----------
        new_drone = drone_src.copy()
        new_drone.data = drone_src.data  # ใช้ mesh เดิม ประหยัด
        out_coll.objects.link(new_drone)

        # ให้โดรนอยู่ใต้ anchor
        new_drone.parent = anchor
        new_drone.matrix_parent_inverse = Matrix.Identity(4)
        new_drone.location = (0, 0, 0)

        # หมุนให้หันตาม normal ตอนวาง
        new_drone.rotation_mode = 'QUATERNION'
        align_q = base_up.rotation_difference(normal_world.normalized())
        new_drone.rotation_quaternion = align_q @ base_rot

        # เก็บข้อมูลไว้กับโดรนด้วย
        new_drone["vertex_index"] = int(v_idx)
        new_drone["spawn_x"] = float(point_world.x)
        new_drone["spawn_y"] = float(point_world.y)
        new_drone["spawn_z"] = float(point_world.z)

        placed_centers.append(point_world)
        placed_count += 1

    else:
        placing_finished = True

# cleanup
target_eval.to_mesh_clear()
bpy.context.view_layer.update()

print(f"Done! {placed_count} drones created.")
print("Real world position example:")
for obj in list(out_coll.objects)[:5]:
    p = obj.matrix_world.translation
    print(obj.name, round(p.x, 3), round(p.y, 3), round(p.z, 3))
