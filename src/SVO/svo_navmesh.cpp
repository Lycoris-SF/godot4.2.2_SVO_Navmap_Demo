#include "svo_navmesh.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/godot.hpp>
#define global_min_depth 1
#define global_max_depth 9

using namespace godot;

void SvoNavmesh::_bind_methods() {
    ClassDB::bind_method(D_METHOD("insert_voxel", "position"), &SvoNavmesh::insert_voxel);
    ClassDB::bind_method(D_METHOD("query_voxel", "position"), &SvoNavmesh::query_voxel);
    ClassDB::bind_method(D_METHOD("update_voxel", "position", "isSolid"), &SvoNavmesh::update_voxel);
    ClassDB::bind_method(D_METHOD("get_info"), &SvoNavmesh::get_info);
    ClassDB::bind_method(D_METHOD("set_info", "p_info"), &SvoNavmesh::set_info);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "testdouble"), "set_info", "get_info");
    //ClassDB::bind_method(D_METHOD("find_path", "start", "end"), &SvoNavmesh::find_path);

    ClassDB::bind_method(D_METHOD("get_origin_position"), &SvoNavmesh::get_offset_position);
    ClassDB::bind_method(D_METHOD("set_origin_position", "position"), &SvoNavmesh::set_offset_position);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::VECTOR3, "origin_position"), "set_origin_position", "get_origin_position");

    ClassDB::bind_method(D_METHOD("get_origin_rotation"), &SvoNavmesh::get_offset_rotation);
    ClassDB::bind_method(D_METHOD("set_origin_rotation", "rotation"), &SvoNavmesh::set_offset_rotation);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::VECTOR3, "origin_rotation"), "set_origin_rotation", "get_origin_rotation");

    ClassDB::bind_method(D_METHOD("get_voxel_size"), &SvoNavmesh::get_voxel_size);
    ClassDB::bind_method(D_METHOD("set_voxel_size", "size"), &SvoNavmesh::set_voxel_size);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "voxel_size"), "set_voxel_size", "get_voxel_size");

    ClassDB::bind_method(D_METHOD("get_max_depth"), &SvoNavmesh::get_max_depth);
    ClassDB::bind_method(D_METHOD("set_max_depth", "depth"), &SvoNavmesh::set_max_depth);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::INT, "max_depth"), "set_max_depth", "get_max_depth");

    // debug draw property
    ClassDB::bind_method(D_METHOD("get_DR_min_depth"), &SvoNavmesh::get_DR_min_depth);
    ClassDB::bind_method(D_METHOD("set_DR_min_depth", "depth"), &SvoNavmesh::set_DR_min_depth);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::INT, "DrawRef_minDepth"), "set_DR_min_depth", "get_DR_min_depth");
    ClassDB::bind_method(D_METHOD("get_DR_max_depth"), &SvoNavmesh::get_DR_max_depth);
    ClassDB::bind_method(D_METHOD("set_DR_max_depth", "depth"), &SvoNavmesh::set_DR_max_depth);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::INT, "DrawRef_maxDepth"), "set_DR_max_depth", "get_DR_max_depth");

    // button method
    ClassDB::bind_method(D_METHOD("rebuild_svo"), &SvoNavmesh::rebuild_svo);
    ClassDB::bind_method(D_METHOD("refresh_svo"), &SvoNavmesh::refresh_svo);
    ClassDB::bind_method(D_METHOD("clear_svo"), &SvoNavmesh::clear_svo);

    // generate svo from collider
    ClassDB::bind_method(D_METHOD("insert_svo_based_on_collision_shapes"), &SvoNavmesh::insert_svo_based_on_collision_shapes);
}

SvoNavmesh::SvoNavmesh(){
    offset_position = Vector3(0, 0, 0);
    offset_rotation = Vector3(0, 0, 0);
    maxDepth = 3;
    voxelSize = 1.0f;
    testdouble = 1.14;

    DrawRef_minDepth = 1;
    DrawRef_maxDepth = 3;

    svo = memnew(SparseVoxelOctree);
    init_neighbors();
    init_debugMesh(svo->root, 1);
}

SvoNavmesh::~SvoNavmesh() {
    memdelete(svo);
    //reset_wastepool();
}

// During set_neighbors_from_brother, the index of each child node corresponds to the index in the neighbors array.
// set_neighbors_from_brother时，每个子节点索引对应在neighbors数组中的索引
static const int neighborsMap[8][3] = {
    {0, 2, 4}, // 索引0的邻居在neighbors数组的索引
    {1, 2, 4}, // The index of neighbor for index 1 in the neighbors array.
    {0, 3, 4},
    {1, 3, 4},
    {0, 2, 5},
    {1, 2, 5},
    {0, 3, 5},
    {1, 3, 5}
};
// During set_neighbors_from_brother, each child node corresponds to an index in the father node's children array.
// set_neighbors_from_brother时，，每个子节点对应父节点的children数组中的索引
static const int childrenMap[8][3] = {
    {1, 2, 4}, // 子节点0需要查找的父节点的子节点索引
    {0, 3, 5}, // The index of the child node in the father node that needs to be searched for child node 1.
    {3, 0, 6},
    {2, 1, 7},
    {5, 6, 0},
    {4, 7, 1},
    {7, 4, 2},
    {6, 5, 3}
};

/**
 * Converts a world position to a local grid position.
 *
 * @param world_position: The position in world coordinates.
 * @returns The position in local coordinates of svo.
 */
Vector3 SvoNavmesh::worldToGrid(Vector3 world_position) {
    Vector3 euler_angles_radians(
        // Convert Euler angles from degrees to radians
        // 将欧拉角从度转换为弧度
        Math::deg_to_rad(offset_rotation.x),
        Math::deg_to_rad(offset_rotation.y),
        Math::deg_to_rad(offset_rotation.z)
    );

    Transform3D global_transform = get_global_transform();
    Vector3 local_position = global_transform.xform_inv(world_position); // Global to local; 全局到局部
    local_position = Quaternion(euler_angles_radians).inverse().xform(local_position - offset_position); // Applying offsets; 应用偏移和旋转
    return local_position / voxelSize; // Convert to grid coordinates; 转换到网格坐标
}
bool globalDepthCheck(int depth) {
    if (depth< global_min_depth || depth>global_max_depth) return false;
    else return true;
}

/**
 * Inserts a voxel from the specified world position.
 *
 * @param world_position: The world position where the voxel is to be inserted.
 */
void SvoNavmesh::insert_voxel(Vector3 world_position) {
    Vector3 grid_position = worldToGrid(world_position);

    // Check if the grid coordinates are valid (i.e. within the root node range)
    // 检查网格坐标是否有效(即在根节点范围内)
    if (grid_position.x < -voxelSize / 2 || grid_position.y < -voxelSize / 2 || grid_position.z < -voxelSize / 2 ||
        grid_position.x >= voxelSize / 2 || grid_position.y >= voxelSize / 2 || grid_position.z >= voxelSize / 2) {
        WARN_PRINT("Voxel coordinates out of valid range.");
        return;
    }

    refresh_svo();
    svo->insert(grid_position);
    reset_pool();
    init_debugMesh(svo->root, 1);
    init_neighbors();
}
/**
 * Query a voxel from the specified world position.
 *
 * @param world_position: The world position where the voxel is to be queried.
 */
bool SvoNavmesh::query_voxel(Vector3 world_position) {
    Vector3 grid_position = worldToGrid(world_position);
    return svo->query(grid_position);
}
//临时弃用; Temp Abandon
void SvoNavmesh::update_voxel(Vector3 world_position, bool isSolid) {
    Vector3 grid_position = worldToGrid(world_position);

    // Check if the grid coordinates are valid (i.e. within the root node range)
    // 检查网格坐标是否有效(即在根节点范围内)
    if (grid_position.x < -voxelSize / 2 || grid_position.y < -voxelSize / 2 || grid_position.z < -voxelSize / 2 ||
        grid_position.x >= voxelSize / 2 || grid_position.y >= voxelSize / 2 || grid_position.z >= voxelSize / 2) {
        WARN_PRINT("Voxel coordinates out of valid range.");
        return;
    }

    refresh_svo();
    svo->update(grid_position, isSolid);
}

Vector3 SvoNavmesh::get_offset_position() const {
    return offset_position;
}
void SvoNavmesh::set_offset_position(Vector3 position) {
    offset_position = position;
}

Vector3 SvoNavmesh::get_offset_rotation() const {
    return offset_rotation;
}
void SvoNavmesh::set_offset_rotation(Vector3 rotation) {
    offset_rotation = rotation;
}

float SvoNavmesh::get_voxel_size() const {
    return voxelSize;
}
void SvoNavmesh::set_voxel_size(float size) {
    if (voxelSize != size) {
        voxelSize = size;
    }
}

int SvoNavmesh::get_max_depth() const {
    return maxDepth;
}
void SvoNavmesh::set_max_depth(int depth) {
    if (!globalDepthCheck(depth)) {
        WARN_PRINT("Depth out of setting range.(1-9)");
        return;
    }
    if (maxDepth != depth) {
        maxDepth = depth;
    }
}

// <debug draw set/get>
int SvoNavmesh::get_DR_min_depth() const {
    return DrawRef_minDepth;
}
void SvoNavmesh::set_DR_min_depth(int depth) {
    if (!globalDepthCheck(depth)) {
        WARN_PRINT("Depth out of setting range.(1-9)");
        return;
    }
    if (depth > DrawRef_maxDepth) {
        DrawRef_minDepth = DrawRef_maxDepth;
        return;
    }
    DrawRef_minDepth = depth;
}
int SvoNavmesh::get_DR_max_depth() const {
    return DrawRef_maxDepth;
}
void SvoNavmesh::set_DR_max_depth(int depth) {
    if (!globalDepthCheck(depth)) {
        WARN_PRINT("Depth out of setting range.(1-9)");
        return;
    }
    if (depth > maxDepth) {
        DrawRef_maxDepth = maxDepth;
        return;
    }
    DrawRef_maxDepth = depth;
}
// </debug draw set/get>

/**
 Clear the svo and total rebuild base on collision shapes.
 */
void SvoNavmesh::rebuild_svo() {
    uint64_t begin = Time::get_singleton()->get_ticks_msec();

    svo->clear();
    reset_pool();
    init_debugMesh(svo->root, 1);

    insert_svo_based_on_collision_shapes();
    reset_pool();
    init_debugMesh(svo->root, 1);
    init_neighbors();

    uint64_t end = Time::get_singleton()->get_ticks_msec();
    WARN_PRINT_ED(vformat("build svo with %d milliseconds", end - begin));
}

/**
 Refresh the svo base on svo setting changes.
 */
void SvoNavmesh::refresh_svo() {
    if (svo->maxDepth != maxDepth) {
        if (maxDepth < svo->maxDepth) {
            // compress
            svo->maxDepth = maxDepth;
            svo->compress_node();
        }
        else {
            // expand
            svo->maxDepth = maxDepth;
            svo->expand_node();
        }
    }
    if (svo->voxelSize != voxelSize) {
        svo->voxelSize = voxelSize;
        // refresh center
        svo->update_node_centers();
    }
    reset_pool();
    init_debugMesh(svo->root, 1);
    init_neighbors();
}
/**
 * Clear the svo.
 *
 * @param clear_setting: whether also clear svo setting.
 */
void SvoNavmesh::clear_svo(bool clear_setting) {
    if (clear_setting) {
        // clear svo and settings
        offset_position = Vector3(0, 0, 0);
        offset_rotation = Vector3(0, 0, 0);
        maxDepth = 3;
        voxelSize = 1.0f;
        svo->clear();
    }
    else {
        // clear only svo
        svo->clear();
    }
    reset_pool();
    init_debugMesh(svo->root, 1);
    init_neighbors();
}

/**
 Insert after collect.
 */
void build_svo(SparseVoxelOctree* svo, const Vector<Vector3>& points) {
    for (int i = 0; i < points.size(); ++i) {
        svo->insert(points[i]);
    }
}
/**
 Generate svo from collider.
 */
void SvoNavmesh::insert_svo_based_on_collision_shapes() {
    uint64_t begin = Time::get_singleton()->get_ticks_msec();

    Node* parent_node = get_parent();
    RID space_rid = this->get_world_3d()->get_space();
    if (parent_node != nullptr) {
        collect_collision_shapes(parent_node, space_rid);
    }

    uint64_t end = Time::get_singleton()->get_ticks_msec();
    WARN_PRINT_ED(vformat("insert svo nodes with %d milliseconds", end - begin));
}
/**
 Use Godot's physics engine to check if the point is inside CollisionShape3D.
 */
bool check_point_inside_mesh(Vector3 point, RID& space_rid, RID& target_rid) {
    // Get the PhysicsDirectSpaceState3D instance
    // 获取 PhysicsDirectSpaceState3D 实例
    PhysicsDirectSpaceState3D* space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(space_rid);
    if (!space_state) {
        ERR_PRINT_ED("Failed to get PhysicsDirectSpaceState3D instance");
        return false;
    }

    // Create and configure query parameters
    // 创建并配置查询参数
    Ref<PhysicsPointQueryParameters3D> query_params;
    query_params.instantiate();
    query_params->set_position(point);  // Set the position of the query point; 设置查询点的位置

    // Execute query
    // 执行点内查询
    Array results = space_state->intersect_point(query_params, 32);  // max_results = 32

    // Process the query results
    // 处理查询结果
    if (!results.is_empty()) {
        for (int i = 0; i < results.size(); ++i) {
            Dictionary result = results[i];
            // Compare collider object RID
            // 对比 collider 对象RID
            RID result_rid = result["rid"];
            if (result_rid == target_rid) {
                return true;
            }
        }
    }
    return false;
}
/**
 Interval Scan svo to get points.
 */
Vector<Vector3> sample_collision_shape(Ref<ArrayMesh> mesh, Vector3 position, float size, float sample_interval, RID& space_rid, RID& target_rid) {
    Vector<Vector3> sampled_points;
    
    // position is the center of svo, size is voxelSize
    for (float x = position.x - size / 2; x < position.x + size / 2; x += sample_interval) {
        for (float y = position.y - size / 2; y < position.y + size / 2; y += sample_interval) {
            for (float z = position.z - size / 2; z < position.z + size / 2; z += sample_interval) {
                Vector3 point = Vector3(x, y, z);

                // Check if the point is inside the grid
                // 检测点是否在网格内部
                if (check_point_inside_mesh(point, space_rid, target_rid)) {
                    sampled_points.push_back(point);
                }
            }
        }
    }

    // Processing surface vertices
    // 处理表面顶点
    PackedVector3Array faces = mesh->get_faces();
    for (int i = 0; i < faces.size(); i++) {
        sampled_points.push_back(faces[i]);
    }

    return sampled_points;
}
//已弃用; Abandon
Vector<Vector3> get_box_corners(CollisionShape3D* collision_shape) {
    Vector<Vector3> corners;
    Ref<BoxShape3D> box_shape = collision_shape->get_shape();

    if (box_shape.is_valid()) {
        Vector3 size = box_shape->get_size();
        Vector3 extents = size * 0.5; // 计算extents
        Transform3D global_transform = collision_shape->get_global_transform();

        // 计算8个角点
        for (int x = -1; x <= 1; x += 2) {
            for (int y = -1; y <= 1; y += 2) {
                for (int z = -1; z <= 1; z += 2) {
                    Vector3 corner = extents * Vector3(x, y, z);
                    corners.push_back(global_transform.xform(corner));
                }
            }
        }
    }

    return corners;
}
/**
 Collect Points from CollisionShape3D.
 */
Vector<Vector3> get_shape_points(CollisionShape3D* collision_shape, Vector3 position, float size, float sample_interval, RID& space_rid, RID& target_rid) {
    Vector<Vector3> points;
    Ref<Shape3D> shape = collision_shape->get_shape();

    if (shape.is_valid()) {
        Ref<ArrayMesh> debug_mesh = shape.ptr()->get_debug_mesh();
        points = sample_collision_shape(debug_mesh, position, size, sample_interval, space_rid, target_rid);
    }

    return points;
}
/**
 Collect CollisionShape3D from PhysicsBody3D.
 */
void SvoNavmesh::collect_collision_shapes(Node* node, RID& space_rid) {
    if (!node) return;

    // Traverse each node in the subtree
    // 遍历子树中的每个节点
    for (int i = 0; i < node->get_child_count(); ++i) {
        Node* child = node->get_child(i);

        // Try to convert the child node to PhysicsBody3D
        // 尝试将子节点转换为PhysicsBody3D
        PhysicsBody3D* physics_body = Object::cast_to<PhysicsBody3D>(child);
        if (physics_body) {
            // Traverse the child nodes of PhysicsBody3D to find CollisionShape3D
            // 遍历PhysicsBody3D的子节点来查找CollisionShape3D
            for (int j = 0; j < physics_body->get_child_count(); ++j) {
                Node* subChild = physics_body->get_child(j);
                CollisionShape3D* collision_shape = Object::cast_to<CollisionShape3D>(subChild);
                if (collision_shape && !collision_shape->is_disabled()) {
                    RID target_rid = physics_body->get_rid();
                    Vector<Vector3> points = get_shape_points(collision_shape, get_global_position(), voxelSize, svo->calActualVoxelSize(svo->maxDepth) / 2, space_rid, target_rid);
                    build_svo(svo, points);
                }
            }
        }

        // Recursively collect collision shapes of child nodes
        // 递归地收集子节点的碰撞形状
        collect_collision_shapes(child, space_rid);
    }
}

SparseVoxelOctree& SvoNavmesh::get_svo() {
    return *svo;
}

void SvoNavmesh::set_info(float p_info) {
    testdouble = p_info;
}
float SvoNavmesh::get_info() {
    return testdouble;
}

// override
void SvoNavmesh::_process(double delta) {
    // 开销较小，但存在godot子节点导致的体积错误
    //clear_mesh_instances();
    //draw_svo(svo->root, 1, 1, maxDepth);

    // 开销较大，但不会牵涉到godot子节点体积问题
    /*for (MeshInstance3D* instance : mesh_pool) {
        remove_child(instance);
        memdelete(instance); // 确保释放内存
    }
    mesh_pool.clear();*/
    // 以上仅用于draw_svo_v1(废弃)
    // The above is only for draw_svo_v1(abandon)

    draw_svo_v2(svo->root, 1, DrawRef_minDepth, DrawRef_maxDepth);
}
void SvoNavmesh::_physics_process(double delta)
{
    //reset_wastepool();
}
static void init_static_material()
{
    // debug draw
    if (debugSolidMaterial.is_null()) debugSolidMaterial.instantiate();
    debugSolidMaterial->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
    debugSolidMaterial->set_albedo(Color(0.2, 1.0, 0.2, 0.2));  // 半透明绿色
    if (debugEmptyMaterial.is_null()) debugEmptyMaterial.instantiate();
    if (EmptyMaterial_shader.is_null()) {
        EmptyMaterial_shader.instantiate();
        EmptyMaterial_shader->set_code(R"(
            shader_type spatial;
            render_mode wireframe,cull_disabled;

            void fragment() {
                ALBEDO = vec3(1.0, 0.0, 0.0);
            }
        )");
    }
    debugEmptyMaterial->set_shader(EmptyMaterial_shader);
}
static void clear_static_material() {
    debugSolidMaterial.unref();
    debugEmptyMaterial.unref();
    EmptyMaterial_shader.unref();
}
void SvoNavmesh::_enter_tree()
{
    init_static_material();

    //Node3D::_enter_tree();
}
void SvoNavmesh::_exit_tree(){
    clear_static_material();
    //Node3D::_exit_tree();
}

/**
 This method needs to be called manually if changes are involved in the svo structure.
 牵涉到svo结构变化需要手动调用此方法
 */
void SvoNavmesh::reset_pool() {
    if (!mesh_pool.is_empty()) {
        for (MeshInstance3D* instance : mesh_pool) {
            if (instance && instance->is_inside_tree()) {
                remove_child(instance);
                //if (!waste_pool.has(instance)) waste_pool.push_back(instance);
                //instance->queue_free();
            }
        }
        mesh_pool.clear();
    }

    //
    //UtilityFunctions::print(vformat("Waste pool count: %d", waste_pool.size()));
}
//已弃用; Abandon
void SvoNavmesh::reset_wastepool() {
    if (!waste_pool.is_empty()) {
        for (MeshInstance3D* instance : waste_pool) {
            instance->queue_free();
        }
        waste_pool.clear();
    }
}
/**
 This method needs to be called manually if changes are involved in the svo structure.
 牵涉到svo结构变化需要手动调用此方法
 */
void SvoNavmesh::init_debugMesh(OctreeNode* node, int depth)
{
    if (!node || depth > svo->maxDepth) return;

    float size = 2 * voxelSize / pow(2, depth);

    if (depth <= svo->maxDepth) {
        /*if (!node->debugMesh) {
            node->debugMesh = memnew(MeshInstance3D);
            mesh_count_log.test_countA++;
        }*/
        if (node->debugBoxMesh.is_null()) node->debugBoxMesh = Ref<BoxMesh>(memnew(BoxMesh));
        node->debugBoxMesh->set_size(Vector3(size, size, size));
        node->debugMesh.set_mesh(node->debugBoxMesh);

        // Set up materials
        // 设置材料
        if (node->voxel && node->voxel->isSolid())
        {
            node->debugMesh.set_material_override(debugSolidMaterial);
        }
        else {
            node->debugMesh.set_material_override(debugEmptyMaterial);
        }

        add_child(&node->debugMesh);
        mesh_pool.push_back(&node->debugMesh);
    }

    // Recursively traverse child nodes
    // 递归遍历子节点
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            init_debugMesh(node->children[i], depth + 1);
        }
    }
}

/**
 Breadth-first traversal init neighbors
 广度优先遍历初始化neighbors
 This method needs to be called manually if changes are involved in the svo structure.
 牵涉到svo结构变化需要手动调用此方法
 */
void SvoNavmesh::init_neighbors()
{
    Vector<OctreeNode*> queue;
    queue.push_back(svo->root);

    while (!queue.is_empty()) {
        OctreeNode* current = queue[0];
        queue.remove_at(0);

        for (int i = 0; i < 8; ++i) {
            if (current->children[i]) {
                queue.push_back(current->children[i]);
            }
        }

        set_neighbors(current);
    }
}
void SvoNavmesh::set_neighbors(OctreeNode* node)
{
    if (node->father == nullptr) return; // 排除根节点; Exclude root nodes

    // Set the neighbor relationship from the brother node
    // 兄弟节点间有直接的neighbor关系
    set_neighbors_from_brother(node);

    // Need to get neighbors from the father node or further nodes
    // 需要从父节点或更远节点获取邻居
    for (int i = 0; i < 3; i++) {
        int neighborIndex = neighborsMap[7-node->currentIndex][i];  // Reverse index to get correct external neighbor direction; 反向索引以获取正确的外部邻居方向
        int childIndex = childrenMap[node->currentIndex][i];        // Get the child node index, the magic is that there is no need to rebuild the index; 获取子节点索引，神奇的是不需要重建索引
        OctreeNode* externalNeighbor = node->father->neighbors[neighborIndex];
        // 尝试从外部邻居的子节点获取邻居; Try to get neighbors from the children of external neighbors
        if (externalNeighbor != nullptr) {
            if (!externalNeighbor->isLeaf && externalNeighbor->children[childIndex] != nullptr) {
                node->neighbors[neighborIndex] = externalNeighbor->children[childIndex];
            }
            else {
                // If the corresponding child node of the external neighbor does not exist, 
                // use the external neighbor as the neighbor
                // 如果外部邻居是叶节点或相应子节点不存在，则使用外部邻居的父节点作为邻居
                node->neighbors[neighborIndex] = externalNeighbor;
            }
        }
    }
}
void SvoNavmesh::set_neighbors_from_brother(OctreeNode* node) {
    // Set the neighbor of the current node from the children array of the father node
    // 从父节点的children数组中设置当前节点的邻居
    for (int i = 0; i < 3; i++) {
        int neighborIndex = neighborsMap[node->currentIndex][i];
        int childIndex = childrenMap[node->currentIndex][i];
        node->neighbors[neighborIndex] = node->father->children[childIndex];
    }
}

/**
 Draw the svo for debuging.
 */
void SvoNavmesh::draw_svo_v2(OctreeNode* node, int current_depth, int min_depth, int max_depth) {
    if (!node || current_depth > svo->maxDepth) return;

    float size = 2 * voxelSize / pow(2, current_depth);

    Vector3 euler_angles_radians(
        Math::deg_to_rad(offset_rotation.x),
        Math::deg_to_rad(offset_rotation.y),
        Math::deg_to_rad(offset_rotation.z)
    );
    Transform3D local_transform = Transform3D(Quaternion(euler_angles_radians), offset_position);
    Vector3 global_pos = local_transform.xform(node->center);
    Transform3D global_transform = local_transform.translated(global_pos);


    if (current_depth >= min_depth && current_depth <= max_depth) {
        node->debugMesh.set_transform(global_transform);
        node->debugMesh.set_visible(true);
    }
    else {
        node->debugMesh.set_visible(false);
    }

    // FIXME: 对于EMPTY节点，修复min_depth只在叶子层有效的问题

    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            draw_svo_v2(node->children[i], current_depth + 1, min_depth, max_depth);
        }
    }
}

//已弃用; Abandon
void SvoNavmesh::draw_svo_v1(OctreeNode* node, int current_depth, int min_depth, int max_depth) {
    if (!node || current_depth > max_depth) return;  // 如果当前深度超过最大深度，停止递归

    float size = 2*voxelSize / pow(2, current_depth);   // 计算当前深度的体素尺寸

    // 应用 offset_position 和 offset_rotation
    Vector3 euler_angles_radians(
        Math::deg_to_rad(offset_rotation.x),
        Math::deg_to_rad(offset_rotation.y),
        Math::deg_to_rad(offset_rotation.z)
    );
    Transform3D local_transform = Transform3D(Quaternion(euler_angles_radians), offset_position);
    Vector3 global_pos = local_transform.xform(node->center);
    Transform3D global_transform = local_transform.translated(global_pos);


    // 检查是否在指定深度范围内
    if (current_depth >= min_depth && current_depth <= max_depth) {
        // 创建 MeshInstance3D
            //小开销版
            //MeshInstance3D* mesh_instance = get_mesh_instance_from_pool();
            //active_meshes.push_back(mesh_instance);  // 标记为活跃，以便后续回收
            //大开销版
            MeshInstance3D* mesh_instance = memnew(MeshInstance3D);
        // 创建 BoxMesh
        Ref<BoxMesh> box_mesh = Ref<BoxMesh>(memnew(BoxMesh));
        box_mesh->set_size(Vector3(size, size, size));  // 设置盒子大小

        mesh_instance->set_mesh(box_mesh);
        mesh_instance->set_transform(global_transform);  // 设置体素的全局变换，包括位置和旋转

        // 设置材料
        if (node->voxel && node->voxel->isSolid())
        {
            Ref<StandardMaterial3D> material = Ref<StandardMaterial3D>(memnew(StandardMaterial3D));
            material->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
            material->set_albedo(Color(0.2, 1.0, 0.2, 0.2));  // 半透明绿色
            mesh_instance->set_material_override(material);
        }
        else {
            Ref<ShaderMaterial> wireframe_material = Ref<ShaderMaterial>(memnew(ShaderMaterial));
            Ref<Shader> shader = Ref<Shader>(memnew(Shader));
            shader->set_code(R"(
            shader_type spatial;
            render_mode wireframe,cull_disabled;

            void fragment() {
                ALBEDO = vec3(1.0, 0.0, 0.0);
            }
            )");

            wireframe_material->set_shader(shader);
            mesh_instance->set_material_override(wireframe_material);
        }

        //小开销版
        //if (!mesh_instance->get_parent()) add_child(mesh_instance);
        //mesh_instance->set_visible(true);

        //大开销版
        add_child(mesh_instance);
        mesh_pool.push_back(mesh_instance);

    }

    if (!node->voxel->isLiquid()) {
        return;
    }

    // 递归遍历子节点
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            draw_svo_v1(node->children[i], current_depth + 1, min_depth, max_depth); // 递归绘制子节点
        }
    }
}
//小开销版所需方法依赖，仅用于draw_svo_v1
/*MeshInstance3D* SvoNavmesh::get_mesh_instance_from_pool() {
    if (mesh_pool.size() > 0) {
        MeshInstance3D* instance = mesh_pool[mesh_pool.size() - 1]; // 获取最后一个元素
        mesh_pool.remove_at(mesh_pool.size() - 1); // 移除最后一个元素
        return instance;
    }
    return memnew(MeshInstance3D); // 如果池中没有可用对象，创建新的实例
}
void SvoNavmesh::recycle_mesh_instance(MeshInstance3D* instance) {
    instance->set_visible(false);  // 隐藏实例，避免渲染
    mesh_pool.push_back(instance);  // 将实例回收到池中
}
void SvoNavmesh::clear_mesh_instances() {
    // 清空活跃列表，并回收所有实例
    for (int i = 0; i < active_meshes.size(); ++i) {
        recycle_mesh_instance(active_meshes[i]);
    }
    active_meshes.clear();
}*/