#include "svo_navmesh.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/godot.hpp>
#define global_min_depth 1
#define global_max_depth 9

using namespace godot;

// print example
// WARN_PRINT("");
// WARN_PRINT_ONCE(vformat(""));
// WARN_PRINT_ED(vformat(""));
// UtilityFunctions::print("");
// UtilityFunctions::print(vformat(""));

void OctreeNode::_bind_methods()
{

}
void SvoNavmesh::_bind_methods() {
    ClassDB::bind_method(D_METHOD("insert_voxel", "position"), &SvoNavmesh::insert_voxel);
    ClassDB::bind_method(D_METHOD("query_voxel", "position"), &SvoNavmesh::query_voxel);
    ClassDB::bind_method(D_METHOD("check_voxel_with_id", "id"), &SvoNavmesh::check_voxel_with_id);
    ClassDB::bind_method(D_METHOD("update_voxel", "position", "isSolid"), &SvoNavmesh::update_voxel);
    ClassDB::bind_method(D_METHOD("get_info"), &SvoNavmesh::get_info);
    ClassDB::bind_method(D_METHOD("set_info", "p_info"), &SvoNavmesh::set_info);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "testdouble"), "set_info", "get_info");
    ClassDB::bind_method(D_METHOD("get_debug_mode"), &SvoNavmesh::get_debug_mode);
    ClassDB::bind_method(D_METHOD("set_debug_mode", "debug_mode"), &SvoNavmesh::set_debug_mode);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::BOOL, "debug_mode"), "set_debug_mode", "get_debug_mode");
    ClassDB::bind_method(D_METHOD("get_collision_layer"), &SvoNavmesh::get_collision_layer);
    ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &SvoNavmesh::set_collision_layer);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::INT, "collision_layer"), "set_collision_layer", "get_collision_layer");
    
    ClassDB::bind_method(D_METHOD("get_voxel_size"), &SvoNavmesh::get_voxel_size);
    ClassDB::bind_method(D_METHOD("set_voxel_size", "size"), &SvoNavmesh::set_voxel_size);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "rootVoxelSize"), "set_voxel_size", "get_voxel_size");

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
    ClassDB::bind_method(D_METHOD("get_show_empty"), &SvoNavmesh::get_show_empty);
    ClassDB::bind_method(D_METHOD("set_show_empty", "show_empty"), &SvoNavmesh::set_show_empty);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::BOOL, "show_empty"), "set_show_empty", "get_show_empty");
    ClassDB::bind_method(D_METHOD("get_debug_path_scale"), &SvoNavmesh::get_debug_path_scale);
    ClassDB::bind_method(D_METHOD("set_debug_path_scale", "scale"), &SvoNavmesh::set_debug_path_scale);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "debug_path_scale"), "set_debug_path_scale", "get_debug_path_scale");

    // button method
    ClassDB::bind_method(D_METHOD("rebuild_svo"), &SvoNavmesh::rebuild_svo);
    ClassDB::bind_method(D_METHOD("refresh_svo"), &SvoNavmesh::refresh_svo);
    ClassDB::bind_method(D_METHOD("clear_svo"), &SvoNavmesh::clear_svo);

    // generate svo from collider
    ClassDB::bind_method(D_METHOD("insert_svo_based_on_collision_shapes"), &SvoNavmesh::insert_svo_based_on_collision_shapes);

    // path finding
    ClassDB::bind_method(D_METHOD("find_path", "start", "end", "agent_r", "is_smooth"), &SvoNavmesh::find_path_v1);
    // path finding multi thread
    ClassDB::bind_method(D_METHOD("find_path_v2", "start", "end", "agent_r", "is_smooth"), &SvoNavmesh::find_path_v2);
    ClassDB::bind_method(D_METHOD("find_path_multi_thread", "start", "end", "agent_r", "is_smooth"), &SvoNavmesh::find_path_multi_thread);
    ClassDB::bind_method(D_METHOD("get_last_path_result"), &SvoNavmesh::get_last_path_result);
}

SvoNavmesh::SvoNavmesh(): 
    maxDepth(3), rootVoxelSize(1.0f), minVoxelSize(0.25f), testdouble(1.14f), debug_mode(false), collision_layer(5), 
    debug_path_scale(1.0f), node_ready(false), DrawRef_minDepth(1), DrawRef_maxDepth(3), show_empty(true), debugChecked_node(nullptr)
{
    svo = memnew(SparseVoxelOctree);
}
SvoNavmesh::~SvoNavmesh() {
    //call_deferred("reset_pool");
    if(svo) memdelete(svo);
    svo = nullptr;
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
// set_neighbors_from_brother时，每个子节点对应父节点的children数组中的索引
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
// During get_neighbors, each direction corresponds to the index in the children array of the neighbor.
// get_neighbors时，每个方向对应邻居的children数组中的索引
static const int childDirectionMap[6][4] = {
    {0, 2, 4, 6}, // +X 方向：西面，需要检查X=0的所有子节点
    {1, 3, 5, 7}, // -X 方向：东面，需要检查X=1的所有子节点
    {0, 1, 4, 5}, // +Y 方向：下面，需要检查Y=0的所有子节点
    {2, 3, 6, 7}, // -Y 方向：上面，需要检查Y=1的所有子节点
    {0, 1, 2, 3}, // +Z 方向：南面，需要检查Z=0的所有子节点
    {4, 5, 6, 7}, // -Z 方向：北面，需要检查Z=1的所有子节点
};

/**
 * Converts a world position to a local grid position.
 *
 * @param world_position: The position in world coordinates.
 * @returns The position in local coordinates of svo.
 */
Vector3 SvoNavmesh::worldToGrid(Vector3 world_position) {
    Transform3D global_transform = get_global_transform();

    // Remove scaling: Normalize the columns of the rotation matrix
    // 移除缩放：将旋转矩阵的列向量归一化
    // In fact, the scale of SVO in the engine should not be changed
    // 事实上引擎内SVO的scale是不因该去改变的
    Vector3 x = global_transform.basis.get_column(0).normalized();
    Vector3 y = global_transform.basis.get_column(1).normalized();
    Vector3 z = global_transform.basis.get_column(2).normalized();
    global_transform.basis.set_column(0, x);
    global_transform.basis.set_column(1, y);
    global_transform.basis.set_column(2, z);

    Vector3 local_position = global_transform.xform_inv(world_position); // Global to local; 全局到局部
    return local_position; // Convert to grid coordinates; 转换到网格坐标
}
Vector3 SvoNavmesh::gridToWorld(Vector3 grid_position) {
    Transform3D global_transform = get_global_transform();

    // Remove scaling: Normalize the columns of the rotation matrix
    // 移除缩放：将旋转矩阵的列向量归一化
    // In fact, the scale of SVO in the engine should not be changed
    // 事实上引擎内SVO的scale是不因该去改变的
    Vector3 x = global_transform.basis.get_column(0).normalized();
    Vector3 y = global_transform.basis.get_column(1).normalized();
    Vector3 z = global_transform.basis.get_column(2).normalized();
    global_transform.basis.set_column(0, x);
    global_transform.basis.set_column(1, y);
    global_transform.basis.set_column(2, z);

    return global_transform.xform(grid_position);
}
bool globalDepthCheck(int depth) {
    if (depth< global_min_depth || depth>global_max_depth) return false;
    else return true;
}
uint32_t create_collision_mask(const Vector<int>& layers) {
    uint32_t mask = 0;
    for (int layer : layers) {
        if (layer < 1 || layer > 32) {
            UtilityFunctions::print("Layer number must be between 1 and 32 inclusive.");
            continue;
        }
        mask |= 1 << (layer - 1);
    }
    return mask;
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
    if (grid_position.x < -rootVoxelSize / 2 || grid_position.y < -rootVoxelSize / 2 || grid_position.z < -rootVoxelSize / 2 ||
        grid_position.x >= rootVoxelSize / 2 || grid_position.y >= rootVoxelSize / 2 || grid_position.z >= rootVoxelSize / 2) {
        UtilityFunctions::print("insert: Position out of valid range.");
        return;
    }

    refresh_svo();
    //reset_pool();
    if (debug_mode) reset_pool_v3();
    svo->insert(grid_position);
    //init_debug_mesh(svo->root, 1);
    if (debug_mode) init_debug_mesh_v3();
    init_neighbors();
}
/**
 * Query a voxel from the specified world position.
 *
 * @param world_position: The world position where the voxel is to be queried.
 */
bool SvoNavmesh::query_voxel(Vector3 world_position) {
    Vector3 grid_position = worldToGrid(world_position);

    // Check if the grid coordinates are valid (i.e. within the root node range)
    // 检查网格坐标是否有效(即在根节点范围内)
    if (grid_position.x < -rootVoxelSize / 2 || grid_position.y < -rootVoxelSize / 2 || grid_position.z < -rootVoxelSize / 2 ||
        grid_position.x >= rootVoxelSize / 2 || grid_position.y >= rootVoxelSize / 2 || grid_position.z >= rootVoxelSize / 2) {
        UtilityFunctions::print("query: Position out of valid range.");
        return false;
    }

    return svo->query(grid_position);
}
/**
 * Checkout a voxel from the specified id.
 *
 * @param id: The id of the voxel in an SVO. Note that 2 voxel in 2 SVO could have same id.
 */
void SvoNavmesh::check_voxel_with_id(String id) {
    // 检查路径ID是否只有一个字符且为'0'
    if (id.length() == 1 && id[0] == '0') {
        UtilityFunctions::print("Checking root node voxel information.");
        UtilityFunctions::print(svo->root->get_voxel_info());
        return;
    }
    else if (id.length() == 0) {
        return;
    }
    
    OctreeNode* current_node = svo->root;
    for (int i = 1; i < id.length(); i++) {
        char child_index = id[i];

        // 检查字符是否有效
        if (child_index < '0' || child_index > '7') {
            UtilityFunctions::print("Invalid ID: ", id);
            return;
        }

        // 转换字符为子节点索引
        int index = child_index - '0';

        // 检查子节点是否存在
        if (!current_node->children[index]) {
            UtilityFunctions::print("Node does not exist at path: ", id.substr(0, i + 1));
            return;
        }

        // 移动到子节点
        current_node = current_node->children[index];
    }
    // 打印找到的体素信息
    if (current_node && current_node->voxel) {
        UtilityFunctions::print("Voxel found at path: ", id);
        UtilityFunctions::print(current_node->get_voxel_info());
        reset_debugCheck();
        current_node->debugMesh->set_material_override(debugCheckMaterial);
        current_node->debugChecked = true;
        debugChecked_node = current_node;
    }
    else {
        UtilityFunctions::print("No voxel found at path: ", id);
    }
}
//临时弃用; Temp Abandon
void SvoNavmesh::update_voxel(Vector3 world_position, bool isSolid) {
    Vector3 grid_position = worldToGrid(world_position);

    WARN_PRINT("Update is not ready for now.");
    return;

    // Check if the grid coordinates are valid (i.e. within the root node range)
    // 检查网格坐标是否有效(即在根节点范围内)
    if (grid_position.x < -rootVoxelSize / 2 || grid_position.y < -rootVoxelSize / 2 || grid_position.z < -rootVoxelSize / 2 ||
        grid_position.x >= rootVoxelSize / 2 || grid_position.y >= rootVoxelSize / 2 || grid_position.z >= rootVoxelSize / 2) {
        UtilityFunctions::print("update: Position out of valid range.");
        return;
    }

    refresh_svo();
    svo->update(grid_position, isSolid);
}

float SvoNavmesh::get_voxel_size() const {
    return rootVoxelSize;
}
void SvoNavmesh::set_voxel_size(float size) {
    if (rootVoxelSize != size) {
        rootVoxelSize = size;
        minVoxelSize = svo->calActualVoxelSize(maxDepth);
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
        minVoxelSize = svo->calActualVoxelSize(maxDepth);
    }
}

int SvoNavmesh::get_collision_layer() const {
    return collision_layer;
}
void SvoNavmesh::set_collision_layer(int layer) {
    collision_layer = layer;
}

// <debug draw set/get>
bool SvoNavmesh::get_debug_mode() const {
    return debug_mode;
}
void SvoNavmesh::set_debug_mode(bool debug_mode) {
    this->debug_mode = debug_mode;
    // TODO: when ready for demo game,
    //       this needs to be removed,
    //       debug_mode should only control rendering in game.
    if (node_ready) {
        if (debug_mode) {
            //reset_pool();
            //init_debug_mesh(svo->root, 1);
            init_debug_mesh_v3();
        }
        else {
            //force_clear_debug_mesh();
            reset_pool_v3();
        }
    }
}
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
bool SvoNavmesh::get_show_empty() const {
    return show_empty;
}
void SvoNavmesh::set_show_empty(bool show_empty) {
    this->show_empty = show_empty;
}
float SvoNavmesh::get_debug_path_scale() const {
    return debug_path_scale;
}
void SvoNavmesh::set_debug_path_scale(float scale)
{
    if (scale<0 || scale>1) {
        WARN_PRINT("Scale out of setting range.(0-1)");
        return;
    }
    debug_path_scale = scale;
}
// </debug draw set/get>

Array SvoNavmesh::get_last_path_result() {
    return path_result;
}

/**
 Clear the svo and total rebuild base on collision shapes.
 */
void SvoNavmesh::rebuild_svo() {
    uint64_t begin = Time::get_singleton()->get_ticks_msec();

    svo->maxDepth = maxDepth;
    svo->voxelSize = rootVoxelSize;

    if (debug_mode) {
        //reset_pool();
        reset_pool_v3();
        svo->clear();

        insert_svo_based_on_collision_shapes();
        //init_debug_mesh(svo->root, 1);
        init_debug_mesh_v3();
        init_neighbors();
    }
    else {
        svo->clear();
        insert_svo_based_on_collision_shapes();
        init_neighbors();
    }

    uint64_t end = Time::get_singleton()->get_ticks_msec();
    UtilityFunctions::print(vformat("init neighbors with %d milliseconds", end - begin));
}

/**
 Refresh the svo base on svo setting changes.
 */
void SvoNavmesh::refresh_svo() {
    // FIXME: expand_node() and compress_node() are doing
    // dynamic changes to svo nodes, which needs to call init_debug_mesh
    // to have proper rendering. And right now it's gonna face 
    // memory access voilation. Abandon this temp since 
    // rebuild_svo is the only needed right now.

    //if (debug_mode) reset_pool();
    if (debug_mode) reset_pool_v3();
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
    if (svo->voxelSize != rootVoxelSize) {
        svo->voxelSize = rootVoxelSize;
        // refresh center
        svo->update_node_centers();
    }
    //if (debug_mode) init_debug_mesh(svo->root, 1);
    if (debug_mode) init_debug_mesh_v3();
    init_neighbors();
}

/**
 * Clear the svo.
 *
 * @param clear_setting: whether also clear svo setting.
 */
void SvoNavmesh::clear_svo(bool clear_setting) {
    //if (debug_mode) reset_pool();
    if (debug_mode) reset_pool_v3();
    if (clear_setting) {
        // clear svo and settings
        maxDepth = 3;
        rootVoxelSize = 1.0f;
        svo->clear();
    }
    else {
        // clear only svo
        svo->clear();
    }
    //if (debug_mode) init_debug_mesh(svo->root, 1);
    if (debug_mode) init_debug_mesh_v3();
    init_neighbors();
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
    traverse_svo_space_and_insert(svo->root, 1, space_rid);

    uint64_t end = Time::get_singleton()->get_ticks_msec();
    UtilityFunctions::print(vformat("insert svo nodes with %d milliseconds", end - begin));
}

/**
 Use Godot's physics engine to check if the point is inside CollisionShape3D.
 */
bool SvoNavmesh::check_point_inside_mesh(Vector3 point, RID& space_rid) {
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
    uint32_t collision_mask = 0;
    collision_mask |= 1 << (collision_layer - 1);
    query_params->set_collision_mask(collision_mask);   // check svo layer

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
            if (target_rids.has(result_rid)) {
                return true;
            }
        }
    }
    return false;
}

/**
 Use Godot's physics engine to check if the cube intersect CollisionShape3D.
 */
bool SvoNavmesh::check_box_intersect_mesh(Vector3 position, Quaternion rotation, float size, RID& space_rid) {
    // Get the PhysicsDirectSpaceState3D instance
    PhysicsDirectSpaceState3D* space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(space_rid);
    if (!space_state) {
        ERR_PRINT_ED("Failed to get PhysicsDirectSpaceState3D instance");
        return false;
    }

    // Create a BoxShape3D with the given size
    Ref<BoxShape3D> box_shape;
    box_shape.instantiate();
    box_shape->set_size(Vector3(size, size, size));

    // Create and configure query parameters
    Ref<PhysicsShapeQueryParameters3D> query_params;
    query_params.instantiate();
    query_params->set_shape(box_shape);
    query_params->set_transform(Transform3D(rotation, position));
    uint32_t collision_mask = 0;
    collision_mask |= 1 << (collision_layer - 1);
    query_params->set_collision_mask(collision_mask);   // check svo layer

    // Execute query
    Array results = space_state->intersect_shape(query_params);

    // Process the query results
    if (!results.is_empty()) {
        for (int i = 0; i < results.size(); ++i) {
            Dictionary result = results[i];
            RID result_rid = result["rid"];
            if (target_rids.has(result_rid)) {
                return true;
            }
        }
    }
    return false;
}
bool SvoNavmesh::is_box_fully_inside_mesh(Vector3 position, float size, RID& space_rid) {
    Vector3 half_size = Vector3(size, size, size) * 0.5;

    // Sample points: 8 vertices + 6 face centers + 12 edge midpoints + 1 center = 27 points
    Vector3 sample_points[27] = {
        position + Vector3(-half_size.x, -half_size.y, -half_size.z),
        position + Vector3(half_size.x, -half_size.y, -half_size.z),
        position + Vector3(-half_size.x, half_size.y, -half_size.z),
        position + Vector3(half_size.x, half_size.y, -half_size.z),
        position + Vector3(-half_size.x, -half_size.y, half_size.z),
        position + Vector3(half_size.x, -half_size.y, half_size.z),
        position + Vector3(-half_size.x, half_size.y, half_size.z),
        position + Vector3(half_size.x, half_size.y, half_size.z),

        position + Vector3(0, 0, 0),

        position + Vector3(-half_size.x, 0, 0),
        position + Vector3(half_size.x, 0, 0),
        position + Vector3(0, -half_size.y, 0),
        position + Vector3(0, half_size.y, 0),
        position + Vector3(0, 0, -half_size.z),
        position + Vector3(0, 0, half_size.z),

        position + Vector3(-half_size.x, -half_size.y, 0),
        position + Vector3(half_size.x, -half_size.y, 0),
        position + Vector3(-half_size.x, half_size.y, 0),
        position + Vector3(half_size.x, half_size.y, 0),
        position + Vector3(-half_size.x, 0, -half_size.z),
        position + Vector3(half_size.x, 0, -half_size.z),
        position + Vector3(-half_size.x, 0, half_size.z),
        position + Vector3(half_size.x, 0, half_size.z),
        position + Vector3(0, -half_size.y, -half_size.z),
        position + Vector3(0, half_size.y, -half_size.z),
        position + Vector3(0, -half_size.y, half_size.z),
        position + Vector3(0, half_size.y, half_size.z)
    };

    for (int i = 0; i < 27; i++) {
        if (!check_point_inside_mesh(sample_points[i], space_rid)) {
            return false;
        }
    }

    return true;
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
                    target_rids.push_back(target_rid);
                    //Vector<Vector3> points = get_shape_points(collision_shape, get_global_position(), voxelSize, svo->calActualVoxelSize(svo->maxDepth) / 2, space_rid, target_rid);
                    //build_svo(svo, points);
                }
            }
        }

        // Recursively collect collision shapes of child nodes
        // 递归地收集子节点的碰撞形状
        collect_collision_shapes(child, space_rid);
    }
}

/**
 Intersect svo nodes with collider recursively
 */
void SvoNavmesh::traverse_svo_space_and_insert(OctreeNode* node, int depth, RID& space_rid) {
    if (depth > maxDepth) {
        return;
    }

    // TODO: Start working on this when ready for real Sparse
    bool temp = false;
    if (temp && is_box_fully_inside_mesh(gridToWorld(node->center), node->voxel->size, space_rid)) {
        if (depth == maxDepth) {
            node->voxel->state = VS_SOLID;
            node->isLeaf = true;
        }
        else {
            node->voxel->state = VS_LIQUID;
            if (node->children[0] == nullptr) {
                svo->create_empty_children(node, depth);
            }

            for (int i = 0; i < 8; ++i) {
                traverse_svo_space_and_insert(node->children[i], depth + 1, space_rid);
            }
            svo->evaluate_homogeneity(node);
        }
        return;
    }

    if (check_box_intersect_mesh(gridToWorld(node->center), get_global_rotation(), node->voxel->size, space_rid)) {
        if (depth == maxDepth) {
            node->voxel->state = VS_SOLID;
            node->isLeaf = true;
        }
        else {
            node->voxel->state = VS_LIQUID;
            if (node->children[0] == nullptr) {
                svo->create_empty_children(node, depth);
            }

            for (int i = 0; i < 8; ++i) {
                traverse_svo_space_and_insert(node->children[i], depth + 1, space_rid);
            }
            svo->evaluate_homogeneity(node);
        }
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
void SvoNavmesh::_ready() {
    if (debug_mode)
    {
        //reset_pool();
        //init_debug_mesh(svo->root, 1);
        reset_pool_v3();
        init_debug_mesh_v3();
    }
    init_neighbors();
    node_ready = true;

    refresh_svo();
}
void SvoNavmesh::_process(double delta) {
    if (debug_mode) draw_svo_v3(svo->root, 1, DrawRef_minDepth, DrawRef_maxDepth);
}
void SvoNavmesh::_physics_process(double delta)
{

}
static void init_static_material()
{
    // debug draw
    if (debugSolidMaterial.is_null()) debugSolidMaterial.instantiate();
    debugSolidMaterial->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
    debugSolidMaterial->set_albedo(Color(0.2, 1.0, 0.2, 0.2));  // 半透明绿色

    if (debugCheckMaterial.is_null()) debugCheckMaterial.instantiate();
    debugCheckMaterial->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
    debugCheckMaterial->set_albedo(Color(0.2, 0.2, 1.0, 0.2));  // 半透明蓝紫色

    if (debugPathMaterialB.is_null()) debugPathMaterialB.instantiate();
    debugPathMaterialB->set_albedo(Color(0.2, 0.2, 0.7));  // 半透明蓝色

    if (debugPathMaterialY.is_null()) debugPathMaterialY.instantiate();
    debugPathMaterialY->set_albedo(Color(0.9, 0.6, 0.1));  // 半透明黄色

    if (debugPathMaterialR.is_null()) debugPathMaterialR.instantiate();
    debugPathMaterialR->set_albedo(Color(0.9, 0.2, 0.2));  // 半透明红色

    if (debugEmptyMaterial.is_null()) debugEmptyMaterial.instantiate();
    if (EmptyMaterial_shader.is_null()) {
        EmptyMaterial_shader.instantiate();
        EmptyMaterial_shader->set_code(R"(
            shader_type spatial;
            render_mode wireframe, cull_disabled;

            void fragment() {
                ALBEDO = vec3(1.0, 0.0, 0.0);
            }
        )");
    }
    debugEmptyMaterial->set_shader(EmptyMaterial_shader);
}
static void clear_static_material() {
    // static Mat is not freed after level switch
    // TODO: change this when ready for game
    debugSolidMaterial.unref();
    debugCheckMaterial.unref();
    debugPathMaterialB.unref();
    debugPathMaterialY.unref();
    debugPathMaterialR.unref();
    debugEmptyMaterial.unref();
    EmptyMaterial_shader.unref();
}
void SvoNavmesh::_enter_tree()
{
    init_static_material();
}
void SvoNavmesh::_exit_tree(){
    clear_static_material();
}

/**
 This method needs to be called manually before changes are involved in the svo structure.
 牵涉到svo结构变化前需要手动调用此方法
 Debug rendering only
 */
void SvoNavmesh::reset_pool_v3() {
    if (!exist_meshes.is_empty()) {
        for (int i = 0; i < exist_meshes.size(); ++i) {
            OctreeNode* instance_node = exist_meshes[i];
            if (instance_node) {
                remove_child(instance_node->debugMesh);
                instance_node->freeDebugMesh();
            }
        }
        exist_meshes.clear();
    }
    if (!path_pool.is_empty()) {
        // Clear existing path_pool if necessary
        for (int i = 0; i < path_pool.size(); ++i) {
            remove_child(path_pool[i]);
            memdelete(path_pool[i]);
        }
        path_pool.clear();
    }
    target_rids.clear();
    reset_debugCheck();
}

void SvoNavmesh::reset_debugCheck() {
    if (debugChecked_node) {
        if (debugChecked_node->voxel && debugChecked_node->voxel->isSolid())
        {
            debugChecked_node->debugMesh->set_material_override(debugSolidMaterial);
        }
        else {
            debugChecked_node->debugMesh->set_material_override(debugEmptyMaterial);
        }
        debugChecked_node->debugChecked = false;
        debugChecked_node = nullptr;
    }
}

/**
 This method needs to be called manually after changes are involved in the svo structure.
 牵涉到svo结构变化后需要手动调用此方法
 Debug rendering only
 */
void SvoNavmesh::init_debug_mesh_v3() {
    init_debug_mesh_v3(svo->root, 1);
    if (!exist_meshes.is_empty()) {
        for (int i = 0; i < exist_meshes.size(); ++i) {
            OctreeNode* instance_node = exist_meshes[i];
            if (instance_node && !active_meshes.has(instance_node)) {
                remove_child(instance_node->debugMesh);
                instance_node->freeDebugMesh();
            }
        }
        exist_meshes.clear();
    }
    if (!active_meshes.is_empty()) {
        for (OctreeNode* instance_node : active_meshes) {
            exist_meshes.push_back(instance_node);
        }
        active_meshes.clear();
    }
}
void SvoNavmesh::init_debug_mesh_v3(OctreeNode* node, int depth)
{
    if (!node || depth > svo->maxDepth) return;

    float size = 2 * rootVoxelSize / pow(2, depth);

    if (depth <= svo->maxDepth) {
        if (!node->isDebugMeshValid()) {
            node->debugMesh = get_mesh_instance_from_pool();
        }
        if (node->debugBoxMesh.is_null()) {
            node->debugBoxMesh = Ref<BoxMesh>(memnew(BoxMesh));
        }
        node->debugBoxMesh->set_size(Vector3(size, size, size));
        node->debugMesh->set_mesh(node->debugBoxMesh);

        // Set up materials
        // 设置材料
        if (node->voxel && node->voxel->isSolid())
        {
            node->debugMesh->set_material_override(debugSolidMaterial);
        }
        else {
            node->debugMesh->set_material_override(debugEmptyMaterial);
        }

        if(node->debugMesh->get_parent() != this) add_child(node->debugMesh);
        active_meshes.push_back(node);
    }

    // Recursively traverse child nodes
    // 递归遍历子节点
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            init_debug_mesh_v3(node->children[i], depth + 1);
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
void SvoNavmesh::draw_svo_v3(OctreeNode* node, int current_depth, int min_depth, int max_depth) {
    if (!node->isDebugMeshValid()) {
        WARN_PRINT_ONCE(vformat("debugMesh is null"));
        return;
    }
    if (!node || current_depth > svo->maxDepth) return;

    bool is_painting = node->voxel->isSolid() || (show_empty && node->voxel->isEmpty());

    if (is_painting || node->debugChecked) {
        float size = 2 * rootVoxelSize / pow(2, current_depth);

        if (node->debugChecked || (current_depth >= min_depth && current_depth <= max_depth)) {
            node->debugMesh->set_transform(Transform3D(Basis(), node->center));
            node->debugMesh->set_visible(true);
        }
        else {
            node->debugMesh->set_visible(false);
        }

    }
    else {
        node->debugMesh->set_visible(false);
    }

    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            draw_svo_v3(node->children[i], current_depth + 1, min_depth, max_depth);
        }
    }
}

/**
 debugMesh dynamic support.
 */
MeshInstance3D* SvoNavmesh::get_mesh_instance_from_pool() {
    if (exist_meshes.size() > 0) {
        MeshInstance3D* instance = exist_meshes[exist_meshes.size() - 1]->debugMesh; // 获取最后一个元素
        exist_meshes.remove_at(exist_meshes.size() - 1); // 移除最后一个元素
        return instance;
    }
    return memnew(MeshInstance3D); // 如果池中没有可用对象，创建新的实例
}

/**
 Helper function to calculate the heuristic based on Euclidean distance
 */
float heuristic(Vector3 a, Vector3 b) {
    return a.distance_to(b);  // Euclidean distance
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);  // Manhattan distance
}
float heuristic(OctreeNode* a, OctreeNode* b) {
    return heuristic(a->center, b->center);
}

/**
 reconstruct path from came_from map
 */
Vector<Vector3> reconstruct_path(const Dictionary& came_from, const Vector3& end) {
    Vector<Vector3> path;
    Vector3 current = end;

    path.insert(0, current); // Start with the end position
    while (came_from.has(current)) {
        current = came_from[current]; // Retrieve the next step in the path
        path.insert(0, current);
    }

    return path; // Return the reconstructed path from start to end
}

/**
 find the node with the lowest f-score
 */
OctreeNode* get_lowest_f_score_node(const Vector<OctreeNode*>& open_set, const Dictionary& f_score) {
    OctreeNode* lowest = open_set[0];
    float lowest_score = f_score[lowest->get_instance_id()];
    for (int i = 1; i < open_set.size(); i++) {
        OctreeNode* temp = open_set[i];
        float score = f_score[temp->get_instance_id()];
        if (score < lowest_score) {
            lowest = temp;
            lowest_score = score;
        }
    }
    return lowest;
}

/**
 get liquid neighbors' navigable children
 */
void add_liquid_children(Vector<OctreeNode*>& neighbors, OctreeNode* node, float agent_r, int direction) {
    if (node->voxel->size/2 < agent_r) return; // 如果节点太小，不适合代理通过，停止递归

    for (int j = 0; j < 4; ++j) {
        OctreeNode* sub_neighbor = node->children[childDirectionMap[direction][j]];
        if (!sub_neighbor) continue;
        if (sub_neighbor->voxel->isLiquid()) {
            add_liquid_children(neighbors, sub_neighbor, agent_r, direction); // 递归调用以检查更小的子节点
        }
        else if (sub_neighbor->voxel->isEmpty()) {
            neighbors.push_back(sub_neighbor); // 空节点直接加入
        }
    }
}

/**
 get navigable neighbors, considering SVO layers
 */
Vector<OctreeNode*> get_neighbors(OctreeNode* node, float agent_r) {
    Vector<OctreeNode*> neighbors;

    if (!node) return neighbors;

    for (int i = 0; i < 6; ++i) {
        OctreeNode* neighbor = node->neighbors[i];
        if (!neighbor) continue;

        if (neighbor->voxel->isLiquid()) {
            // For a Liquid node, recursively check its child nodes
            // 对于液态节点，递归检查其子节点
            add_liquid_children(neighbors, neighbor, agent_r, i);
        }
        else if (neighbor->voxel->isEmpty()) {
            if (neighbor->voxel->size < agent_r) continue; // 如果节点太小，不适合代理通过
            neighbors.push_back(neighbor); // 空节点直接加入
        }
    }
    return neighbors;
}

/**
 check if a direct path (with radius) between two points is feasible
 */
bool SvoNavmesh::can_travel_directly_with_cylinder(const Vector3& from, const Vector3& to, float agent_radius, RID& space_rid) {
    // Get the PhysicsDirectSpaceState3D instance
    // 获取 PhysicsDirectSpaceState3D 实例
    PhysicsDirectSpaceState3D* space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(space_rid);
    if (!space_state) {
        ERR_PRINT_ED("Failed to get PhysicsDirectSpaceState3D instance");
        return false;
    }

    Vector3 from_w = gridToWorld(from);
    Vector3 to_w = gridToWorld(to);

    // Set up Cylinder
    // 设置圆柱体网格
    Ref<CylinderShape3D> cylinder_shape = memnew(CylinderShape3D);
    cylinder_shape->set_radius(agent_radius);
    cylinder_shape->set_height(from_w.distance_to(to_w));

    CollisionShape3D collision_shape;
    collision_shape.set_shape(cylinder_shape);

    // Set the transform of the query parameters
    Transform3D transform;
    Vector3 direction = (to_w - from_w).normalized();
    // 默认的向上方向
    Vector3 up(0, 1, 0);
    // 计算旋转轴
    Vector3 rotation_axis = up.cross(direction).normalized();
    // 计算旋转角度
    float angle = acos(up.dot(direction));
    if (angle != 0) {
        // 创建四元数
        Quaternion rotation(rotation_axis, angle);
        transform.basis = Basis(rotation);
    }
    Vector3 midpoint = (from_w + to_w) * 0.5;
    transform.origin = midpoint;

    // Set up the query parameters
    Ref<PhysicsShapeQueryParameters3D> query_params;
    query_params.instantiate();
    query_params->set_shape(collision_shape.get_shape());
    query_params->set_transform(transform);
    //Vector<int> layers = { 1,2,3,4 };
    //uint32_t collision_mask = create_collision_mask(layers);
    //query_params->set_collision_mask(collision_mask);   // check all
    //query_params->set_collide_with_areas(true);
    //query_params->set_collide_with_bodies(true);
    //query_params->set_margin(0.1);

    // ABANDON: 
    bool debug_smooth = false;
    if (debug_smooth) {
        // Create Materials for Debugging
        // 创建用于调试的材料
        // Should use static ones instead
        Ref<StandardMaterial3D> material = memnew(StandardMaterial3D);
        material->set_albedo(Color(0.2, 0.7, 0.2)); // 绿色
        // 设置圆柱体网格
        Ref<CylinderMesh> cylinderMesh = memnew(CylinderMesh);
        cylinderMesh->set_bottom_radius(agent_radius / 2); // 圆柱体半径
        cylinderMesh->set_top_radius(agent_radius / 2);
        cylinderMesh->set_height(from_w.distance_to(to_w));
        MeshInstance3D* cylinder = memnew(MeshInstance3D);
        cylinder->set_mesh(cylinderMesh);
        cylinder->set_material_override(material);
        Transform3D transform_d = transform;
        transform_d.basis.scale_local(Vector3(1.1, 1, 1.1)); // 圆柱体的缩放
        cylinder->set_transform(transform_d);
        add_child(cylinder);
        path_pool.push_back(cylinder);
    }

    // Execute query
    // 执行点内查询
    Array results = space_state->intersect_shape(query_params, 32);

    // Process the query results
    // 处理查询结果
    if (!results.is_empty()) {
        for (int i = 0; i < results.size(); ++i) {
            Dictionary result = results[i];
            // Compare collider object RID
            // 对比 collider 对象RID
            RID result_rid = result["rid"];
            if (target_rids.has(result_rid)) {
                return false;
            }
        }
    }
    return true;
}

/**
 check if a direct path (no radius) between two points is feasible
 */
bool SvoNavmesh::can_travel_directly_with_ray(const Vector3& from, const Vector3& to, RID& space_rid) {
    // Get the PhysicsDirectSpaceState3D instance
    // 获取 PhysicsDirectSpaceState3D 实例
    PhysicsDirectSpaceState3D* space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(space_rid);
    if (!space_state) {
        ERR_PRINT_ED("Failed to get PhysicsDirectSpaceState3D instance");
        return false;
    }

    // Set up the query parameters
    Ref<PhysicsRayQueryParameters3D> query_params;
    query_params.instantiate();
    query_params->set_from(gridToWorld(from));
    query_params->set_to(gridToWorld(to));
    //Vector<int> layers = {1,2,3,4};
    //uint32_t collision_mask = create_collision_mask(layers);
    //query_params->set_collision_mask(collision_mask);   // check all

    // Execute query
    // 执行点内查询
    Dictionary result = space_state->intersect_ray(query_params);

    // Process the query results
    // 处理查询结果
    if (!result.is_empty()) {
        RID result_rid = result["rid"];
        if (target_rids.has(result_rid)) {
            return false;
        }
    }
    return true;
}

float calculateSegmentLength(float rootVoxelSize, float minVoxelSize) {
    float phiInverse = 0.61803398875;
    float segment_length = minVoxelSize + (rootVoxelSize - minVoxelSize) * phiInverse;
    return segment_length;
}

/**
 fast smooth_path (though not lot faster)
 */
void SvoNavmesh::smooth_path_string_pulling_fast(float agent_radius, RID& space_rid) {
    if (exist_path.size() < 2) {
        return;
    }

    Vector<Vector3> smooth_path;
    smooth_path.push_back(exist_path[0]);

    int i = 0;
    while (i < exist_path.size() - 1) {
        int j = i + 1;
        while (j < exist_path.size()) {
            bool can_traverse;
            if (agent_radius > 0.0f) {
                can_traverse = can_travel_directly_with_cylinder(exist_path[i], exist_path[j], agent_radius, space_rid);
            }
            else {
                can_traverse = can_travel_directly_with_ray(exist_path[i], exist_path[j], space_rid);
            }

            if (!can_traverse) {
                break;
            }
            ++j;
        }

        if (j == i + 1) {
            // Avoid infinite loop by ensuring at least one progress
            j++;
        }

        smooth_path.push_back(exist_path[j - 1]);
        i = j - 1;
    }
    exist_path = smooth_path;
}

/**
 fast smooth_path (with subdivide path)
 */
void SvoNavmesh::smooth_path_string_pulling_fast_v2(float agent_radius, RID& space_rid) {
    if (exist_path.size() < 2) {
        return;
    }

    // First cut the entire path
    // 先对整个路径进行切割
    // 目前切割使用距离为: minVoxelSize*2
    Vector<Vector3> subdivided_path;
    float segment_length = calculateSegmentLength(rootVoxelSize, minVoxelSize);
    for (int i = 0; i < exist_path.size() - 1; ++i) {
        Vector<Vector3> segment_points = subdivide_path(exist_path[i], exist_path[i + 1], segment_length);
        if (i != 0) {
            segment_points.remove_at(0);
        }
        for (int k = 0; k < segment_points.size(); ++k) {
            subdivided_path.push_back(segment_points[k]);
        }
    }

    Vector<Vector3> smooth_path;
    smooth_path.push_back(subdivided_path[0]);

    int i = 0;
    while (i < subdivided_path.size() - 1) {
        int j = i + 1;
        while (j < subdivided_path.size()) {
            bool can_traverse;
            if (agent_radius > 0.0f) {
                can_traverse = can_travel_directly_with_cylinder(subdivided_path[i], subdivided_path[j], agent_radius, space_rid);
            }
            else {
                can_traverse = can_travel_directly_with_ray(subdivided_path[i], subdivided_path[j], space_rid);
            }

            if (!can_traverse) {
                break;
            }
            ++j;
        }

        if (j == i + 1) {
            // Avoid infinite loop by ensuring at least one progress
            j++;
        }

        smooth_path.push_back(subdivided_path[j - 1]);
        i = j - 1;
    }
    exist_path = smooth_path;
}

/**
 better smooth_path
 */
void SvoNavmesh::smooth_path_string_pulling_full(float agent_radius, RID& space_rid) {
    if (exist_path.size() < 2) {
        return;
    }

    Vector<Vector3> smooth_path;
    smooth_path.push_back(exist_path[0]);

    int i = 0;
    while (i < exist_path.size() - 1) {
        int best_j = i + 1;
        for (int j = i + 1; j < exist_path.size(); ++j) {
            bool can_traverse;
            if (agent_radius > 0.0f) {
                can_traverse = can_travel_directly_with_cylinder(exist_path[i], exist_path[j], agent_radius, space_rid);
            }
            else {
                can_traverse = can_travel_directly_with_ray(exist_path[i], exist_path[j], space_rid);
            }

            if (can_traverse) {
                best_j = j;
            }
        }

        smooth_path.push_back(exist_path[best_j]);
        i = best_j;
    }

    exist_path = smooth_path;
}

/**
 better smooth_path (with subdivide path)
 */
void SvoNavmesh::smooth_path_string_pulling_full_v2(float agent_radius, RID& space_rid) {
    if (exist_path.size() < 2) {
        return;
    }

    // First cut the entire path
    // 先对整个路径进行切割
    // 目前切割使用距离为: minVoxelSize*2
    Vector<Vector3> subdivided_path;
    float segment_length = calculateSegmentLength(rootVoxelSize, minVoxelSize);
    for (int i = 0; i < exist_path.size() - 1; ++i) {
        Vector<Vector3> segment_points = subdivide_path(exist_path[i], exist_path[i + 1], segment_length);
        if (i != 0) {
            segment_points.remove_at(0);
        }
        for (int k = 0; k < segment_points.size(); ++k) {
            subdivided_path.push_back(segment_points[k]);
        }
    }

    Vector<Vector3> smooth_path;
    smooth_path.push_back(subdivided_path[0]);

    int i = 0;
    while (i < subdivided_path.size() - 1) {
        int best_j = i + 1;
        for (int j = i + 1; j < subdivided_path.size(); ++j) {
            bool can_traverse;
            if (agent_radius > 0.0f) {
                can_traverse = can_travel_directly_with_cylinder(subdivided_path[i], subdivided_path[j], agent_radius, space_rid);
            }
            else {
                can_traverse = can_travel_directly_with_ray(subdivided_path[i], subdivided_path[j], space_rid);
            }

            if (can_traverse) {
                best_j = j;
            }
        }

        smooth_path.push_back(subdivided_path[best_j]);
        i = best_j;
    }

    exist_path = smooth_path;
}
/**
 cut the path
 */
Vector<Vector3> SvoNavmesh::subdivide_path(Vector3 start, Vector3 end, float segment_length) {
    Vector<Vector3> subdivided;
    subdivided.push_back(start); // Start point is always added

    Vector3 direction = end - start;
    float total_length = direction.length();
    direction = direction.normalized();

    float accumulated_length = segment_length;
    while (accumulated_length < total_length) {
        Vector3 new_point = start + direction * accumulated_length;
        subdivided.push_back(new_point);
        accumulated_length += segment_length;
    }

    subdivided.push_back(end); // End point is always added
    return subdivided;
}

/**
 * A* pathfinding.
 *
 * @param start: The path start position(world).
 * @param end: The path end position(world).
 * @param agent_r: The radius of nav agent.
 * @param is_smooth: Whether show smoothed path.
 */
Array SvoNavmesh::find_path_v1(const Vector3 start, const Vector3 end, float agent_r, bool is_smooth) {
    // both points need to be in empty
    // TODO: change this when ready for game
    if (query_voxel(start) || query_voxel(end)) {
        UtilityFunctions::print("Point inside SOLID!");
        return Array();
    }

    uint64_t begin_time_u = Time::get_singleton()->get_ticks_usec();
    uint64_t begin_time_m = Time::get_singleton()->get_ticks_msec();
    uint64_t end_time_u;
    uint64_t end_time_m;

    // check direct path
    Vector3 start_grid = worldToGrid(start);
    Vector3 end_grid = worldToGrid(end);
    RID space_rid = this->get_world_3d()->get_space();
    bool can_traverse;
    if (agent_r > 0.0f) {
        can_traverse = can_travel_directly_with_cylinder(start_grid, end_grid, agent_r, space_rid);
    }
    else {
        can_traverse = can_travel_directly_with_ray(start_grid, end_grid, space_rid);
    }

    if (can_traverse) {
        exist_path.clear();
        exist_path.append(start_grid);
        exist_path.append(end_grid);
        end_time_u = Time::get_singleton()->get_ticks_usec();
        end_time_m = Time::get_singleton()->get_ticks_msec();

        if (end_time_m - begin_time_m < 1) {
            // If the millisecond timing is less than 1 millisecond, use microsecond output
            // 如果毫秒计时小于1毫秒，使用微秒输出
            UtilityFunctions::print(vformat("Path finding took %d microseconds", end_time_u - begin_time_u));
        }
        else {
            // Otherwise use milliseconds output
            // 否则使用毫秒输出
            UtilityFunctions::print(vformat("Path finding took %d milliseconds", end_time_m - begin_time_m));
        }
    }
    else {
        // get raw path
        find_raw_path(start, end, agent_r);
        if (exist_path.is_empty()) {
            UtilityFunctions::print("Path finding failed!");
            return Array();
        }

        end_time_u = Time::get_singleton()->get_ticks_usec();
        end_time_m = Time::get_singleton()->get_ticks_msec();

        if (end_time_m - begin_time_m < 1) {
            // If the millisecond timing is less than 1 millisecond, use microsecond output
            // 如果毫秒计时小于1毫秒，使用微秒输出
            UtilityFunctions::print(vformat("Path finding took %d microseconds", end_time_u - begin_time_u));
        }
        else {
            // Otherwise use milliseconds output
            // 否则使用毫秒输出
            UtilityFunctions::print(vformat("Path finding took %d milliseconds", end_time_m - begin_time_m));
        }

        // smooth path
        if (is_smooth) {
            begin_time_u = Time::get_singleton()->get_ticks_usec();
            begin_time_m = Time::get_singleton()->get_ticks_msec();

            smooth_path_string_pulling_fast_v2(agent_r, space_rid);

            end_time_u = Time::get_singleton()->get_ticks_usec();
            end_time_m = Time::get_singleton()->get_ticks_msec();

            if (end_time_m - begin_time_m < 1) {
                // If the millisecond timing is less than 1 millisecond, use microsecond output
                // 如果毫秒计时小于1毫秒，使用微秒输出
                UtilityFunctions::print(vformat("Smooth path took %d microseconds", end_time_u - begin_time_u));
            }
            else {
                // Otherwise use milliseconds output
                // 否则使用毫秒输出
                UtilityFunctions::print(vformat("Smooth path took %d milliseconds", end_time_m - begin_time_m));
            }
        }
    }

    // debug rendering
    if(debug_mode) init_debug_path(agent_r * debug_path_scale);

    // return with array
    path_result.clear();
    for (const Vector3& point : exist_path) {
        path_result.append(gridToWorld(point));
    }
    return path_result;
}

/**
 * A* pathfinding void.
 *
 * @param start: The path start position(world).
 * @param end: The path end position(world).
 * @param agent_r: The radius of nav agent.
 * @param is_smooth: Whether show smoothed path.
 */
void SvoNavmesh::find_path_v2(const Vector3 start, const Vector3 end, float agent_r, bool is_smooth) {
    // both points need to be in empty
    // TODO: change this when ready for game
    if (query_voxel(start) || query_voxel(end)) {
        UtilityFunctions::print("Point inside SOLID!");
        return;
    }

    uint64_t begin_time_u = Time::get_singleton()->get_ticks_usec();
    uint64_t begin_time_m = Time::get_singleton()->get_ticks_msec();
    uint64_t end_time_u;
    uint64_t end_time_m;

    // check direct path
    Vector3 start_grid = worldToGrid(start);
    Vector3 end_grid = worldToGrid(end);
    RID space_rid = this->get_world_3d()->get_space();
    bool can_traverse;
    if (agent_r > 0.0f) {
        can_traverse = can_travel_directly_with_cylinder(start_grid, end_grid, agent_r, space_rid);
    }
    else {
        can_traverse = can_travel_directly_with_ray(start_grid, end_grid, space_rid);
    }

    if (can_traverse) {
        exist_path.clear();
        exist_path.append(start_grid);
        exist_path.append(end_grid);
        end_time_u = Time::get_singleton()->get_ticks_usec();
        end_time_m = Time::get_singleton()->get_ticks_msec();

        if (end_time_m - begin_time_m < 1) {
            // If the millisecond timing is less than 1 millisecond, use microsecond output
            // 如果毫秒计时小于1毫秒，使用微秒输出
            UtilityFunctions::print(vformat("Path finding took %d microseconds", end_time_u - begin_time_u));
        }
        else {
            // Otherwise use milliseconds output
            // 否则使用毫秒输出
            UtilityFunctions::print(vformat("Path finding took %d milliseconds", end_time_m - begin_time_m));
        }
    }
    else {
        // get raw path
        find_raw_path(start, end, agent_r);
        if (exist_path.is_empty()) {
            UtilityFunctions::print("Path finding failed!");
            return;
        }

        end_time_u = Time::get_singleton()->get_ticks_usec();
        end_time_m = Time::get_singleton()->get_ticks_msec();

        if (end_time_m - begin_time_m < 1) {
            // If the millisecond timing is less than 1 millisecond, use microsecond output
            // 如果毫秒计时小于1毫秒，使用微秒输出
            UtilityFunctions::print(vformat("Path finding took %d microseconds", end_time_u - begin_time_u));
        }
        else {
            // Otherwise use milliseconds output
            // 否则使用毫秒输出
            UtilityFunctions::print(vformat("Path finding took %d milliseconds", end_time_m - begin_time_m));
        }

        // smooth path
        if (is_smooth) {
            begin_time_u = Time::get_singleton()->get_ticks_usec();
            begin_time_m = Time::get_singleton()->get_ticks_msec();

            smooth_path_string_pulling_fast_v2(agent_r, space_rid);

            end_time_u = Time::get_singleton()->get_ticks_usec();
            end_time_m = Time::get_singleton()->get_ticks_msec();

            if (end_time_m - begin_time_m < 1) {
                // If the millisecond timing is less than 1 millisecond, use microsecond output
                // 如果毫秒计时小于1毫秒，使用微秒输出
                UtilityFunctions::print(vformat("Smooth path took %d microseconds", end_time_u - begin_time_u));
            }
            else {
                // Otherwise use milliseconds output
                // 否则使用毫秒输出
                UtilityFunctions::print(vformat("Smooth path took %d milliseconds", end_time_m - begin_time_m));
            }
        }
    }

    // debug rendering
    if (debug_mode) init_debug_path(agent_r * debug_path_scale);

    // return with array
    path_result.clear();
    for (const Vector3& point : exist_path) {
        path_result.append(gridToWorld(point));
    }
}

/**
 * A* pathfinding multi thread.
 *
 * @param start: The path start position(world).
 * @param end: The path end position(world).
 * @param agent_r: The radius of nav agent.
 * @param is_smooth: Whether show smoothed path.
 */
void SvoNavmesh::find_path_multi_thread(const Vector3 start, const Vector3 end, float agent_r, bool is_smooth) {
    // this one will just crash
    call_deferred("find_path_v2", start, end, agent_r);
}

/**
 init MeshInstance3D for path finding.
 For static debug draw only.
 */
void SvoNavmesh::init_debug_path(float agent_r) {
    // Clear old debug children
    // 清除旧的调试对象
    for (int i = 0; i < path_pool.size(); ++i) {
        remove_child(path_pool[i]);
        memdelete(path_pool[i]);
    }
    path_pool.clear();

    // SphereMesh for path points
    // 设置球形网格
    Ref<SphereMesh> sphereMesh = memnew(SphereMesh);
    if (agent_r > 0.0f) {
        sphereMesh->set_radius(agent_r); // 球体半径
        sphereMesh->set_height(agent_r * 2);
    }
    else {
        sphereMesh->set_radius(0.02); // 球体半径
        sphereMesh->set_height(0.04);
    }

    // CylinderMesh for path connecting lines
    // 设置圆柱体网格
    Ref<CylinderMesh> cylinderMesh = memnew(CylinderMesh);
    if (agent_r > 0.0f) {
        cylinderMesh->set_bottom_radius(agent_r / 2); // 圆柱体半径
        cylinderMesh->set_top_radius(agent_r / 2);
    }
    else {
        cylinderMesh->set_bottom_radius(0.01); // 圆柱体半径
        cylinderMesh->set_top_radius(0.01);
    }
    cylinderMesh->set_height(1);  // 默认高度，将会被缩放

    // Create path points & path connecting lines
    // 创建路径点和连接线
    for (int i = 0; i < exist_path.size(); ++i) {
        // If it is the last point
        // 如果是最后一个点
        if (i == exist_path.size() - 1) {
            MeshInstance3D* sphere = memnew(MeshInstance3D);

            sphere->set_mesh(sphereMesh);
            sphere->set_material_override(debugPathMaterialR);
            sphere->set_transform(Transform3D(Basis(), exist_path[i]));

            add_child(sphere);
            path_pool.push_back(sphere);
            break;
        }
        MeshInstance3D* sphere = memnew(MeshInstance3D);

        sphere->set_mesh(sphereMesh);
        sphere->set_material_override(debugPathMaterialY);
        sphere->set_transform(Transform3D(Basis(), exist_path[i]));

        add_child(sphere);
        path_pool.push_back(sphere);

        // If it is not the last point, create a cylinder connecting to the next point
        // 如果不是最后一个点，创建圆柱体连接到下一个点
        if (i < exist_path.size() - 1) {
            MeshInstance3D* cylinder = memnew(MeshInstance3D);

            cylinder->set_mesh(cylinderMesh);
            cylinder->set_material_override(debugPathMaterialB);

            // Place a cylinder between two waypoints
            // 在两个路径点之间放置一个圆柱体
            Vector3 direction = (exist_path[i + 1] - exist_path[i]).normalized();
            float distance = exist_path[i + 1].distance_to(exist_path[i]);
            // 默认的向上方向
            Vector3 up(0, 1, 0);
            // 计算旋转轴
            Vector3 rotation_axis = up.cross(direction).normalized();
            // 计算旋转角度
            float angle = acos(up.dot(direction));
            // 设置圆柱体的变换
            Transform3D transform;

            if (angle != 0) {
                // 创建四元数
                Quaternion rotation(rotation_axis, angle);
                transform.basis = Basis(rotation);
            }
            transform.origin = (exist_path[i] + exist_path[i + 1]) * 0.5;
            transform.basis.scale_local(Vector3(1, distance, 1)); // 圆柱体的缩放

            cylinder->set_transform(transform);
            add_child(cylinder);
            path_pool.push_back(cylinder);
        }
    }
}

/**
 * A* pathfinding adapted for use with SVO.
 *
 * @param start: The path start position(world).
 * @param end: The path end position(world).
 * @param agent_r: The radius of nav agent.
 */
void SvoNavmesh::find_raw_path(const Vector3 start, const Vector3 end, float agent_r) {
    Vector<OctreeNode*> open_set;
    Dictionary came_from;
    Dictionary g_score;
    Dictionary f_score;

    Vector3 start_grid = worldToGrid(start);
    Vector3 end_grid = worldToGrid(end);
    OctreeNode* start_node = svo->get_deepest_node(start_grid); // Convert world coordinates to SVO grid coordinates
    OctreeNode* end_node = svo->get_deepest_node(end_grid);

    came_from[start_node->center] = start_grid;  // manual add start
    open_set.push_back(start_node);
    g_score[start_node->get_instance_id()] = 0;
    f_score[start_node->get_instance_id()] = heuristic(start_node, end_node);

    while (!open_set.is_empty()) {
        OctreeNode* current = get_lowest_f_score_node(open_set, f_score);
        if (current == end_node) {
            came_from[end_grid] = end_node->center;  // manual add end
            exist_path = reconstruct_path(came_from, end_grid);
            return;
        }

        open_set.erase(current);
        Vector<OctreeNode*> neighbors = get_neighbors(current, agent_r);

        for (int i = 0; i < neighbors.size(); ++i) {
            OctreeNode* neighbor = neighbors[i];

            // If the neighbor is not in g_score, initialize it to a very high value.
            // This is because in svo, we cannt init g_score for all the OctreeNodes before loops.
            // std:: is used here, which is not good for godot regulation.
            // 如果neighbor不在g_score中，将其初始化为非常高的值
            if (!g_score.has(neighbor->get_instance_id())) {
                g_score[neighbor->get_instance_id()] = std::numeric_limits<float>::max();  // 使用最大float值初始化
            }

            //float temp_current = (float)g_score[current];
            float tentative_g_score = (float)g_score[current->get_instance_id()] + current->center.distance_to(neighbor->center);
            //float temp_neighbor = (float)g_score[neighbor];

            if (tentative_g_score < (float)g_score[neighbor->get_instance_id()]) {
                came_from[neighbor->center] = current->center;
                g_score[neighbor->get_instance_id()] = tentative_g_score;
                f_score[neighbor->get_instance_id()] = tentative_g_score + heuristic(neighbor, end_node);

                if (!open_set.has(neighbor)) {
                    open_set.push_back(neighbor);
                }
            }
        }
    }
    exist_path = Vector<Vector3>();
    // Return empty path if no path found
}
