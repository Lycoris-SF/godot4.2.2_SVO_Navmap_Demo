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
    ClassDB::bind_method(D_METHOD("check_voxel_with_id", "id"), &SvoNavmesh::check_voxel_with_id);
    ClassDB::bind_method(D_METHOD("update_voxel", "position", "isSolid"), &SvoNavmesh::update_voxel);
    ClassDB::bind_method(D_METHOD("get_info"), &SvoNavmesh::get_info);
    ClassDB::bind_method(D_METHOD("set_info", "p_info"), &SvoNavmesh::set_info);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "testdouble"), "set_info", "get_info");
    ClassDB::bind_method(D_METHOD("get_debug_mode"), &SvoNavmesh::get_debug_mode);
    ClassDB::bind_method(D_METHOD("set_debug_mode", "debug_mode"), &SvoNavmesh::set_debug_mode);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::BOOL, "debug_mode"), "set_debug_mode", "get_debug_mode");

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
    ClassDB::bind_method(D_METHOD("get_show_empty"), &SvoNavmesh::get_show_empty);
    ClassDB::bind_method(D_METHOD("set_show_empty", "show_empty"), &SvoNavmesh::set_show_empty);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::BOOL, "show_empty"), "set_show_empty", "get_show_empty");

    // button method
    ClassDB::bind_method(D_METHOD("rebuild_svo"), &SvoNavmesh::rebuild_svo);
    ClassDB::bind_method(D_METHOD("refresh_svo"), &SvoNavmesh::refresh_svo);
    ClassDB::bind_method(D_METHOD("clear_svo"), &SvoNavmesh::clear_svo);

    // generate svo from collider
    ClassDB::bind_method(D_METHOD("insert_svo_based_on_collision_shapes"), &SvoNavmesh::insert_svo_based_on_collision_shapes);

    // path finding
    ClassDB::bind_method(D_METHOD("find_path", "start", "end", "agent_r", "is_smooth"), &SvoNavmesh::find_path);
}

SvoNavmesh::SvoNavmesh(){
    maxDepth = 3;
    voxelSize = 1.0f;
    testdouble = 1.14;

    debug_mode = false;
    DrawRef_minDepth = 1;
    DrawRef_maxDepth = 3;
    show_empty = true;
    debugChecked_node = nullptr;

    svo = memnew(SparseVoxelOctree);
}

SvoNavmesh::~SvoNavmesh() {
    //call_deferred("reset_pool");
    if(svo) memdelete(svo);
    //reset_wastepool();
}

// During set_neighbors_from_brother, the index of each child node corresponds to the index in the neighbors array.
// set_neighbors_from_brotherʱ��ÿ���ӽڵ�������Ӧ��neighbors�����е�����
static const int neighborsMap[8][3] = {
    {0, 2, 4}, // ����0���ھ���neighbors���������
    {1, 2, 4}, // The index of neighbor for index 1 in the neighbors array.
    {0, 3, 4},
    {1, 3, 4},
    {0, 2, 5},
    {1, 2, 5},
    {0, 3, 5},
    {1, 3, 5}
};
// During set_neighbors_from_brother, each child node corresponds to an index in the father node's children array.
// set_neighbors_from_brotherʱ��ÿ���ӽڵ��Ӧ���ڵ��children�����е�����
static const int childrenMap[8][3] = {
    {1, 2, 4}, // �ӽڵ�0��Ҫ���ҵĸ��ڵ���ӽڵ�����
    {0, 3, 5}, // The index of the child node in the father node that needs to be searched for child node 1.
    {3, 0, 6},
    {2, 1, 7},
    {5, 6, 0},
    {4, 7, 1},
    {7, 4, 2},
    {6, 5, 3}
};
// During get_neighbors, each direction corresponds to the index in the children array of the neighbor.
// get_neighborsʱ��ÿ�������Ӧ�ھӵ�children�����е�����
static const int childDirectionMap[6][4] = {
    {0, 2, 4, 6}, // +X �������棬��Ҫ���X=0�������ӽڵ�
    {1, 3, 5, 7}, // -X ���򣺶��棬��Ҫ���X=1�������ӽڵ�
    {0, 1, 4, 5}, // +Y �������棬��Ҫ���Y=0�������ӽڵ�
    {2, 3, 6, 7}, // -Y �������棬��Ҫ���Y=1�������ӽڵ�
    {0, 1, 2, 3}, // +Z �������棬��Ҫ���Z=0�������ӽڵ�
    {4, 5, 6, 7}, // -Z ���򣺱��棬��Ҫ���Z=1�������ӽڵ�
};

/**
 * Converts a world position to a local grid position.
 *
 * @param world_position: The position in world coordinates.
 * @returns The position in local coordinates of svo.
 */
Vector3 SvoNavmesh::worldToGrid(Vector3 world_position) {
    Transform3D global_transform = get_global_transform();
    Vector3 local_position = global_transform.xform_inv(world_position); // Global to local; ȫ�ֵ��ֲ�
    return local_position; // Convert to grid coordinates; ת������������
}
Vector3 SvoNavmesh::gridToWorld(Vector3 grid_position) {
    Transform3D global_transform = get_global_transform();
    return global_transform.xform(grid_position);
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
    // ������������Ƿ���Ч(���ڸ��ڵ㷶Χ��)
    if (grid_position.x < -voxelSize / 2 || grid_position.y < -voxelSize / 2 || grid_position.z < -voxelSize / 2 ||
        grid_position.x >= voxelSize / 2 || grid_position.y >= voxelSize / 2 || grid_position.z >= voxelSize / 2) {
        UtilityFunctions::print("insert: Position out of valid range.");
        return;
    }

    refresh_svo();
    reset_pool();
    svo->insert(grid_position);
    init_debug_mesh(svo->root, 1);
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
    // ������������Ƿ���Ч(���ڸ��ڵ㷶Χ��)
    if (grid_position.x < -voxelSize / 2 || grid_position.y < -voxelSize / 2 || grid_position.z < -voxelSize / 2 ||
        grid_position.x >= voxelSize / 2 || grid_position.y >= voxelSize / 2 || grid_position.z >= voxelSize / 2) {
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
    // ���·��ID�Ƿ�ֻ��һ���ַ���Ϊ'0'
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

        // ����ַ��Ƿ���Ч
        if (child_index < '0' || child_index > '7') {
            UtilityFunctions::print("Invalid ID: ", id);
            return;
        }

        // ת���ַ�Ϊ�ӽڵ�����
        int index = child_index - '0';

        // ����ӽڵ��Ƿ����
        if (!current_node->children[index]) {
            UtilityFunctions::print("Node does not exist at path: ", id.substr(0, i + 1));
            return;
        }

        // �ƶ����ӽڵ�
        current_node = current_node->children[index];
    }
    // ��ӡ�ҵ���������Ϣ
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
//��ʱ����; Temp Abandon
void SvoNavmesh::update_voxel(Vector3 world_position, bool isSolid) {
    Vector3 grid_position = worldToGrid(world_position);

    // Check if the grid coordinates are valid (i.e. within the root node range)
    // ������������Ƿ���Ч(���ڸ��ڵ㷶Χ��)
    if (grid_position.x < -voxelSize / 2 || grid_position.y < -voxelSize / 2 || grid_position.z < -voxelSize / 2 ||
        grid_position.x >= voxelSize / 2 || grid_position.y >= voxelSize / 2 || grid_position.z >= voxelSize / 2) {
        UtilityFunctions::print("update: Position out of valid range.");
        return;
    }

    refresh_svo();
    svo->update(grid_position, isSolid);
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
bool SvoNavmesh::get_debug_mode() const {
    return debug_mode;
}
void SvoNavmesh::set_debug_mode(bool debug_mode) {
    this->debug_mode = debug_mode;
    // TODO: when ready for demo game,
    //       this needs to be removed,
    //       debug_mode should only control rendering in game.
    if (debug_mode) {
        reset_pool();
        init_debug_mesh(svo->root, 1);
    }
    else force_clear_debug_mesh();
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
// </debug draw set/get>

/**
 Clear the svo and total rebuild base on collision shapes.
 */
void SvoNavmesh::rebuild_svo() {
    uint64_t begin = Time::get_singleton()->get_ticks_msec();

    svo->maxDepth = maxDepth;
    svo->voxelSize = voxelSize;

    if (debug_mode) {
        reset_pool();
        svo->clear();
        //init_debug_mesh(svo->root, 1);

        insert_svo_based_on_collision_shapes();
        //reset_pool();
        init_debug_mesh(svo->root, 1);
        init_neighbors();
    }
    else {
        svo->clear();
        insert_svo_based_on_collision_shapes();
        init_neighbors();
    }

    uint64_t end = Time::get_singleton()->get_ticks_msec();
    WARN_PRINT_ED(vformat("build svo with %d milliseconds", end - begin));
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
    //if (debug_mode) init_debug_mesh(svo->root, 1);
    init_neighbors();
}

/**
 * Clear the svo.
 *
 * @param clear_setting: whether also clear svo setting.
 */
void SvoNavmesh::clear_svo(bool clear_setting) {
    if (debug_mode) reset_pool();
    if (clear_setting) {
        // clear svo and settings
        maxDepth = 3;
        voxelSize = 1.0f;
        svo->clear();
    }
    else {
        // clear only svo
        svo->clear();
    }
    if (debug_mode) init_debug_mesh(svo->root, 1);
    init_neighbors();
}

//������; Abandon
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
    traverse_svo_space_and_insert(svo->root, 1, space_rid);

    uint64_t end = Time::get_singleton()->get_ticks_msec();
    WARN_PRINT_ED(vformat("insert svo nodes with %d milliseconds", end - begin));
}

/**
 Use Godot's physics engine to check if the point is inside CollisionShape3D.
 */
bool check_point_inside_mesh(Vector3 point, RID& space_rid) {
    // Get the PhysicsDirectSpaceState3D instance
    // ��ȡ PhysicsDirectSpaceState3D ʵ��
    PhysicsDirectSpaceState3D* space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(space_rid);
    if (!space_state) {
        ERR_PRINT_ED("Failed to get PhysicsDirectSpaceState3D instance");
        return false;
    }

    // Create and configure query parameters
    // ���������ò�ѯ����
    Ref<PhysicsPointQueryParameters3D> query_params;
    query_params.instantiate();
    query_params->set_position(point);  // Set the position of the query point; ���ò�ѯ���λ��

    // Execute query
    // ִ�е��ڲ�ѯ
    Array results = space_state->intersect_point(query_params, 32);  // max_results = 32

    // Process the query results
    // �����ѯ���
    if (!results.is_empty()) {
        for (int i = 0; i < results.size(); ++i) {
            Dictionary result = results[i];
            // Compare collider object RID
            // �Ա� collider ����RID
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
bool check_box_intersect_mesh(Vector3 position, Quaternion rotation, float size, RID& space_rid) {
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
bool is_box_fully_inside_mesh(Vector3 position, float size, RID& space_rid) {
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

//������; Abandon
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
                // �����Ƿ��������ڲ�
                if (check_point_inside_mesh(point, space_rid)) {
                    sampled_points.push_back(point);
                }
            }
        }
    }

    // Processing surface vertices
    // ������涥��
    PackedVector3Array faces = mesh->get_faces();
    for (int i = 0; i < faces.size(); i++) {
        sampled_points.push_back(faces[i]);
    }

    return sampled_points;
}

//������; Abandon
Vector<Vector3> get_box_corners(CollisionShape3D* collision_shape) {
    Vector<Vector3> corners;
    Ref<BoxShape3D> box_shape = collision_shape->get_shape();

    if (box_shape.is_valid()) {
        Vector3 size = box_shape->get_size();
        Vector3 extents = size * 0.5; // ����extents
        Transform3D global_transform = collision_shape->get_global_transform();

        // ����8���ǵ�
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

//������; Abandon
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
    // ���������е�ÿ���ڵ�
    for (int i = 0; i < node->get_child_count(); ++i) {
        Node* child = node->get_child(i);

        // Try to convert the child node to PhysicsBody3D
        // ���Խ��ӽڵ�ת��ΪPhysicsBody3D
        PhysicsBody3D* physics_body = Object::cast_to<PhysicsBody3D>(child);
        if (physics_body) {
            // Traverse the child nodes of PhysicsBody3D to find CollisionShape3D
            // ����PhysicsBody3D���ӽڵ�������CollisionShape3D
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
        // �ݹ���ռ��ӽڵ����ײ��״
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
        reset_pool();
        init_debug_mesh(svo->root, 1);
    }
    init_neighbors();
}
void SvoNavmesh::_process(double delta) {
    if (debug_mode) draw_svo_v2(svo->root, 1, DrawRef_minDepth, DrawRef_maxDepth);
}
void SvoNavmesh::_physics_process(double delta)
{

}
static void init_static_material()
{
    // debug draw
    if (debugSolidMaterial.is_null()) debugSolidMaterial.instantiate();
    debugSolidMaterial->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
    debugSolidMaterial->set_albedo(Color(0.2, 1.0, 0.2, 0.2));  // ��͸����ɫ

    if (debugCheckMaterial.is_null()) debugCheckMaterial.instantiate();
    debugCheckMaterial->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
    debugCheckMaterial->set_albedo(Color(0.2, 0.2, 1.0, 0.2));  // ��͸����ɫ

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
    debugSolidMaterial.unref();
    debugCheckMaterial.unref();
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
 ǣ�浽svo�ṹ�仯��Ҫ�ֶ����ô˷���
 Debug rendering only
 */
/*void SvoNavmesh::reset_pool_v1() {
    if (!mesh_pool.is_empty()) {
        for (MeshInstance3D* instance : mesh_pool) {
            if (instance && instance->get_parent() == this) {
                remove_child(instance);
                // if (!waste_pool.has(instance)) waste_pool.push_back(instance);
                // instance->queue_free();
            } else {
                UtilityFunctions::print("Instance is not a child of this node or is null");
            }
        }
        mesh_pool.clear();
    }
    if (!path_pool.is_empty()) {
        // Clear existing path_pool if necessary
        for (int i = 0; i < path_pool.size(); ++i) {
            if (path_pool[i] && path_pool[i]->get_parent() == this) {
                remove_child(path_pool[i]);
                memdelete(path_pool[i]);
            } else {
                UtilityFunctions::print("Path pool element is not a child of this node or is null");
            }
        }
        path_pool.clear();
    }
    target_rids.clear();
    reset_debugCheck();

    // UtilityFunctions::print(vformat("Waste pool count: %d", waste_pool.size()));
}*/
void SvoNavmesh::reset_pool() {
    if (!mesh_pool.is_empty()) {
        for (MeshInstance3D* instance : mesh_pool) {
            if (instance && instance->is_inside_tree()) {
                remove_child(instance);
                instance->queue_free();
            }
        }
        mesh_pool.clear();
    }
    if (!path_pool.is_empty()) {
        // Clear existing path_pool if necessary
        for (MeshInstance3D* instance : path_pool) {
            remove_child(instance);
            instance->queue_free();
        }
        path_pool.clear();
    }
    target_rids.clear();
    reset_debugCheck();
}

void SvoNavmesh::force_clear_debug_mesh() {
    if (!mesh_pool.is_empty()) {
        for (MeshInstance3D* instance : mesh_pool) {
            if (instance) {
                remove_child(instance);
                instance->queue_free();
            }
        }
        mesh_pool.clear();
    }
    if (!path_pool.is_empty()) {
        // Clear existing path_pool if necessary
        for (MeshInstance3D* instance : path_pool) {
            remove_child(instance);
            instance->queue_free();
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
 This method needs to be called manually if changes are involved in the svo structure.
 ǣ�浽svo�ṹ�仯��Ҫ�ֶ����ô˷���
 Debug rendering only
 */
void SvoNavmesh::init_debug_mesh(OctreeNode* node, int depth)
{
    if (!node || depth > svo->maxDepth) return;

    float size = 2 * voxelSize / pow(2, depth);

    if (depth <= svo->maxDepth) {
        if (!node->debugMesh) {
            node->debugMesh = memnew(MeshInstance3D);
            //mesh_count_log.test_countA++;
        }
        if (node->debugBoxMesh.is_null()) {
            node->debugBoxMesh = Ref<BoxMesh>(memnew(BoxMesh));
        }
        node->debugBoxMesh->set_size(Vector3(size, size, size));
        node->debugMesh->set_mesh(node->debugBoxMesh);

        // Set up materials
        // ���ò���
        if (node->voxel && node->voxel->isSolid())
        {
            node->debugMesh->set_material_override(debugSolidMaterial);
        }
        else {
            node->debugMesh->set_material_override(debugEmptyMaterial);
        }

        add_child(node->debugMesh);
        mesh_pool.push_back(node->debugMesh);
    }

    // Recursively traverse child nodes
    // �ݹ�����ӽڵ�
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            init_debug_mesh(node->children[i], depth + 1);
        }
    }
}

/**
 Breadth-first traversal init neighbors
 ������ȱ�����ʼ��neighbors
 This method needs to be called manually if changes are involved in the svo structure.
 ǣ�浽svo�ṹ�仯��Ҫ�ֶ����ô˷���
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
    if (node->father == nullptr) return; // �ų����ڵ�; Exclude root nodes

    // Set the neighbor relationship from the brother node
    // �ֵܽڵ����ֱ�ӵ�neighbor��ϵ
    set_neighbors_from_brother(node);

    // Need to get neighbors from the father node or further nodes
    // ��Ҫ�Ӹ��ڵ���Զ�ڵ��ȡ�ھ�
    for (int i = 0; i < 3; i++) {
        int neighborIndex = neighborsMap[7-node->currentIndex][i];  // Reverse index to get correct external neighbor direction; ���������Ի�ȡ��ȷ���ⲿ�ھӷ���
        int childIndex = childrenMap[node->currentIndex][i];        // Get the child node index, the magic is that there is no need to rebuild the index; ��ȡ�ӽڵ�������������ǲ���Ҫ�ؽ�����
        OctreeNode* externalNeighbor = node->father->neighbors[neighborIndex];
        // ���Դ��ⲿ�ھӵ��ӽڵ��ȡ�ھ�; Try to get neighbors from the children of external neighbors
        if (externalNeighbor != nullptr) {
            if (!externalNeighbor->isLeaf && externalNeighbor->children[childIndex] != nullptr) {
                node->neighbors[neighborIndex] = externalNeighbor->children[childIndex];
            }
            else {
                // If the corresponding child node of the external neighbor does not exist, 
                // use the external neighbor as the neighbor
                // ����ⲿ�ھ���Ҷ�ڵ����Ӧ�ӽڵ㲻���ڣ���ʹ���ⲿ�ھӵĸ��ڵ���Ϊ�ھ�
                node->neighbors[neighborIndex] = externalNeighbor;
            }
        }
    }
}
void SvoNavmesh::set_neighbors_from_brother(OctreeNode* node) {
    // Set the neighbor of the current node from the children array of the father node
    // �Ӹ��ڵ��children���������õ�ǰ�ڵ���ھ�
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
    if (!node->debugMesh) {
        WARN_PRINT_ONCE(vformat("debugMesh is null"));
        return;
    }
    if (!node || current_depth > svo->maxDepth) return;

    bool is_painting = node->voxel->isSolid() || (show_empty && node->voxel->isEmpty());

    if (is_painting || node->debugChecked) {
        float size = 2 * voxelSize / pow(2, current_depth);

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
            draw_svo_v2(node->children[i], current_depth + 1, min_depth, max_depth);
        }
    }
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
    float lowest_score = f_score[lowest->debugMesh->get_instance_id()];
    for (int i = 1; i < open_set.size(); i++) {
        OctreeNode* temp = open_set[i];
        float score = f_score[temp->debugMesh->get_instance_id()];
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
    if (node->voxel->size/2 < agent_r) return; // ����ڵ�̫С�����ʺϴ���ͨ����ֹͣ�ݹ�

    for (int j = 0; j < 4; ++j) {
        OctreeNode* sub_neighbor = node->children[childDirectionMap[direction][j]];
        if (!sub_neighbor) continue;
        if (sub_neighbor->voxel->isLiquid()) {
            add_liquid_children(neighbors, sub_neighbor, agent_r, direction); // �ݹ�����Լ���С���ӽڵ�
        }
        else if (sub_neighbor->voxel->isEmpty()) {
            neighbors.push_back(sub_neighbor); // �սڵ�ֱ�Ӽ���
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
            // ����Һ̬�ڵ㣬�ݹ������ӽڵ�
            add_liquid_children(neighbors, neighbor, agent_r, i);
        }
        else if (neighbor->voxel->isEmpty()) {
            if (neighbor->voxel->size < agent_r) continue; // ����ڵ�̫С�����ʺϴ���ͨ��
            neighbors.push_back(neighbor); // �սڵ�ֱ�Ӽ���
        }
    }
    return neighbors;
}

/**
 check if a direct path (with radius) between two points is feasible
 */
bool SvoNavmesh::can_travel_directly_with_cylinder(const Vector3& from, const Vector3& to, float agent_radius, RID& space_rid) {
    // Get the PhysicsDirectSpaceState3D instance
    // ��ȡ PhysicsDirectSpaceState3D ʵ��
    PhysicsDirectSpaceState3D* space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(space_rid);
    if (!space_state) {
        ERR_PRINT_ED("Failed to get PhysicsDirectSpaceState3D instance");
        return false;
    }

    Vector3 from_w = gridToWorld(from);
    Vector3 to_w = gridToWorld(to);

    // Set up Cylinder
    // ����Բ��������
    Ref<CylinderShape3D> cylinder_shape = memnew(CylinderShape3D);
    cylinder_shape->set_radius(agent_radius);
    cylinder_shape->set_height(from_w.distance_to(to_w));

    CollisionShape3D collision_shape;
    collision_shape.set_shape(cylinder_shape);

    // Set the transform of the query parameters
    Transform3D transform;
    Vector3 direction = (to_w - from_w).normalized();
    // Ĭ�ϵ����Ϸ���
    Vector3 up(0, 1, 0);
    // ������ת��
    Vector3 rotation_axis = up.cross(direction).normalized();
    // ������ת�Ƕ�
    float angle = acos(up.dot(direction));
    if (angle != 0) {
        // ������Ԫ��
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
    //query_params->set_collide_with_areas(true);
    //query_params->set_collide_with_bodies(true);
    //query_params->set_collision_mask(1); // Adjust collision mask as needed
    //query_params->set_margin(0.1);

    // ABANDON: 
    bool debug_smooth = false;
    if (debug_smooth) {
        // Create Materials for Debugging
        // �������ڵ��ԵĲ���
        // Should use static ones instead
        Ref<StandardMaterial3D> material = memnew(StandardMaterial3D);
        material->set_albedo(Color(0.2, 0.7, 0.2)); // ��ɫ
        // ����Բ��������
        Ref<CylinderMesh> cylinderMesh = memnew(CylinderMesh);
        cylinderMesh->set_bottom_radius(agent_radius / 2); // Բ����뾶
        cylinderMesh->set_top_radius(agent_radius / 2);
        cylinderMesh->set_height(from_w.distance_to(to_w));
        MeshInstance3D* cylinder = memnew(MeshInstance3D);
        cylinder->set_mesh(cylinderMesh);
        cylinder->set_material_override(material);
        Transform3D transform_d = transform;
        transform_d.basis.scale_local(Vector3(1.1, 1, 1.1)); // Բ���������
        cylinder->set_transform(transform_d);
        add_child(cylinder);
        path_pool.push_back(cylinder);
    }

    // Execute query
    // ִ�е��ڲ�ѯ
    Array results = space_state->intersect_shape(query_params, 32);

    // Process the query results
    // �����ѯ���
    if (!results.is_empty()) {
        for (int i = 0; i < results.size(); ++i) {
            Dictionary result = results[i];
            // Compare collider object RID
            // �Ա� collider ����RID
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
    // ��ȡ PhysicsDirectSpaceState3D ʵ��
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

    // Execute query
    // ִ�е��ڲ�ѯ
    Dictionary result = space_state->intersect_ray(query_params);

    // Process the query results
    // �����ѯ���
    if (!result.is_empty()) {
        RID result_rid = result["rid"];
        if (target_rids.has(result_rid)) {
            return false;
        }
    }
    return true;
}

/**
 fast smooth_path (though not lot faster)
 */
Vector<Vector3> SvoNavmesh::smooth_path_string_pulling_fast(const Vector<Vector3>& path, float agent_radius, RID& space_rid) {
    if (path.size() < 2) {
        return path;
    }

    Vector<Vector3> smooth_path;
    smooth_path.push_back(path[0]);

    int i = 0;
    while (i < path.size() - 1) {
        int j = i + 1;
        while (j < path.size()) {
            bool can_traverse;
            if (agent_radius > 0.0f) {
                can_traverse = can_travel_directly_with_cylinder(path[i], path[j], agent_radius, space_rid);
            }
            else {
                can_traverse = can_travel_directly_with_ray(path[i], path[j], space_rid);
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

        smooth_path.push_back(path[j - 1]);
        i = j - 1;
    }

    return smooth_path;
}

/**
 perfect smooth_path
 */
Vector<Vector3> SvoNavmesh::smooth_path_string_pulling_best(const Vector<Vector3>& path, float agent_radius, RID& space_rid) {
    if (path.size() < 2) {
        return path;
    }

    Vector<Vector3> smooth_path;
    smooth_path.push_back(path[0]);

    int i = 0;
    while (i < path.size() - 1) {
        int best_j = i + 1;
        for (int j = i + 1; j < path.size(); ++j) {
            bool can_traverse;
            if (agent_radius > 0.0f) {
                can_traverse = can_travel_directly_with_cylinder(path[i], path[j], agent_radius, space_rid);
            }
            else {
                can_traverse = can_travel_directly_with_ray(path[i], path[j], space_rid);
            }

            if (can_traverse) {
                best_j = j;
            }
        }

        smooth_path.push_back(path[best_j]);
        i = best_j;
    }

    return smooth_path;
}

/**
 * A* pathfinding.
 *
 * @param start: The path start position(world).
 * @param end: The path end position(world).
 * @param agent_r: The radius of nav agent.
 * @param is_smooth: Whether show smoothed path.
 */
Array SvoNavmesh::find_path(const Vector3 start, const Vector3 end, float agent_r, bool is_smooth) {
    // both points need to be in empty
    // TODO: change this when ready for game
    if (query_voxel(start) || query_voxel(end)) {
        UtilityFunctions::print("Point inside SOLID!");
        return Array();
    }

    // get raw path
    uint64_t begin_time = Time::get_singleton()->get_ticks_usec();

    Vector<Vector3> path = find_raw_path(start, end, agent_r);
    if (path.is_empty()) {
        UtilityFunctions::print("Path finding failed!");
        return Array();
    }

    uint64_t end_time = Time::get_singleton()->get_ticks_usec();
    WARN_PRINT_ED(vformat("Path finding with %d microseconds", end_time - begin_time));

    // smooth path
    if (is_smooth) {
        RID space_rid = this->get_world_3d()->get_space();

        begin_time = Time::get_singleton()->get_ticks_usec();

        path = smooth_path_string_pulling_best(path, agent_r, space_rid);

        end_time = Time::get_singleton()->get_ticks_usec();
        WARN_PRINT_ED(vformat("Smooth path with %d microseconds", end_time - begin_time));
    }

    debug_path = path; // Store the path for else possible use

    // debug rendering
    if(debug_mode) init_debug_path(path, agent_r);

    // return with array
    Array path_array;
    for (const Vector3& point : path) {
        path_array.append(point);
    }
    return path_array;
}

/**
 init MeshInstance3D for path finding.
 For static debug draw only.
 */
void SvoNavmesh::init_debug_path(const Vector<Vector3>& path, float agent_r) {
    // Clear old debug children
    // ����ɵĵ��Զ���
    for (int i = 0; i < path_pool.size(); ++i) {
        remove_child(path_pool[i]);
        memdelete(path_pool[i]);
    }
    path_pool.clear();

    // Create Materials for Debugging
    // �������ڵ��ԵĲ���
    // Should use static ones instead
    Ref<StandardMaterial3D> material = memnew(StandardMaterial3D);
    Ref<StandardMaterial3D> material2 = memnew(StandardMaterial3D);
    material->set_albedo(Color(0.2, 0.2, 0.7)); // ��ɫ
    material2->set_albedo(Color(0.9, 0.6, 0.1)); // ��ɫ

    // SphereMesh for path points
    // ������������
    Ref<SphereMesh> sphereMesh = memnew(SphereMesh);
    if (agent_r > 0.0f) {
        sphereMesh->set_radius(agent_r); // ����뾶
        sphereMesh->set_height(agent_r * 2);
    }
    else {
        sphereMesh->set_radius(0.02); // ����뾶
        sphereMesh->set_height(0.04);
    }

    // CylinderMesh for path connecting lines
    // ����Բ��������
    Ref<CylinderMesh> cylinderMesh = memnew(CylinderMesh);
    if (agent_r > 0.0f) {
        cylinderMesh->set_bottom_radius(agent_r / 2); // Բ����뾶
        cylinderMesh->set_top_radius(agent_r / 2);
    }
    else {
        cylinderMesh->set_bottom_radius(0.01); // Բ����뾶
        cylinderMesh->set_top_radius(0.01);
    }
    cylinderMesh->set_height(1);  // Ĭ�ϸ߶ȣ����ᱻ����

    // Create path points & path connecting lines
    // ����·�����������
    for (int i = 0; i < path.size(); ++i) {
        MeshInstance3D* sphere = memnew(MeshInstance3D);

        //Vector3 world_point = gridToWorld(path[i]);
        //UtilityFunctions::print(vformat("world_point %d: %v", i, world_point));

        sphere->set_mesh(sphereMesh);
        sphere->set_material_override(material2);
        sphere->set_transform(Transform3D(Basis(), path[i]));

        add_child(sphere);
        path_pool.push_back(sphere);

        // If it is not the last point, create a cylinder connecting to the next point
        // ����������һ���㣬����Բ�������ӵ���һ����
        if (i < path.size() - 1) {
            MeshInstance3D* cylinder = memnew(MeshInstance3D);

            cylinder->set_mesh(cylinderMesh);
            cylinder->set_material_override(material);

            // Place a cylinder between two waypoints
            // ������·����֮�����һ��Բ����
            //Vector3 next_world_point = gridToWorld(path[i + 1]);
            Vector3 direction = (path[i + 1] - path[i]).normalized();
            float distance = path[i + 1].distance_to(path[i]);
            // Ĭ�ϵ����Ϸ���
            Vector3 up(0, 1, 0);
            // ������ת��
            Vector3 rotation_axis = up.cross(direction).normalized();
            // ������ת�Ƕ�
            float angle = acos(up.dot(direction));
            // ����Բ����ı任
            Transform3D transform;

            if (angle != 0) {
                // ������Ԫ��
                Quaternion rotation(rotation_axis, angle);
                transform.basis = Basis(rotation);
            }
            transform.origin = (path[i] + path[i + 1]) * 0.5;
            transform.basis.scale_local(Vector3(1, distance, 1)); // Բ���������

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
Vector<Vector3> SvoNavmesh::find_raw_path(const Vector3 start, const Vector3 end, float agent_r) {
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
    g_score[start_node->debugMesh->get_instance_id()] = 0;
    f_score[start_node->debugMesh->get_instance_id()] = heuristic(start_node, end_node);

    while (!open_set.is_empty()) {
        OctreeNode* current = get_lowest_f_score_node(open_set, f_score);
        if (current == end_node) {
            came_from[end_grid] = end_node->center;  // manual add end
            return reconstruct_path(came_from, end_grid);
        }

        open_set.erase(current);
        Vector<OctreeNode*> neighbors = get_neighbors(current, agent_r);

        for (int i = 0; i < neighbors.size(); ++i) {
            OctreeNode* neighbor = neighbors[i];

            // If the neighbor is not in g_score, initialize it to a very high value.
            // This is because in svo, we cannt init g_score for all the OctreeNodes before loops.
            // std:: is used here, which is not good for godot regulation.
            // ���neighbor����g_score�У������ʼ��Ϊ�ǳ��ߵ�ֵ
            if (!g_score.has(neighbor->debugMesh->get_instance_id())) {
                g_score[neighbor->debugMesh->get_instance_id()] = std::numeric_limits<float>::max();  // ʹ�����floatֵ��ʼ��
            }

            //float temp_current = (float)g_score[current];
            float tentative_g_score = (float)g_score[current->debugMesh->get_instance_id()] + current->center.distance_to(neighbor->center);
            //float temp_neighbor = (float)g_score[neighbor];

            if (tentative_g_score < (float)g_score[neighbor->debugMesh->get_instance_id()]) {
                came_from[neighbor->center] = current->center;
                g_score[neighbor->debugMesh->get_instance_id()] = tentative_g_score;
                f_score[neighbor->debugMesh->get_instance_id()] = tentative_g_score + heuristic(neighbor, end_node);

                if (!open_set.has(neighbor)) {
                    open_set.push_back(neighbor);
                }
            }
        }
    }
    return Vector<Vector3>(); // Return empty path if no path found
}
