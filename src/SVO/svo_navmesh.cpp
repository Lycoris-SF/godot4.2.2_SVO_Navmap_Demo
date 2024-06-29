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

    ClassDB::bind_method(D_METHOD("get_origin_position"), &SvoNavmesh::get_origin_position);
    ClassDB::bind_method(D_METHOD("set_origin_position", "position"), &SvoNavmesh::set_origin_position);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::VECTOR3, "origin_position"), "set_origin_position", "get_origin_position");

    ClassDB::bind_method(D_METHOD("get_origin_rotation"), &SvoNavmesh::get_origin_rotation);
    ClassDB::bind_method(D_METHOD("set_origin_rotation", "rotation"), &SvoNavmesh::set_origin_rotation);
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

    //
    //mesh_count_log.log_textA = "memnew count: ";
    //mesh_count_log.log_textB = "memdelete count: ";

    svo = memnew(SparseVoxelOctree);
    init_debugMesh(svo->root, 1);
}

SvoNavmesh::~SvoNavmesh() {
    memdelete(svo);
    //reset_wastepool();
    //
    //mesh_count_log.test_print();
}

Vector3 SvoNavmesh::worldToGrid(Vector3 world_position) {
    // ��ŷ���ǴӶ�ת��Ϊ����
    Vector3 euler_angles_radians(
        Math::deg_to_rad(offset_rotation.x),
        Math::deg_to_rad(offset_rotation.y),
        Math::deg_to_rad(offset_rotation.z)
    );

    Transform3D global_transform = get_global_transform();
    Vector3 local_position = global_transform.xform_inv(world_position); // ȫ�ֵ��ֲ�
    local_position = Quaternion(euler_angles_radians).inverse().xform(local_position - offset_position); // Ӧ��ƫ�ƺ���ת
    return local_position / voxelSize; // ת������������
}
bool globalDepthCheck(int depth) {
    if (depth< global_min_depth || depth>global_max_depth) return false;
    else return true;
}

void SvoNavmesh::insert_voxel(Vector3 world_position) {
    Vector3 grid_position = worldToGrid(world_position);

    // ������������Ƿ���Ч(���ڸ��ڵ㷶Χ��)
    if (grid_position.x < -voxelSize / 2 || grid_position.y < -voxelSize / 2 || grid_position.z < -voxelSize / 2 ||
        grid_position.x >= voxelSize / 2 || grid_position.y >= voxelSize / 2 || grid_position.z >= voxelSize / 2) {
        WARN_PRINT("Voxel coordinates out of valid range.");
        return;
    }

    refresh_svo();
    svo->insert(grid_position);
    reset_pool();
    init_debugMesh(svo->root, 1);
}
bool SvoNavmesh::query_voxel(Vector3 world_position) {
    Vector3 grid_position = worldToGrid(world_position);
    return svo->query(grid_position);
}
void SvoNavmesh::update_voxel(Vector3 world_position, bool isSolid) {
    Vector3 grid_position = worldToGrid(world_position);

    // ������������Ƿ���Ч(���ڸ��ڵ㷶Χ��)
    if (grid_position.x < -voxelSize / 2 || grid_position.y < -voxelSize / 2 || grid_position.z < -voxelSize / 2 ||
        grid_position.x >= voxelSize / 2 || grid_position.y >= voxelSize / 2 || grid_position.z >= voxelSize / 2) {
        WARN_PRINT("Voxel coordinates out of valid range.");
        return;
    }

    refresh_svo();
    svo->update(grid_position, isSolid);
}

Vector3 SvoNavmesh::get_origin_position() const {
    return offset_position;
}
void SvoNavmesh::set_origin_position(Vector3 position) {
    offset_position = position;
}

Vector3 SvoNavmesh::get_origin_rotation() const {
    return offset_rotation;
}
void SvoNavmesh::set_origin_rotation(Vector3 rotation) {
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

// debug draw set/get
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

void SvoNavmesh::rebuild_svo() {
    svo->clear();
    reset_pool();
    init_debugMesh(svo->root, 1);
    // ���ݵ�ǰ�� voxelSize ���¹��� SVO
    // �߼�������֮ǰ�ᵽ���ؽ��߼�
    insert_svo_based_on_collision_shapes();
    reset_pool();
    init_debugMesh(svo->root, 1);
}
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
}
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
}

// <generate svo from collider>
void build_svo(SparseVoxelOctree* svo, const Vector<Vector3>& points) {
    for (int i = 0; i < points.size(); ++i) {
        svo->insert(points[i]);
    }
}
void SvoNavmesh::insert_svo_based_on_collision_shapes() {
    uint64_t begin = Time::get_singleton()->get_ticks_msec();

    Node* parent_node = get_parent();  // ��ȡ��ǰ�ڵ�ĸ��ڵ�
    RID space_rid = this->get_world_3d()->get_space();
    if (parent_node != nullptr) {
        collect_collision_shapes(parent_node, space_rid);
    }

    uint64_t end = Time::get_singleton()->get_ticks_msec();
    WARN_PRINT_ED(vformat("build svo with %d milliseconds", end - begin));
}
bool check_point_inside_mesh(Vector3 point, RID& space_rid, RID& target_rid) {
    // ��ȡ PhysicsDirectSpaceState3D ʵ��
    PhysicsDirectSpaceState3D* space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(space_rid);
    if (!space_state) {
        ERR_PRINT_ED("Failed to get PhysicsDirectSpaceState3D instance");
        return false;
    }

    // ���������ò�ѯ����
    Ref<PhysicsPointQueryParameters3D> query_params;
    query_params.instantiate();
    query_params->set_position(point);  // ���ò�ѯ���λ��

    // ִ�е��ڲ�ѯ
    Array results = space_state->intersect_point(query_params, 32);  // max_results ����Ϊ 32

    // �����ѯ���
    if (!results.is_empty()) {
        for (int i = 0; i < results.size(); ++i) {
            Dictionary result = results[i];
            // ��һ���������ȡ collider �����
            RID result_rid = result["rid"];
            if (result_rid == target_rid) {
                return true;
            }
        }
    }
    return false;
}
Vector<Vector3> sample_collision_shape(Ref<ArrayMesh> mesh, Vector3 position, float size, float sample_interval, RID& space_rid, RID& target_rid) {
    Vector<Vector3> sampled_points;
    
    // position is the center of svo, size is voxelSize
    for (float x = position.x - size / 2; x < position.x + size / 2; x += sample_interval) {
        for (float y = position.y - size / 2; y < position.y + size / 2; y += sample_interval) {
            for (float z = position.z - size / 2; z < position.z + size / 2; z += sample_interval) {
                Vector3 point = Vector3(x, y, z);

                // �����Ƿ��������ڲ�
                if (check_point_inside_mesh(point, space_rid, target_rid)) {
                    sampled_points.push_back(point);
                }
            }
        }
    }

    // �������񶥵�
    PackedVector3Array faces = mesh->get_faces();
    for (int i = 0; i < faces.size(); i++) {
        sampled_points.push_back(faces[i]);
    }

    return sampled_points;
}
Vector<Vector3> get_box_corners(CollisionShape3D* collision_shape) {    // off duty
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
Vector<Vector3> get_shape_points(CollisionShape3D* collision_shape, Vector3 position, float size, float sample_interval, RID& space_rid, RID& target_rid) {
    Vector<Vector3> points;
    Ref<Shape3D> shape = collision_shape->get_shape();

    if (shape.is_valid()) {
        Ref<ArrayMesh> debug_mesh = shape.ptr()->get_debug_mesh();
        points = sample_collision_shape(debug_mesh, position, size, sample_interval, space_rid, target_rid);
    }

    return points;
}
void SvoNavmesh::collect_collision_shapes(Node* node, RID& space_rid) {
    if (!node) return;

    // ���������е�ÿ���ڵ�
    for (int i = 0; i < node->get_child_count(); ++i) {
        Node* child = node->get_child(i);

        // ���Խ��ӽڵ�ת��ΪPhysicsBody3D
        PhysicsBody3D* physics_body = Object::cast_to<PhysicsBody3D>(child);
        if (physics_body) {
            // ����PhysicsBody3D���ӽڵ�������CollisionShape3D
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

        // �ݹ���ռ��ӽڵ����ײ��״
        collect_collision_shapes(child, space_rid);
    }
}
// </generate svo from collider>

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
    // ������С��������godot�ӽڵ㵼�µ��������
    //clear_mesh_instances();
    //draw_svo(svo->root, 1, 1, maxDepth);

    // �����ϴ󣬵�����ǣ�浽godot�ӽڵ��������
    /*for (MeshInstance3D* instance : mesh_pool) {
        remove_child(instance);
        memdelete(instance); // ȷ���ͷ��ڴ�
    }
    mesh_pool.clear();*/
    // ���Ͻ�����draw_svo_v1

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
    debugSolidMaterial->set_albedo(Color(0.2, 1.0, 0.2, 0.2));  // ��͸����ɫ
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
    // ִ�������������ͷ���Դ
    clear_static_material();

    //Node3D::_exit_tree();
}

// debug draw
//ǣ�浽svo�ṹ�仯��Ҫ�ֶ����ô˷���
void SvoNavmesh::reset_pool() {
    if (!mesh_pool.is_empty()) {
        for (MeshInstance3D* instance : mesh_pool) {
            if (instance->is_inside_tree()) {
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
void SvoNavmesh::reset_wastepool() {
    if (!waste_pool.is_empty()) {
        for (MeshInstance3D* instance : waste_pool) {
            instance->queue_free();
        }
        waste_pool.clear();
    }
}
//ǣ�浽svo�ṹ�仯��Ҫ�ֶ����ô˷���
void SvoNavmesh::init_debugMesh(OctreeNode* node, int depth)
{
    if (!node || depth > svo->maxDepth) return;  // �����ǰ��ȳ��������ȣ�ֹͣ�ݹ�

    float size = 2 * voxelSize / pow(2, depth);   // ���㵱ǰ��ȵ����سߴ�

    // ����Ƿ���ָ����ȷ�Χ��
    if (depth <= svo->maxDepth) {
        /*if (!node->debugMesh) {
            node->debugMesh = memnew(MeshInstance3D);
            mesh_count_log.test_countA++;
        }*/
        if (node->debugBoxMesh.is_null()) node->debugBoxMesh = Ref<BoxMesh>(memnew(BoxMesh));
        node->debugBoxMesh->set_size(Vector3(size, size, size));  // ���ú��Ӵ�С
        node->debugMesh.set_mesh(node->debugBoxMesh);

        // ���ò���
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

    // �ݹ�����ӽڵ�
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            init_debugMesh(node->children[i], depth + 1);
        }
    }
}
void SvoNavmesh::draw_svo_v2(OctreeNode* node, int current_depth, int min_depth, int max_depth) {
    if (!node || current_depth > svo->maxDepth) return;  // �����ǰ��ȳ��������ȣ�ֹͣ�ݹ�

    float size = 2 * voxelSize / pow(2, current_depth);   // ���㵱ǰ��ȵ����سߴ�

    // Ӧ�� offset_position �� offset_rotation
    Vector3 euler_angles_radians(
        Math::deg_to_rad(offset_rotation.x),
        Math::deg_to_rad(offset_rotation.y),
        Math::deg_to_rad(offset_rotation.z)
    );
    Transform3D local_transform = Transform3D(Quaternion(euler_angles_radians), offset_position);
    Vector3 global_pos = local_transform.xform(node->center);
    Transform3D global_transform = local_transform.translated(global_pos);


    // ����Ƿ���ָ����ȷ�Χ��
    if (current_depth >= min_depth && current_depth <= max_depth) {
        node->debugMesh.set_transform(global_transform);  // �������ص�ȫ�ֱ任������λ�ú���ת
        node->debugMesh.set_visible(true);
    }
    else {
        node->debugMesh.set_visible(false);
    }

    //TOFIX 
    //����EMPTY�ڵ㣬�޸�min_depthֻ��Ҷ�Ӳ���Ч������

    // �ݹ�����ӽڵ�
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            draw_svo_v2(node->children[i], current_depth + 1, min_depth, max_depth); // �ݹ�����ӽڵ�
        }
    }
}

void SvoNavmesh::draw_svo_v1(OctreeNode* node, int current_depth, int min_depth, int max_depth) {
    if (!node || current_depth > max_depth) return;  // �����ǰ��ȳ��������ȣ�ֹͣ�ݹ�

    float size = 2*voxelSize / pow(2, current_depth);   // ���㵱ǰ��ȵ����سߴ�

    // Ӧ�� offset_position �� offset_rotation
    Vector3 euler_angles_radians(
        Math::deg_to_rad(offset_rotation.x),
        Math::deg_to_rad(offset_rotation.y),
        Math::deg_to_rad(offset_rotation.z)
    );
    Transform3D local_transform = Transform3D(Quaternion(euler_angles_radians), offset_position);
    Vector3 global_pos = local_transform.xform(node->center);
    Transform3D global_transform = local_transform.translated(global_pos);


    // ����Ƿ���ָ����ȷ�Χ��
    if (current_depth >= min_depth && current_depth <= max_depth) {
        // ���� MeshInstance3D
            //С������
            //MeshInstance3D* mesh_instance = get_mesh_instance_from_pool();
            //active_meshes.push_back(mesh_instance);  // ���Ϊ��Ծ���Ա��������
            //������
            MeshInstance3D* mesh_instance = memnew(MeshInstance3D);
        // ���� BoxMesh
        Ref<BoxMesh> box_mesh = Ref<BoxMesh>(memnew(BoxMesh));
        box_mesh->set_size(Vector3(size, size, size));  // ���ú��Ӵ�С

        mesh_instance->set_mesh(box_mesh);
        mesh_instance->set_transform(global_transform);  // �������ص�ȫ�ֱ任������λ�ú���ת

        // ���ò���
        if (node->voxel && node->voxel->isSolid())
        {
            Ref<StandardMaterial3D> material = Ref<StandardMaterial3D>(memnew(StandardMaterial3D));
            material->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
            material->set_albedo(Color(0.2, 1.0, 0.2, 0.2));  // ��͸����ɫ
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

        //С������
        //if (!mesh_instance->get_parent()) add_child(mesh_instance);
        //mesh_instance->set_visible(true);

        //������
        add_child(mesh_instance);
        mesh_pool.push_back(mesh_instance);

    }

    if (!node->voxel->isLiquid()) {
        return;
    }

    // �ݹ�����ӽڵ�
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            draw_svo_v1(node->children[i], current_depth + 1, min_depth, max_depth); // �ݹ�����ӽڵ�
        }
    }
}
//С���������跽��������������draw_svo_v1
/*MeshInstance3D* SvoNavmesh::get_mesh_instance_from_pool() {
    if (mesh_pool.size() > 0) {
        MeshInstance3D* instance = mesh_pool[mesh_pool.size() - 1]; // ��ȡ���һ��Ԫ��
        mesh_pool.remove_at(mesh_pool.size() - 1); // �Ƴ����һ��Ԫ��
        return instance;
    }
    return memnew(MeshInstance3D); // �������û�п��ö��󣬴����µ�ʵ��
}
void SvoNavmesh::recycle_mesh_instance(MeshInstance3D* instance) {
    instance->set_visible(false);  // ����ʵ����������Ⱦ
    mesh_pool.push_back(instance);  // ��ʵ�����յ�����
}
void SvoNavmesh::clear_mesh_instances() {
    // ��ջ�Ծ�б�����������ʵ��
    for (int i = 0; i < active_meshes.size(); ++i) {
        recycle_mesh_instance(active_meshes[i]);
    }
    active_meshes.clear();
}*/