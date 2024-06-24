#include "SvoNavmesh.h"
#include "SVO_structure.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/shader_material.hpp>
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
}

SvoNavmesh::SvoNavmesh(){
    offset_position = Vector3(0, 0, 0);
    offset_rotation = Vector3(0, 0, 0);
    maxDepth = 3;
    voxelSize = 1.0f;
    testdouble = 1.14;

    DrawRef_minDepth = 1;
    DrawRef_maxDepth = 3;

    svo = new SparseVoxelOctree();
}

SvoNavmesh::~SvoNavmesh() {
    delete svo; // ����̬������ڴ�
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
        ERR_PRINT("Voxel coordinates out of valid range.");
        return;
    }

    refresh_svo();
    svo->insert(grid_position);
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
        ERR_PRINT("Voxel coordinates out of valid range.");
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
        ERR_PRINT("Depth out of setting range.(1-9)");
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
        ERR_PRINT("Depth out of setting range.(1-9)");
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
        ERR_PRINT("Depth out of setting range.(1-9)");
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
    // ���ݵ�ǰ�� voxelSize ���¹��� SVO
    // �߼�������֮ǰ�ᵽ���ؽ��߼�
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

void SvoNavmesh::_process(double delta) {
    // ������С��������godot�ӽڵ㵼�µ��������
    //clear_mesh_instances();
    //draw_svo(svo->root, 1, 1, maxDepth);

    // �����ϴ󣬵�����ǣ�浽godot�ӽڵ��������
    for (MeshInstance3D* instance : mesh_pool) {
        remove_child(instance);
        memdelete(instance); // ȷ���ͷ��ڴ�
    }
    mesh_pool.clear();
    draw_svo(svo->root, 1, DrawRef_minDepth, DrawRef_maxDepth); // ���ƴӸ��ڵ㵽Ҷ�ڵ��ȫ����
}

void SvoNavmesh::draw_svo(OctreeNode* node, int current_depth, int min_depth, int max_depth) {
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
        Ref<StandardMaterial3D> material = Ref<StandardMaterial3D>(memnew(StandardMaterial3D));
        if (node->voxel && node->voxel->isSolid())
        {
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

    if (node->isHomogeneous) {
        return;
    }

    // �ݹ�����ӽڵ�
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            draw_svo(node->children[i], current_depth + 1, min_depth, max_depth); // �ݹ�����ӽڵ�
        }
    }
}
//С���������跽������
MeshInstance3D* SvoNavmesh::get_mesh_instance_from_pool() {
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
}
