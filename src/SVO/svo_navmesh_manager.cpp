#include "svo_navmesh_manager.h"

using namespace godot;

void SvoNavmeshManager::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("acquire_navmesh"), &SvoNavmeshManager::acquire_navmesh);
    ClassDB::bind_method(D_METHOD("tidy_adjacents"), &SvoNavmeshManager::tidy_adjacents);
}

SvoNavmeshManager::SvoNavmeshManager()
{

}
SvoNavmeshManager::~SvoNavmeshManager() {
    // ����SvoNavmesh�����볡��������SvoNavmeshManager������ܻ���ɻ���
}

void SvoNavmeshManager::acquire_navmesh(SvoNavmesh* navmesh) {
    navmeshes[navmesh->uuid] = navmesh;
}

bool SvoNavmeshManager::query(Vector3 position) const {
    Array keys = navmeshes.keys();
    for (int i = 0; i < keys.size(); i++) {
        SvoNavmesh* navmesh = Object::cast_to<SvoNavmesh>(navmeshes[keys[i]]);
        if (navmesh && navmesh->query_voxel(position)) {
            return true;
        }
    }
    return false;
}

SvoNavmesh* SvoNavmeshManager::find_navmesh_containing(Vector3 position) const {
    Array keys = navmeshes.keys();
    for (int i = 0; i < keys.size(); i++) {
        SvoNavmesh* navmesh = Object::cast_to<SvoNavmesh>(navmeshes[keys[i]]);
        if (navmesh && navmesh->query_voxel(position)) {
            return navmesh;
        }
    }
    return nullptr;
}

Vector<Vector3> SvoNavmeshManager::find_path(Vector3 start, Vector3 end) const {
    SvoNavmesh* start_navmesh = find_navmesh_containing(start);
    SvoNavmesh* end_navmesh = find_navmesh_containing(end);

    if (!start_navmesh || !end_navmesh) {
        return {}; // ��������յ㲻���κ�һ��Navmesh�У��򷵻ؿ�·��
    }

    if (start_navmesh == end_navmesh) {
        // ��������յ���ͬһ��Navmesh�У�ֱ���ڸ�Navmesh��Ѱ·
        // ��Ҫʵ��SparseVoxelOctree::findPath����
        //return start_navmesh->get_svo().find_path(start, end);
    }

    // ������Ҫ�������Navmesh��Ѱ·
    // �򻯴����ҵ�����Navmesh֮������ӵ㣬����A*������·���滮
    Vector<Vector3> path;

    // ����ֻ��һ��ʾ����ʵ��ʵ�ֻ���Ӹ���
    path.push_back(start);
    path.push_back(Vector3(0, 0, 0)); // Placeholder for boundary point
    path.push_back(end);

    return path;
}

void SvoNavmeshManager::tidy_adjacents() {
    auto roots = find_possible_roots();
    HashSet<String> visited;

    for (int i = 0; i < roots.size(); i++) {
        String root = roots[i];
        if (!visited.has(root)) {  // ʹ�� .has() �������Ԫ���Ƿ����
            bfs(root, visited);
        }
    }
}

void SvoNavmeshManager::bfs(String start_uuid, HashSet<String>& visited) {
    std::queue<String> queue;
    queue.push(start_uuid);
    visited.insert(start_uuid);

    while (!queue.empty()) {
        String current_uuid = queue.front();
        queue.pop();

        // ��ȫ�ػ�ȡ��ǰ navmesh
        SvoNavmesh* current_navmesh = Object::cast_to<SvoNavmesh>(navmeshes[current_uuid]);
        if (!current_navmesh) continue;  // ���ת��ʧ�ܻ�navmesh�����ڣ�������

        Array adjacent_uuids = current_navmesh->adjacent_uuids;
        UtilityFunctions::print("Processing: " + current_uuid);
        Transform3D current_transform = current_navmesh->get_transform();
        float voxel_size = current_navmesh->get_voxel_size();

        for (int i = 0; i < adjacent_uuids.size(); i++) {
            Variant adj_uuid_var = adjacent_uuids[i];

            // ��� UUID �Ƿ�Ϊ��Ч���ַ���
            if (adj_uuid_var.get_type() == Variant::STRING) {
                String adj_uuid = adj_uuid_var;

                // ��� UUID �ַ����Ƿ�Ϊ�ղ��Ҹ� UUID �Ƿ��ѱ�����
                if (!adj_uuid.is_empty() && !visited.has(adj_uuid)) {
                    // ��ȷ�ػ�ȡ���ڵ� navmesh
                    SvoNavmesh* adj_navmesh = Object::cast_to<SvoNavmesh>(navmeshes[adj_uuid]);
                    if (!adj_navmesh) continue;  // ���ת��ʧ�ܻ�navmesh�����ڣ�������

                    visited.insert(adj_uuid);
                    queue.push(adj_uuid);

                    // ����λ��ƫ��
                    Vector3 local_offset;
                    switch (i) {
                    case 0: local_offset = Vector3(voxel_size, 0, 0); break;
                    case 1: local_offset = Vector3(-voxel_size, 0, 0); break;
                    case 2: local_offset = Vector3(0, voxel_size, 0); break;
                    case 3: local_offset = Vector3(0, -voxel_size, 0); break;
                    case 4: local_offset = Vector3(0, 0, voxel_size); break;
                    case 5: local_offset = Vector3(0, 0, -voxel_size); break;
                    }

                    // ���ֲ�ƫ��ת��Ϊȫ��ƫ��
                    Vector3 global_offset = current_transform.basis.xform(local_offset);
                    Transform3D new_transform = Transform3D(current_transform.basis, current_transform.origin + global_offset);
                    adj_navmesh->set_transform(new_transform);
                    adj_navmesh->set_voxel_size(current_navmesh->get_voxel_size());
                    adj_navmesh->refresh_svo();
                    UtilityFunctions::print(vformat("Adjust SVO: " + adj_uuid));
                }
            }
        }
    }
}

Array SvoNavmeshManager::find_possible_roots() {
    // ������SvoNavmesh UUID��ӵ�Ǳ�ڸ�������
    Array keys = navmeshes.keys();
    Array potential_roots = keys.duplicate();

    // ��Ǳ�ڸ��������Ƴ��κα����õ�UUID
    for (int i = 0; i < keys.size(); i++) {
        Array adjacent_uuids = Object::cast_to<SvoNavmesh>(navmeshes[keys[i]])->adjacent_uuids;
        for (int j = 0; j < adjacent_uuids.size(); j++) {
            Variant adj_uuid_var = adjacent_uuids[j];
            // ���adj_uuid_var�Ƿ�Ϊ��Ч��UUID
            if (adj_uuid_var.get_type() == Variant::STRING) {
                String adj_uuid = adj_uuid_var;
                if (!adj_uuid.is_empty() && navmeshes.has(adj_uuid)) {
                    potential_roots.erase(adj_uuid);
                }
            }
        }
    }
    return potential_roots;
}