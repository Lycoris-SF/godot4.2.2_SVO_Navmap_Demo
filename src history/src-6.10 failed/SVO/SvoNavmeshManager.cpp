#include "SvoNavmeshManager.h"

using namespace godot;

SvoNavmeshManager::SvoNavmeshManager() {}

SvoNavmeshManager::~SvoNavmeshManager() {
    for (auto navmesh : navmeshes) {
        // �������navmeshes������������Godot������˲�ɾ��navmeshָ��
    }
}

void SvoNavmeshManager::add_navmesh(SvoNavmesh* navmesh) {
    //navmeshes.push_back(navmesh);
}

bool SvoNavmeshManager::query(Vector3 position) const {
    /*for (auto navmesh : navmeshes) {
        if (navmesh->query_voxel(position)) {
            return true;
        }
    }*/
    return false;
}

SvoNavmesh* SvoNavmeshManager::find_navmesh_containing(Vector3 position) const {
    /*for (auto navmesh : navmeshes) {
        if (navmesh->query_voxel(position)) {
            return navmesh;
        }
    }*/
    return nullptr;
}

Vector<Vector3> SvoNavmeshManager::find_path(Vector3 start, Vector3 end) const {
    /*SvoNavmesh* start_navmesh = find_navmesh_containing(start);
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

    return path;*/
    return {};
}
