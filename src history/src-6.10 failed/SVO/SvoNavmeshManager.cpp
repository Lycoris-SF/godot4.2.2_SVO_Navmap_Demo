#include "SvoNavmeshManager.h"

using namespace godot;

SvoNavmeshManager::SvoNavmeshManager() {}

SvoNavmeshManager::~SvoNavmeshManager() {
    for (auto navmesh : navmeshes) {
        // 这里假设navmeshes的生命周期由Godot管理，因此不删除navmesh指针
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
        return {}; // 如果起点或终点不在任何一个Navmesh中，则返回空路径
    }

    if (start_navmesh == end_navmesh) {
        // 如果起点和终点在同一个Navmesh中，直接在该Navmesh中寻路
        // 需要实现SparseVoxelOctree::findPath方法
        //return start_navmesh->get_svo().find_path(start, end);
    }

    // 否则，需要处理跨多个Navmesh的寻路
    // 简化处理：找到相邻Navmesh之间的连接点，进行A*或其他路径规划
    Vector<Vector3> path;

    // 这里只是一个示例，实际实现会更加复杂
    path.push_back(start);
    path.push_back(Vector3(0, 0, 0)); // Placeholder for boundary point
    path.push_back(end);

    return path;*/
    return {};
}
