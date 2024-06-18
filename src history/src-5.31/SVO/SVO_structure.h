#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <vector>

class Voxel {
public:
    bool isSolid;
    float size;
    Voxel(bool solid = false, float voxel_size = 1.0f) : isSolid(solid), size(voxel_size) {}
};

class OctreeNode {
public:
    bool isLeaf;
    bool isHomogeneous; // 表示该节点是否为同质节点
    Voxel* voxel;
    OctreeNode* children[8];
    OctreeNode* neighbors[6];

    OctreeNode();
    ~OctreeNode();
    bool isEmpty() const;
};

namespace godot {

    class SparseVoxelOctree {
    public:
        float voxelSize;
        int maxDepth;
        Vector3 origin;
        OctreeNode* root;

        SparseVoxelOctree(float voxel_size, int max_depth, Vector3 origin = Vector3(0.0f, 0.0f, 0.0f));
        ~SparseVoxelOctree();

        void insert(float x, float y, float z);
        bool query(float x, float y, float z) const;
        void update(float x, float y, float z, bool isSolid);
        //std::vector<Vector3> find_path(Vector3 start, Vector3 end) const;

    private:
        void insert(OctreeNode* node, int x, int y, int z, int depth);
        bool query(OctreeNode* node, int x, int y, int z, int depth) const;
        void update(OctreeNode* node, int x, int y, int z, int depth, bool isSolid);

        Vector3 worldToGrid(float x, float y, float z) const;
        //std::vector<Vector3> reconstruct_path(const std::map<Vector3, Vector3>& came_from, Vector3 current) const;
    };

}