#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/shader_material.hpp>

namespace godot {
    enum VoxelState {
        VS_EMPTY,
        VS_SOLID,
        VS_LIQUID
    };

    class Voxel {
    public:
        VoxelState state;
        float size;
        Voxel();
        Voxel(VoxelState state, float voxel_size);
        bool isSolid();
        bool isEmpty();
        bool isLiquid();
    };

    class OctreeNode {
    public:
        bool isLeaf;        // 表示该节点是否为叶子节点
        int currentDepth;
        int currentIndex;
        Vector3 center;
        Voxel* voxel;
        OctreeNode* father;
        OctreeNode* children[8];
        OctreeNode* neighbors[6];

        // debug draw
        MeshInstance3D* debugMesh;
        Ref<BoxMesh> debugBoxMesh;
        static Ref<StandardMaterial3D> debugSolidMaterial;
        static Ref<ShaderMaterial> debugEmptyMaterial;
        static Ref<Shader> EmptyMaterial_shader;

        OctreeNode(OctreeNode* father_node, int depth = -1, int index = -1);
        ~OctreeNode();
    };

    class SparseVoxelOctree {
    public:
        int maxDepth;
        float voxelSize;
        OctreeNode* root;

        SparseVoxelOctree();
        SparseVoxelOctree(int max_depth, float voxelSize);
        ~SparseVoxelOctree();

        void insert(Vector3 pos);
        bool query(Vector3 pos) const;
        void update(Vector3 pos, bool isSolid);
        void update_node_centers();
        void compress_node();
        void expand_node();
        void clear();

    private:
        void insert(OctreeNode* node, Vector3 pos, Vector3 center, int depth);
        bool query(OctreeNode* node, Vector3 pos, Vector3 center, int depth) const;
        void update(OctreeNode* node, Vector3 pos, Vector3 center, int depth, VoxelState newState);
        // functional update
        void update_node_centers(OctreeNode* node, Vector3 center, int depth);
        void compress_node(OctreeNode* node, int depth);
        void expand_node(OctreeNode* node, int depth);
        void reevaluate_homogeneity_insert(OctreeNode* node);
        void reevaluate_homogeneity_update(OctreeNode* node);
        // delete
        void deleteChildren(OctreeNode* node);
        void deleteNode(OctreeNode* node);  // 递归删除节点
        // other tool
        float calActualVoxelSize(int depth); // 计算实际体素大小
        void merge_children(OctreeNode* node);
        void create_empty_children(OctreeNode* node, int depth);
        void create_solid_children(OctreeNode* node, int depth);
    };

}
