#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/templates/vector.hpp>

#include "SVO_structure.h"

namespace godot {

    class SvoNavmesh : public Node3D {
        GDCLASS(SvoNavmesh, Node3D)

    private:
        SparseVoxelOctree *svo;
        Vector3 offset_position; // 体素空间的相对位置
        Vector3 offset_rotation; // 体素空间的相对旋转
        int maxDepth;
        float voxelSize; // 体素大小
        double testdouble;

        Vector3 worldToGrid(Vector3 world_position);

        // debug draw
        int DrawRef_minDepth;
        int DrawRef_maxDepth;
        int get_DR_min_depth() const;
        void set_DR_min_depth(int depth);
        int get_DR_max_depth() const;
        void set_DR_max_depth(int depth);
        Vector<MeshInstance3D*> mesh_pool;  // 对象池
        //Vector<MeshInstance3D*> active_meshes;  // 活跃的 MeshInstance 列表

        void init_debugMesh(OctreeNode* node, int depth);
        void reset_pool();
        void draw_svo_v2(OctreeNode* node, int current_depth, int min_depth, int max_depth);
        void draw_svo_v1(OctreeNode* node, int current_depth, int min_depth, int max_depth);
        //MeshInstance3D* get_mesh_instance_from_pool();
        //void recycle_mesh_instance(MeshInstance3D* instance);
        //void clear_mesh_instances();

    protected:
        static void _bind_methods();

    public:
        SvoNavmesh();
        SvoNavmesh(float size, Vector3 position, Vector3 rotation);
        ~SvoNavmesh();

        void insert_voxel(Vector3 position);
        bool query_voxel(Vector3 position);
        void update_voxel(Vector3 position, bool isSolid);

        //svo setting
        Vector3 get_origin_position() const;
        void set_origin_position(Vector3 position);
        Vector3 get_origin_rotation() const;
        void set_origin_rotation(Vector3 rotation);
        float get_voxel_size() const;
        void set_voxel_size(float size);
        int get_max_depth() const;
        void set_max_depth(int depth);
        SparseVoxelOctree& get_svo();
        void rebuild_svo();
        void refresh_svo();
        void clear_svo(bool clear_setting);

        //bind test
        void set_info(float p_info);
        float get_info();

        void _process(double delta);
    };

}