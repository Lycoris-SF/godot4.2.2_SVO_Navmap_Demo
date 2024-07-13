#ifndef SVO_NAVMESH_H
#define SVO_NAVMESH_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <godot_cpp/classes/physics_body3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_point_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_shape_query_parameters3d.hpp>
#include <godot_cpp/classes/world3d.hpp>

#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/shader_material.hpp>

#include "svo_structure.h"
#include "Test/test_svo.h"

namespace godot {
    static Ref<StandardMaterial3D> debugSolidMaterial;
    static Ref<ShaderMaterial> debugEmptyMaterial;
    static Ref<Shader> EmptyMaterial_shader;
    // PhysicsBody3Ds' RID
    static Vector<RID> target_rids;
    // Test only
    static TestLog mesh_count_log;

    class SvoNavmesh : public Node3D {
        GDCLASS(SvoNavmesh, Node3D)

    private:
        SparseVoxelOctree *svo;
        Vector3 offset_position; // Offset position in voxel space
        Vector3 offset_rotation; // Offset rotation in voxel space
        int maxDepth;
        float voxelSize;    // size of the root cube
        double testdouble;

        // <debug draw>
        int DrawRef_minDepth;
        int DrawRef_maxDepth;
        int get_DR_min_depth() const;
        void set_DR_min_depth(int depth);
        int get_DR_max_depth() const;
        void set_DR_max_depth(int depth);
        Vector<MeshInstance3D*> mesh_pool;  // MeshInstance3D children in SvoNavmesh: Node3D
        Vector<MeshInstance3D*> waste_pool;  // MeshInstance3D children to recycle
        Vector<Vector3> debug_path;
        Vector<MeshInstance3D*> path_pool;
        //Vector<MeshInstance3D*> active_meshes;  // active MeshInstance list

        void init_debug_mesh(OctreeNode* node, int depth);
        void init_debug_path(const Vector<Vector3>& path, float agent_r);
        void reset_pool();
        void reset_wastepool();
        void draw_svo_v2(OctreeNode* node, int current_depth, int min_depth, int max_depth);
        void draw_svo_v1(OctreeNode* node, int current_depth, int min_depth, int max_depth);
        //MeshInstance3D* get_mesh_instance_from_pool();
        //void recycle_mesh_instance(MeshInstance3D* instance);
        //void clear_mesh_instances();
        // </debug draw>

        // <neighbors>
        void init_neighbors();
        void set_neighbors(OctreeNode* node);
        void set_neighbors_from_brother(OctreeNode* node);
        // </neighbors>

        // tools
        Vector3 worldToGrid(Vector3 world_position);
        Vector3 gridToWorld(Vector3 grid_position);
        void collect_collision_shapes(Node* node, RID &space_rid);
        bool can_traverse_directly_with_cylinder(const Vector3& from, const Vector3& to, float agent_radius, RID& space_rid);
        bool can_traverse_directly_with_ray(const Vector3& from, const Vector3& to, RID& space_rid);
        Vector<Vector3> smooth_path_string_pulling_fast(const Vector<Vector3>& path, float agent_radius, RID& space_rid);
        Vector<Vector3> smooth_path_string_pulling_best(const Vector<Vector3>& path, float agent_radius, RID& space_rid);

    protected:
        static void _bind_methods();

    public:
        SvoNavmesh();
        //SvoNavmesh(float size, Vector3 position, Vector3 rotation);
        ~SvoNavmesh();

        void insert_voxel(Vector3 position);
        bool query_voxel(Vector3 position);
        void update_voxel(Vector3 position, bool isSolid);

        //svo setting
        Vector3 get_offset_position() const;
        void set_offset_position(Vector3 position);
        Vector3 get_offset_rotation() const;
        void set_offset_rotation(Vector3 rotation);
        float get_voxel_size() const;
        void set_voxel_size(float size);
        int get_max_depth() const;
        void set_max_depth(int depth);
        SparseVoxelOctree& get_svo();

        // tools
        void rebuild_svo();
        void refresh_svo();
        void clear_svo(bool clear_setting);
        void insert_svo_based_on_collision_shapes();

        // bind test
        void set_info(float p_info);
        float get_info();

        // path finding
        void find_path_and_draw(const Vector3 start, const Vector3 end, float agent_r);
        Vector<Vector3> find_path(const Vector3 start, const Vector3 end, float agent_r);

        // override
        void _process(double delta);
        void _physics_process(double delta);
        void _enter_tree();
        void _exit_tree();
    };

}

#endif // SVO_NAVMESH_H