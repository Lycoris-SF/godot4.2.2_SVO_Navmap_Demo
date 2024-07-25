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
    static Ref<StandardMaterial3D> debugCheckMaterial;
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
        int maxDepth;
        float voxelSize;    // size of the root cube
        double testdouble;

        // <debug draw>
        bool debug_mode;
        bool node_ready;
        int DrawRef_minDepth;
        int DrawRef_maxDepth;
        bool show_empty;
        bool get_debug_mode() const;
        void set_debug_mode(bool debug_mode);
        int get_DR_min_depth() const;
        void set_DR_min_depth(int depth);
        int get_DR_max_depth() const;
        void set_DR_max_depth(int depth);
        bool get_show_empty() const;
        void set_show_empty(bool show_empty);
        OctreeNode* debugChecked_node;
        // v2
        Vector<MeshInstance3D*> mesh_pool;  // MeshInstance3D children in SvoNavmesh: Node3D
        Vector<Ref<BoxMesh>> waste_pool;    // BoxMesh to recycle
        // v3
        Vector<OctreeNode*> exist_meshes;   // Node of MeshInstance3D children in SvoNavmesh: Node3D
        Vector<OctreeNode*> active_meshes;  // Node of active MeshInstance list
        
        //
        Vector<Vector3> debug_path;
        Vector<MeshInstance3D*> path_pool;

        // v2
        void init_debug_mesh(OctreeNode* node, int depth);
        void reset_pool();
        void force_clear_debug_mesh();
        void draw_svo_v2(OctreeNode* node, int current_depth, int min_depth, int max_depth);
        // v3
        void init_debug_mesh_v3();
        void init_debug_mesh_v3(OctreeNode* node, int depth);
        void reset_pool_v3();
        void draw_svo_v3(OctreeNode* node, int current_depth, int min_depth, int max_depth);
        MeshInstance3D* get_mesh_instance_from_pool();

        //
        void init_debug_path(Vector<Vector3>& path, float agent_r);
        void reset_debugCheck();
        // </debug draw>

        // <neighbors>
        void init_neighbors();
        void set_neighbors(OctreeNode* node);
        void set_neighbors_from_brother(OctreeNode* node);
        // </neighbors>

        // path finding
        Vector<Vector3> find_raw_path(const Vector3 start, const Vector3 end, float agent_r);

        // tools
        Vector3 worldToGrid(Vector3 world_position);
        Vector3 gridToWorld(Vector3 grid_position);
        void collect_collision_shapes(Node* node, RID &space_rid);
        void traverse_svo_space_and_insert(OctreeNode* node, int depth, RID& space_rid);
        bool can_travel_directly_with_cylinder(const Vector3& from, const Vector3& to, float agent_radius, RID& space_rid);
        bool can_travel_directly_with_ray(const Vector3& from, const Vector3& to, RID& space_rid);
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
        void check_voxel_with_id(String id);
        void update_voxel(Vector3 position, bool isSolid);

        //svo setting
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
        Array find_path(const Vector3 start, const Vector3 end, float agent_r, bool is_smooth = true);

        // override
        void _ready();
        void _process(double delta);
        void _physics_process(double delta);
        void _enter_tree();
        void _exit_tree();
    };

}

#endif // SVO_NAVMESH_H