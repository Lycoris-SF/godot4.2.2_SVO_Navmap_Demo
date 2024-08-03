#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

#include "SVO/svo_navmesh.h"
#include "SVO/svo_navmesh_manager.h"
using namespace godot;


void initialize_svo_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    ClassDB::register_class<Voxel>();
    ClassDB::register_class<OctreeNode>();
    ClassDB::register_class<SparseVoxelOctree>();
    ClassDB::register_class<SvoNavmesh>();          //main
    ClassDB::register_class<SvoNavmeshManager>();
    
}

void uninitialize_svo_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {
    // Initialization.
    GDExtensionBool GDE_EXPORT library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization* r_initialization) {
        godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

        init_obj.register_initializer(initialize_svo_module);
        init_obj.register_terminator(uninitialize_svo_module);
        init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

        return init_obj.init();
    }
}