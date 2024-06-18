#include "SvoNavmesh.h"
#include "SVO_structure.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void SvoNavmesh::_bind_methods() {
    ClassDB::bind_method(D_METHOD("insert_voxel", "position"), &SvoNavmesh::insert_voxel);
    ClassDB::bind_method(D_METHOD("query_voxel", "position"), &SvoNavmesh::query_voxel);
    ClassDB::bind_method(D_METHOD("get_info"), &SvoNavmesh::get_info);
    ClassDB::bind_method(D_METHOD("set_info", "p_info"), &SvoNavmesh::set_info);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "testdouble"), "set_info", "get_info");
    //ClassDB::bind_method(D_METHOD("find_path", "start", "end"), &SvoNavmesh::find_path);

    /*ClassDB::bind_method(D_METHOD("get_origin_position"), &SvoNavmesh::get_origin_position);
    ClassDB::bind_method(D_METHOD("set_origin_position", "position"), &SvoNavmesh::set_origin_position);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::VECTOR3, "origin_position"), "set_origin_position", "get_origin_position");

    //ClassDB::bind_method(D_METHOD("get_origin_rotation"), &SvoNavmesh::get_origin_rotation);
    //ClassDB::bind_method(D_METHOD("set_origin_rotation", "rotation"), &SvoNavmesh::set_origin_rotation);
    //ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::QUATERNION, "origin_rotation"), "set_origin_rotation", "get_origin_rotation");

    ClassDB::bind_method(D_METHOD("get_voxel_size"), &SvoNavmesh::get_voxel_size);
    ClassDB::bind_method(D_METHOD("set_voxel_size", "size"), &SvoNavmesh::set_voxel_size);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "voxel_size"), "set_voxel_size", "get_voxel_size");
    
    ClassDB::bind_method(D_METHOD("rebuild_svo"), &SvoNavmesh::rebuild_svo);*/
}

SvoNavmesh::SvoNavmesh(){
    testdouble = 1.14;
    voxelSize = 1.0f;
    origin_position = Vector3(0, 0, 0);
    origin_rotation = Quaternion();
    // 初始化任何变量
}

SvoNavmesh::SvoNavmesh(float size, Vector3 position, Quaternion rotation)
{
    testdouble = 1.14;
    voxelSize = size;
    origin_position = position;
    origin_rotation = rotation;
}

SvoNavmesh::~SvoNavmesh() {
    // 添加清理代码
}

Vector3 SvoNavmesh::worldToGrid(Vector3 world_position) {
    // 将世界坐标转换为局部坐标
    Vector3 local_position = origin_rotation.inverse().xform(world_position - origin_position);
    // 进一步处理以转换到网格坐标
    return local_position / voxelSize;
}

void SvoNavmesh::insert_voxel(Vector3 world_position) {
    Vector3 grid_position = worldToGrid(world_position);
    svo.insert(grid_position.x, grid_position.y, grid_position.z);
}
bool SvoNavmesh::query_voxel(Vector3 world_position) {
    Vector3 grid_position = worldToGrid(world_position);
    return svo.query(grid_position.x, grid_position.y, grid_position.z);
}

Vector3 SvoNavmesh::get_origin_position() const {
    return origin_position;
}

void SvoNavmesh::set_origin_position(Vector3 position) {
    origin_position = position;
}

Quaternion SvoNavmesh::get_origin_rotation() const {
    return origin_rotation;
}

void SvoNavmesh::set_origin_rotation(Quaternion rotation) {
    origin_rotation = rotation;
}

float SvoNavmesh::get_voxel_size() const {
    return voxelSize;
}

void SvoNavmesh::set_voxel_size(float size) {
    voxelSize = size;
}

void SvoNavmesh::rebuild_svo() {
    svo.clear();  
    // 根据当前的 voxelSize 重新构建 SVO
    // 逻辑类似于之前提到的重建逻辑
}

SparseVoxelOctree& SvoNavmesh::get_svo() {
    return svo;
}

void SvoNavmesh::set_info(float p_info) {
    testdouble = p_info;
}
float SvoNavmesh::get_info(){
    return testdouble;
}

void SvoNavmesh::_process(double delta) {
    // 实现处理方法，如果需要的话
}