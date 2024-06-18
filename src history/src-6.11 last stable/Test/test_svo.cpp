#include "test_svo.h"
#include "SVO/SVO_structure.h"

using namespace godot;

int main() {

    SparseVoxelOctree svo_test;  // 创建对象实例

    // 在这里添加测试代码
    // 例如：svo.insert(x, y, z); 或者 svo.query(x, y, z);
    svo_test.insert(1, 1, 1);

    return 0;
}