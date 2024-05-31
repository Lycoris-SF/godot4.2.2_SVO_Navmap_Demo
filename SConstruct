#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

# For reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

# 设置包含路径，包括src目录及其所有子目录
src_path = "./src"
include_paths = []
for root, dirs, files in os.walk(src_path):
    include_paths.append(root)

env.Append(CPPPATH=include_paths)

# 获取src目录及其所有子目录中的所有cpp文件
sources = []
for root, dirs, files in os.walk(src_path):
    sources.extend([os.path.join(root, file) for file in files if file.endswith(".cpp")])

# 定义变量
output_dir = "godot-project"
library_name = "libgdexample"

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "{}/bin/{}.{}.{}.framework/{}.{}/{}".format(
            output_dir, library_name, env["platform"], env["target"], library_name, env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "{}/bin/{}{}{}".format(output_dir, library_name, env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)