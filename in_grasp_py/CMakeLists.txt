cmake_minimum_required(VERSION 3.18)
project(in_grasp LANGUAGES CXX)

# Placeholder. M2 (see ../MIGRATION.md) adds:
#   - vendored ROS-free ll4ma pieces under cpp/third_party/ll4ma
#   - the planner shell (cpp/planner.cpp) + pybind11 bindings
#   - add_variant(<name> <method .cpp>) building one extension per method variant,
#     compiling the method sources unmodified from ../in_grasp_manipulation/src/problems/
