cmake_minimum_required(VERSION 3.16)
project(arms_planner CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wno-deprecated-declarations)

find_package(OpenCASCADE REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(coal REQUIRED)
find_package(Boost REQUIRED COMPONENTS json)

# ---- arms_planner_core_lib (no ROS, no PIC restriction) ----
add_library(arms_planner_core_lib STATIC
  src/Assembly.cpp
  src/Assembler.cpp
  src/Part.cpp
  src/ModelLoader.cpp
  src/CradleGenerator.cpp
  src/AdvancedCradleGenerator.cpp
  src/TestCradleGenerator.cpp
  src/SparseCradleGenerator.cpp
  src/GCodeGenerator.cpp
  src/JigGenerator.cpp
  src/SparseJigGenerator.cpp
  src/PPGGraspGenerator.cpp
  src/VacuumGraspGenerator.cpp
  src/RaftGenerator.cpp
  src/MeshFunctions.cpp
  src/ARMSConfig.cpp
  src/Config.cpp
  src/Tessellator.cpp
  src/SceneModel.cpp
  src/CoalAdapter.cpp
  src/GlbWriter.cpp
  include/assembler/quickhull/QuickHull.cpp
)

target_include_directories(arms_planner_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCASCADE_INCLUDE_DIR}
  ${OpenCASCADE_INCLUDE_DIRS}
)

target_link_libraries(arms_planner_core_lib PUBLIC
  ${OpenCASCADE_LIBRARIES}
  yaml-cpp::yaml-cpp
  coal::coal
)

# Required when the static lib is linked into a shared object (assembler_node.so)
set_property(TARGET arms_planner_core_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

# ---- arms-plan executable ----
add_executable(arms-plan src/main.cpp)

target_link_libraries(arms-plan PRIVATE
  arms_planner_core_lib
  Boost::json
  minizip
)

# Default slicer config directory: arms_planner/config/ inside this source tree.
# Override at configure time with -DARMS_PLANNER_CONFIG_DIR=/your/path
set(ARMS_PLANNER_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config"
    CACHE PATH "Directory containing arms_prusa_config.ini")
target_compile_definitions(arms-plan PRIVATE
  "ARMS_PLANNER_CONFIG_DIR=\"${ARMS_PLANNER_CONFIG_DIR}\"")

install(TARGETS arms-plan RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/arms_planner/config)
