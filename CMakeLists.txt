cmake_minimum_required(VERSION 3.1)
project(QuadriFlow LANGUAGES CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_DEBUG_POSTFIX "-debug")
include(CMakePackageConfigHelpers)

option(BUILD_PERFORMANCE_TEST "More subdivisition for performance test" OFF)
option(BUILD_LOG "Enable verbose log" OFF)
option(BUILD_GUROBI "Enable GUROBI for comparison ONLY" OFF)
option(BUILD_OPENMP "Enable support for OpenMP" OFF)
option(BUILD_TBB "Enable support for TBB" OFF)
option(BUILD_FREE_LICENSE "Only use libraries with permissive licenses" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

if (BUILD_GUROBI)
    find_package(GUROBI REQUIRED)
endif(BUILD_GUROBI)

if (BUILD_TBB)
    find_package(TBB REQUIRED)
endif(BUILD_TBB)

set(LEMON_3RD_PATH 3rd/lemon-1.3.1)

find_path(
    LEMON_SOURCE_ROOT_DIR
    CMakeLists.txt
    PATHS ${CMAKE_SOURCE_DIR}/${LEMON_3RD_PATH} NO_DEFAULT_PATH
    DOC "Location of LEMON source as a CMAKE subproject"
)

if (EXISTS ${LEMON_SOURCE_ROOT_DIR})
    add_subdirectory(${LEMON_SOURCE_ROOT_DIR})
    set(LEMON_INCLUDE_DIRS ${LEMON_SOURCE_ROOT_DIR} ${PROJECT_BINARY_DIR}/${LEMON_3RD_PATH})
    set(LEMON_LIBRARIES lemon)
    unset(LEMON_ROOT_DIR CACHE)
    unset(LEMON_DIR CACHE)
    unset(LEMON_INCLUDE_DIR CACHE)
    unset(LEMON_LIBRARY CACHE)
else()
    find_package(LEMON QUIET NO_MODULE)
    find_package(LEMON REQUIRED)
endif()
set_target_properties(lemon PROPERTIES FOLDER "libraries")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(${BUILD_OPENMP})
	find_package(OpenMP REQUIRED)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(src)
include_directories(3rd/pcg32)
include_directories(3rd/pss)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${GLM_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${LEMON_INCLUDE_DIRS})
include_directories(${TBB_INCLUDE_DIRS})
include_directories(${GUROBI_INCLUDE_DIRS})

if (BUILD_PERFORMANCE_TEST)
    add_definitions(-DPERFORMANCE_TEST)
endif(BUILD_PERFORMANCE_TEST)

if (BUILD_OPENMP)
    add_definitions(-DWITH_OMP)
endif(BUILD_OPENMP)

if (BUILD_LOG)
    add_definitions(-DLOG_OUTPUT)
endif(BUILD_LOG)

if (BUILD_GUROBI)
    add_definitions(-DWITH_GUROBI)
endif(BUILD_GUROBI)

if (BUILD_TBB)
    add_definitions(-DWITH_TBB)
endif(BUILD_TBB)

if (BUILD_FREE_LICENSE)
    add_definitions(-DEIGEN_MPL2_ONLY)
endif(BUILD_FREE_LICENSE)

add_library(libQuadriflow STATIC
    src/adjacent-matrix.cpp
    include/quadriflow/adjacent-matrix.hpp
    include/quadriflow/compare-key.hpp
    include/quadriflow/config.hpp
    src/dedge.cpp
    include/quadriflow/dedge.hpp
    include/quadriflow/disajoint-tree.hpp
    include/quadriflow/dset.hpp
    include/quadriflow/field-math.hpp
    include/quadriflow/flow.hpp
    src/hierarchy.cpp
    include/quadriflow/hierarchy.hpp
    src/loader.cpp
    include/quadriflow/loader.hpp
    src/localsat.cpp
    include/quadriflow/localsat.hpp
    src/main.cpp
    src/merge-vertex.cpp
    include/quadriflow/merge-vertex.hpp
    src/optimizer.cpp
    include/quadriflow/optimizer.hpp
    src/parametrizer.cpp
    src/parametrizer-flip.cpp
    src/parametrizer-int.cpp
    src/parametrizer-mesh.cpp
    src/parametrizer-scale.cpp
    src/parametrizer-sing.cpp
    include/quadriflow/parametrizer.hpp
    include/quadriflow/serialize.hpp
    src/subdivide.cpp
    include/quadriflow/subdivide.hpp
)
target_link_libraries(
    libQuadriflow
    ${TBB_LIBRARIES}
    ${LEMON_LIBRARIES}
    ${GUROBI_LIBRARIES}
)
target_include_directories(libQuadriflow PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(libQuadriflow PROPERTIES FOLDER "libraries")

add_executable(
    quadriflow
    src/main.cpp
)
target_link_libraries(quadriflow libQuadriflow)
set_target_properties(quadriflow PROPERTIES FOLDER "programs")


configure_package_config_file(quadriflowConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/quadriflowConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS libQuadriflow lemon EXPORT quadriflow-targets LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS quadriflow LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

install(EXPORT quadriflow-targets FILE quadriflowTargets.cmake DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/quadriflowConfig.cmake DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")
