cmake_minimum_required(VERSION 3.7)
project(MPCC)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )

include_directories(.)
# include_directories(External/osqp/lib/include)
# include_directories(External/osqp-eigen/lib/include)
find_package(OsqpEigen REQUIRED)
include_directories(External/matplotlib)
# include_directories(External/Eigen)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(External/Json/include)
# include_directories(External/rbdl/lib/include)
find_package(RBDL REQUIRED)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_library(MPCC_LIB SHARED
        config.h
        types.cpp
        types.h
        Model/robot_model.cpp
        Model/robot_model.h
        Constraints/SelfCollision/SelfCollisionModel.cpp
        Constraints/SelfCollision/SelfCollisionModel.h
        Interfaces/osqp_interface.cpp
        Interfaces/solver_interface.cpp
        Model/model.cpp
        Model/integrator.cpp
        Spline/cubic_spline.cpp
        Spline/cubic_spline_rot.cpp
        Spline/arc_length_spline.cpp
        Params/params.cpp
        Constraints/constraints.cpp
        Cost/cost.cpp
        Constraints/bounds.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Params/track.h
        )
target_link_libraries(MPCC_LIB 
        ${catkin_LIBRARIES} 
        ${RBDL_LIBRARY} 
        ${RBDL_URDFReader_LIBRARY} 
        # ${CMAKE_SOURCE_DIR}/External/osqp/lib/lib/libosqp.so 
        # ${CMAKE_SOURCE_DIR}/External/osqp-eigen/lib/lib/libOsqpEigen.so
        OsqpEigen::OsqpEigen 
)

add_executable(MPCC main.cpp)
find_package(Python COMPONENTS Development)
target_include_directories(MPCC PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(MPCC ${Python_LIBRARIES} MPCC_LIB)

add_executable(TEST
        Tests/main_test.cpp
        Tests/model_integrator_test.h
        Tests/robot_model_test.h
        Tests/self_collision_test.h
        Tests/spline_test.h
        Tests/constraints_test.h
        Tests/cost_test.h
)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
find_package(Python COMPONENTS Development)
target_include_directories(TEST PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(TEST PRIVATE MPCC_LIB gtest gtest_main pthread ${Python_LIBRARIES})
