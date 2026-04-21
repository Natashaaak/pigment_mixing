find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)

add_library(matter_lib
			   simulation/simulation.cpp
			   simulation/simulation_units.cpp
			   simulation/save_data.cpp
			   simulation/remesh.cpp
			   simulation/deformation_update.cpp
			   simulation/plasticity.cpp
			   simulation/p2g.cpp
			   simulation/g2p.cpp
			   simulation/explicit_euler_update.cpp
			   simulation/boundary_collision.cpp
			   simulation/update_dt.cpp
			   simulation/position_update.cpp
			   simulation/musl.cpp
			   simulation/pbc.cpp
			   simulation/precompute_weight.cpp
			   simulation/simulation_blobs.cpp
			   )

# For clients to find headers like data_structures.hpp
target_include_directories(matter_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(mpm mpm.cpp)
target_link_libraries(mpm PRIVATE matter_lib)


if (USE_VDB)

	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

	find_package(OpenVDB 3.1 QUIET)
	# This is for matter_lib internal use.
	target_include_directories(matter_lib PRIVATE ${OPENVDB_INCLUDE_DIRS} SYSTEM)

	find_package(TBB REQUIRED)
	target_include_directories(matter_lib PRIVATE ${TBB_INCLUDE_DIRS} SYSTEM)

	find_package(Boost REQUIRED)
	target_include_directories(matter_lib PRIVATE ${Boost_INCLUDE_DIRS} SYSTEM)

	target_link_libraries (matter_lib PUBLIC
						OpenMP::OpenMP_CXX
						Eigen3::Eigen
						${OPENVDB_LIBRARIES}
						${OpenEXR_LIBRARIES}
						${TBB_LIBRARIES}
						${Boost_LIBRARIES}
					)

else (USE_VDB)

	target_link_libraries (matter_lib PUBLIC OpenMP::OpenMP_CXX Eigen3::Eigen)

endif (USE_VDB)

# These definitions are needed by client code (e.g. mpm.cpp, and raytracing)
target_compile_definitions(matter_lib PUBLIC 
    INCLUDE_DIR="${CMAKE_SOURCE_DIR}/levelsets"
    SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# Keep this for backward compatibility with how mpm might be structured.
get_target_property(DEFS mpm COMPILE_DEFINITIONS)
message(STATUS "Compile Definitions for mpm: ${DEFS}")