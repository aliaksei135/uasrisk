add_subdirectory(openaip)
add_subdirectory(opensky)
add_subdirectory(path_planning)

set(UR_SOURCES
	${UR_SOURCES}
	${CMAKE_CURRENT_LIST_DIR}/Bresenham3D.h
	${CMAKE_CURRENT_LIST_DIR}/VoxelGrid.cpp
	${CMAKE_CURRENT_LIST_DIR}/VoxelGridBuilder.h
	${CMAKE_CURRENT_LIST_DIR}/VoxelGridBuilder.cpp
	${CMAKE_CURRENT_LIST_DIR}/Bresenham3D.h
	${CMAKE_CURRENT_LIST_DIR}/AirRiskVoxelGrid.cpp
	${CMAKE_CURRENT_LIST_DIR}/GroundRiskVoxelGrid.cpp
	PARENT_SCOPE)