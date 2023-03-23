add_subdirectory(openaip)
add_subdirectory(opensky)

set(UR_SOURCES
	${UR_SOURCES}
	${CMAKE_CURRENT_LIST_DIR}/Bresenham3D.h
	${CMAKE_CURRENT_LIST_DIR}/VoxelGrid.cpp
	${CMAKE_CURRENT_LIST_DIR}/VoxelGridBuilder.h
	${CMAKE_CURRENT_LIST_DIR}/VoxelGridBuilder.cpp
	${CMAKE_CURRENT_LIST_DIR}/Bresenham3D.h
	${CMAKE_CURRENT_LIST_DIR}/AirRiskVoxelGrid.cpp
	${CMAKE_CURRENT_LIST_DIR}/GroundRiskVoxelGrid.cpp
	${CMAKE_CURRENT_LIST_DIR}/IncrementalGroundRiskVoxelGrid.cpp
	${CMAKE_CURRENT_LIST_DIR}/RiskVoxelGrid.cpp
	PARENT_SCOPE)