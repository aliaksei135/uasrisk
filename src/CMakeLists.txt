add_subdirectory(openaip)
add_subdirectory(opensky)

set(UR_SOURCES
	${UR_SOURCES}
	${CMAKE_CURRENT_LIST_DIR}/Bresenham3D.h
	${CMAKE_CURRENT_LIST_DIR}/uasrisk.cpp
	${CMAKE_CURRENT_LIST_DIR}/uasrisk.h
	${CMAKE_CURRENT_LIST_DIR}/TypeDefs.h
	${CMAKE_CURRENT_LIST_DIR}/VoxelGrid.cpp
	${CMAKE_CURRENT_LIST_DIR}/VoxelGrid.h
	${CMAKE_CURRENT_LIST_DIR}/VoxelGridBuilderVisitor.cpp
	${CMAKE_CURRENT_LIST_DIR}/VoxelGridBuilderVisitor.h
	${CMAKE_CURRENT_LIST_DIR}/Bresenham3D.h
	PARENT_SCOPE)