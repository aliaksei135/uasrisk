target_sources(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/uasrisk/air/io/openaip/OpenAIPReader.h
	${CMAKE_CURRENT_LIST_DIR}/uasrisk/air/io/opensky/OpenSkyCsvReader.h
	${CMAKE_CURRENT_LIST_DIR}/uasrisk/air/io/opensky/OpenSkyJsonReader.h
	${CMAKE_CURRENT_LIST_DIR}/uasrisk/air/io/opensky/OpenSkyReader.h
	"${CMAKE_CURRENT_LIST_DIR}/uasrisk/environment/VoxelGrid.h"
	"${CMAKE_CURRENT_LIST_DIR}/uasrisk/environment/VoxelGridBuilder.h"
	"${CMAKE_CURRENT_LIST_DIR}/uasrisk/environment/TypeDefs.h"
	${CMAKE_CURRENT_LIST_DIR}/uasrisk/ground/GroundRiskVoxelGrid.h
	)