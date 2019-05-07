target_include_directories (libppcvivace PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(libppcvivace PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}/CongestionController.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/MonitorIntervalQueue.cpp
)
