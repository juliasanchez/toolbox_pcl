cmake_minimum_required(VERSION 2.8.11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")

project (get_projection)

find_package(PCL 1.8 REQUIRED COMPONENTS common io features search filters keypoints registration visualization)
if ( PCL_FOUND )
        include_directories( ${PCL_INCLUDE_DIRS} )
        link_directories(${PCL_LIBRARY_DIRS})
        add_definitions( ${PCL_DEFINITIONS} )
        add_definitions( -DFEVV_USE_PCL )
else()
        message ( "Unfound PCL package")
endif ()

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(get_projection main.cpp)

target_link_libraries(get_projection ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_KEYPOINTS_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
