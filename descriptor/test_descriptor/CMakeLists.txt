cmake_minimum_required(VERSION 2.8.11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")


project (test_descriptor)

find_package(PCL 1.8 REQUIRED COMPONENTS common io features search filters keypoints registration visualization)
if ( PCL_FOUND )
        include_directories( ${PCL_INCLUDE_DIRS} )
        link_directories(${PCL_LIBRARY_DIRS})
        add_definitions( ${PCL_DEFINITIONS} )
        add_definitions( -DFEVV_USE_PCL )
else()
        message ( "Unfound PCL package")
endif ()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(DLIB REQUIRED)
if(DLIB_FOUND)
    include_directories(${DLIB_INCLUDE_DIR})
endif(DLIB_FOUND)


add_executable(test_descriptor main.cpp)

target_link_libraries(test_descriptor ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_KEYPOINTS_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

target_link_libraries(test_descriptor ${DLIB_LIBRARIES})
