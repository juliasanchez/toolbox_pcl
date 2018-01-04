cmake_minimum_required(VERSION 2.8.11)

project (NDT)

set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS")

find_package(PCL 1.8 REQUIRED COMPONENTS common io filters registration visualization)
if ( PCL_FOUND )
        include_directories( ${PCL_INCLUDE_DIRS} )
        link_directories(${PCL_LIBRARY_DIRS})
        add_definitions( ${PCL_DEFINITIONS} )
        add_definitions( -DFEVV_USE_PCL )
else()
        message ( "Unfound PCL package")
endif ()

add_executable(NDT main.cpp)

target_link_libraries(NDT ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
