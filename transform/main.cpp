//ROUTINE TO REGISTER 2 POINTCLOUDS

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "/home/julia/Desktop/toolbox_pcl/display/display_clouds.h"

// PCL INCLUDES

using pcl::transformPointCloud;
typedef pcl::PointXYZI pcl_point;

//...............................................................................................................................

int main(int argc, char** argv)
{
    std::string pcd_file( argv[1] );

    pcl::PointCloud<pcl_point>::Ptr cloud(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( pcd_file, *cloud );

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = atof(argv[2]); // The angle of rotation in radians
    transform (0,0) = cos (theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin (theta);
    transform (1,1) = cos (theta);

    // Define a translation of 2.5 meters on the x axis.
    transform (0,3) = atof(argv[3]);

    // Print the transformation
    std::cout << transform << std::endl;

    pcl::PointCloud<pcl_point>::Ptr transformed_cloud(new pcl::PointCloud<pcl_point>);

    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    std::cerr << "Number of points cloud" << cloud->size() << std::endl;
    std::cerr << "Number of points transformed_cloud" << transformed_cloud->size() << std::endl;

    int color1[3]={20,230,20};
    int color2[3]={20,20,230};
    display_clouds(cloud, transformed_cloud, color1, color2, 1, 1);

    pcl::io::savePCDFileASCII ("transformed_cloud.pcd", *transformed_cloud);


    return 0;
}
