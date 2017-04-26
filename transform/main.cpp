//ROUTINE TO TRANSFORM 1 POINTCLOUD

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "/home/julia/Desktop/my_programs/toolbox_pcl/display/display_clouds.h"

// PCL INCLUDES

typedef pcl::PointXYZI pcl_point;

//...............................................................................................................................

int main(int argc, char** argv)
{
    std::string pcd_file( argv[1] );

    pcl::PointCloud<pcl_point>::Ptr cloud(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( pcd_file, *cloud );

    Eigen::Matrix4f transform_rot = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_theta = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_phi = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta =  atof(argv[2]); // The angle of rotation in radians
    float phi=  atof(argv[3]); // The angle of rotation in radians
    transform_theta (0,0) = cos (theta);
    transform_theta (0,1) = -sin(theta);
    transform_theta (1,0) = sin (theta);
    transform_theta (1,1) = cos (theta);

    transform_phi (0,0) = cos (phi);
    transform_phi (0,2) = sin(phi);
    transform_phi (2,0) = -sin(phi);
    transform_phi (2,2) = cos(phi);

    pcl::PointCloud<pcl_point>::Ptr transformed_cloud(new pcl::PointCloud<pcl_point>);

    Eigen::Matrix4f transform_trans = Eigen::Matrix4f::Zero();
    transform_phi (0,3) = 0.2;
    transform_phi (1,3) = 0.3;
    transform_phi (2,3) = 0.1;

    // Print the transformation
    std::cout << transform_theta*transform_phi+transform_trans << std::endl;

    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_theta*transform_phi+transform_trans);
    int color[3]={20,230,20};
    display_clouds(transformed_cloud, transformed_cloud, color, color, 1, 1);

    std::cerr << "Number of points cloud : " << cloud->size() << std::endl;
    std::cerr << "Number of points transformed_cloud : " << transformed_cloud->size() << std::endl;

    pcl::io::savePCDFileASCII ("transformed_cloud.pcd", *transformed_cloud);
    return 0;
}
