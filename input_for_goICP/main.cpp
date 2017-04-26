#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{

    std::string pcd_file1( argv[1] );
    std::string pcd_file2( argv[2] );

    pcl::PointCloud<pcl_point>::Ptr cloud1(new pcl::PointCloud<pcl_point>);
    if( pcl::io::loadPCDFile<pcl_point>( pcd_file1, *cloud1) == -1 )
    {
        PCL_ERROR ("Couldn't read file\n");
    }

    pcl::PointCloud<pcl_point>::Ptr cloud2(new pcl::PointCloud<pcl_point>);
    if( pcl::io::loadPCDFile<pcl_point>( pcd_file2, *cloud2) == -1 )
    {
        PCL_ERROR ("Couldn't read file\n");
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
    pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices);

    pcl::PassThrough<pcl_point> pass;
    pass.setInputCloud (cloud1);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0,0.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud1);
    pass.setInputCloud (cloud2);
    pass.filter (*cloud2);

    float samp=atof(argv[3]);

    pcl::UniformSampling<pcl_point> uniform_sampling;
    uniform_sampling.setInputCloud (cloud1);
    uniform_sampling.setRadiusSearch (samp);
    std::cout << "before : " << cloud1->size ()<<std::endl;
    uniform_sampling.filter (*cloud1);
    std::cout << " after sampling : " << cloud1->size () << std::endl<<std::endl;
    uniform_sampling.setInputCloud (cloud2);
    std::cout << "before : " << cloud2->size ()<<std::endl;
    uniform_sampling.filter (*cloud2);
    std::cout << " after sampling : " << cloud2->size () << std::endl<<std::endl;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud1, centroid);
    for(int i=0; i<cloud1->size(); i++)
    {    
        cloud1->points[i].x=cloud1->points[i].x-centroid(0);
        cloud1->points[i].y=cloud1->points[i].y-centroid(1);
        cloud1->points[i].z=cloud1->points[i].z-centroid(2);
    }

    pcl::compute3DCentroid (*cloud2, centroid);
    for(int i=0; i<cloud2->size(); i++)
    {
        cloud2->points[i].x=cloud2->points[i].x-centroid(0);
        cloud2->points[i].y=cloud2->points[i].y-centroid(1);
        cloud2->points[i].z=cloud2->points[i].z-centroid(2);
    }

    pcl_point point_min;
    pcl_point point_max;
    pcl::getMinMax3D(*cloud1, point_min, point_max);
    float maxi_x1=std::max(abs(point_max.x), abs(point_min.x));
    float maxi_y1=std::max(abs(point_max.y), abs(point_min.y));
    float maxi_z1=std::max(abs(point_max.z), abs(point_min.z));

    pcl::getMinMax3D(*cloud2, point_min, point_max);
    float maxi_x2=std::max(abs(point_max.x), abs(point_min.x));
    float maxi_y2=std::max(abs(point_max.y), abs(point_min.y));
    float maxi_z2=std::max(abs(point_max.z), abs(point_min.z));

    for(int i=0; i<cloud1->size(); i++)
    {
        cloud1->points[i].x=cloud1->points[i].x/std::max(maxi_x1, maxi_x2);
        cloud1->points[i].y=cloud1->points[i].y/std::max(maxi_y1, maxi_y2);
        cloud1->points[i].z=cloud1->points[i].z/std::max(maxi_z1, maxi_z2);
    }

    for(int i=0; i<cloud2->size(); i++)
    {
        cloud2->points[i].x=cloud2->points[i].x/std::max(maxi_x1, maxi_x2);
        cloud2->points[i].y=cloud2->points[i].y/std::max(maxi_y1, maxi_y2);
        cloud2->points[i].z=cloud2->points[i].z/std::max(maxi_z1, maxi_z2);
    }

    pcl::io::savePCDFileASCII ("scaled_cloud.pcd", *cloud1);


    std::ofstream file("1.txt");
    file<<cloud1->size()<<"\n";
    for(int i=0; i<cloud1->size(); i++)
    file<<cloud1->points[i].x<<" "<<cloud1->points[i].y<<" "<<cloud1->points[i].z<<"\n";
    file.close();

    file.open("2.txt");
    file<<cloud2->size()<<"\n";
    for(int i=0; i<cloud2->size(); i++)
    file<<cloud2->points[i].x<<" "<<cloud2->points[i].y<<" "<<cloud2->points[i].z<<"\n";
    file.close();
}
