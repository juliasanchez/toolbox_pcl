//ROUTINE TO REGISTER 2 POINTCLOUDS

#include "/home/julia/Desktop/toolbox_pcl/display/display_clouds.h"
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

// PCL INCLUDES

using pcl::transformPointCloud;
typedef pcl::PointXYZI pcl_point;

//...............................................................................................................................

int main(int argc, char** argv)
{
    std::string pcd_file( argv[1] );

    pcl::PointCloud<pcl_point>::Ptr cloud(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( pcd_file, *cloud );

    //removing Nan

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    //trimming

    typename pcl::ConditionAnd<pcl_point>::Ptr condition (new pcl::ConditionAnd<pcl_point>);
    condition->addComparison(typename pcl::FieldComparison<pcl_point>::ConstPtr(new typename pcl::FieldComparison<pcl_point>("x", pcl::ComparisonOps::LT, atof(argv[2]))));
    condition->addComparison(typename pcl::FieldComparison<pcl_point>::ConstPtr(new typename pcl::FieldComparison<pcl_point>("y", pcl::ComparisonOps::LT, atof(argv[3]))));
    condition->addComparison(typename pcl::FieldComparison<pcl_point>::ConstPtr(new typename pcl::FieldComparison<pcl_point>("z", pcl::ComparisonOps::LT, atof(argv[4]))));

    condition->addComparison(typename pcl::FieldComparison<pcl_point>::ConstPtr(new typename pcl::FieldComparison<pcl_point>("x", pcl::ComparisonOps::GT, atof(argv[5]))));
    condition->addComparison(typename pcl::FieldComparison<pcl_point>::ConstPtr(new typename pcl::FieldComparison<pcl_point>("y", pcl::ComparisonOps::GT, atof(argv[6]))));
    condition->addComparison(typename pcl::FieldComparison<pcl_point>::ConstPtr(new typename pcl::FieldComparison<pcl_point>("z", pcl::ComparisonOps::GT, atof(argv[7]))));

    pcl::ConditionalRemoval<pcl_point> filter (condition);
    filter.setInputCloud(cloud);
    filter.filter(*cloud);


    int color[3]={20,230,20};
    display_clouds(cloud, cloud, color, color, 1, 1);

    pcl::io::savePCDFileASCII ("trimming.pcd", *cloud);


    return 0;
}
