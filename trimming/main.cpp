//ROUTINE TO REGISTER 2 POINTCLOUDS

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/uniform_sampling.h>

// PCL INCLUDES

typedef pcl::PointXYZ pcl_point;

//...............................................................................................................................

int main(int argc, char** argv)
{
    std::string pcd_file( argv[1] );

    pcl::PointCloud<pcl_point>::Ptr cloud(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( pcd_file, *cloud );

    //removing Nan

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::UniformSampling<pcl_point> uniform_sampling;
    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (0.005);
    uniform_sampling.filter (*cloud);

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

    pcl::io::savePCDFileASCII ("trimming.pcd", *cloud);


    return 0;
}
