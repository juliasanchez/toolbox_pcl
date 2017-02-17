#include <iostream>
#include <string>
//#include <pcl/impl/point_types.hpp>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "cloud.h"

typedef pcl::PointXYZI pcl_point;
using pcl::transformPointCloud;

int main(int argc, char *argv[])
{

    int n=1;
    std::string pcd_file1( argv[n] );
    n++;

    pcl::PointCloud<pcl_point>::Ptr cloud_in(new pcl::PointCloud<pcl_point>);

    cloud<pcl_point> cloud_src;
    cloud_src.setInputCloud(cloud_in);
    cloud_src.load(pcd_file1);
    cloud_src.sample(atof(argv[n]));
    cloud_src.clean();
    n++;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    float theta = atof(argv[n]); // The angle of rotation in radians
    n++;
    transform (0,0) = cos (theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin (theta);
    transform (1,1) = cos (theta);

    // Print the transformation
    std::cout << transform << std::endl;

    cloud_src.transform(transform);

    pcl::search::KdTree<pcl_point>::Ptr tree (new pcl::search::KdTree<pcl_point>);
    cloud_src.getTree(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cloud_src.getNormals(0.4, normals);

    float feature_radius_= atof(argv[n]);
    n++;
    pcl::FPFHEstimationOMP<pcl_point, pcl::Normal, pcl::FPFHSignature33> fest;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    fest.setRadiusSearch(feature_radius_);
    fest.setInputCloud(cloud_in);
    fest.setInputNormals(normals);
    fest.compute(*cloud_features);

    FILE* fid = fopen("features.bin", "wb");
    int nV = cloud_in->size(), nDim = 33;
    fwrite(&nV, sizeof(int), 1, fid);
    fwrite(&nDim, sizeof(int), 1, fid);
    for (int v = 0; v < nV; v++)
    {
        const pcl_point &pt = cloud_in->points[v];
        float xyz[3] = {pt.x, pt.y, pt.z};
        fwrite(xyz, sizeof(float), 3, fid);
        const pcl::FPFHSignature33 &feature = cloud_features->points[v];
        fwrite(feature.histogram, sizeof(float), 33, fid);
    }
    fclose(fid);
}
