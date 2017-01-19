#include <iostream>
#include <string>
//#include <pcl/impl/point_types.hpp>
#include <pcl/features/fpfh_omp.h>

#include "cloud.h"

typedef pcl::PointXYZI pcl_point;

int main(int argc, char *argv[])
{

    int n=1;
    std::string pcd_file1( argv[n] );
    n++;

    pcl::PointCloud<pcl_point>::Ptr cloud_in(new pcl::PointCloud<pcl_point>);

    cloud<pcl_point> cloud_src;
    cloud_src.setInputCloud(cloud_in);
    cloud_src.load(pcd_file1);

    cloud_src.clean();
    cloud_src.sample();

    pcl::io::savePCDFileASCII ("sampled_cloud.pcd", *cloud_in);
    pcl::search::KdTree<pcl_point>::Ptr tree (new pcl::search::KdTree<pcl_point>);
    cloud_src.getTree(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cloud_src.getNormals(0.15, normals);

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
