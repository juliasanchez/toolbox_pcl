#include <iostream>
#include <string>
//#include <pcl/impl/point_types.hpp>
#include <pcl/features/fpfh_omp.h>

#include "cloud.h"

typedef pcl::PointXYZI pcl_point;

int main(int argc, char *argv[])
{
    std::string file_address;

    for (int i =2; i<20; i++)
    {
    std::stringstream sstm;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/hokuyo/pcd/Hokuyo_";
    sstm <<i<<".pcd";
    file_address = sstm.str();

    pcl::PointCloud<pcl_point>::Ptr cloud_in(new pcl::PointCloud<pcl_point>);

    cloud<pcl_point> cloud_src;
    cloud_src.setInputCloud(cloud_in);
    cloud_src.load(file_address);

    cloud_src.clean();
    cloud_src.sample();

    pcl::search::KdTree<pcl_point>::Ptr tree (new pcl::search::KdTree<pcl_point>);
    cloud_src.getTree(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cloud_src.getNormals(0.15, normals);

    float feature_radius_= atof(argv[1]);
    pcl::FPFHEstimationOMP<pcl_point, pcl::Normal, pcl::FPFHSignature33> fest;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    fest.setRadiusSearch(feature_radius_);
    fest.setInputCloud(cloud_in);
    fest.setInputNormals(normals);
    fest.compute(*cloud_features);

    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/hokuyo/features/Hokuyo_";
    sstm <<i<<".bin";
    file_address = sstm.str();
    const char *features_file(file_address.c_str());

    FILE* fid = fopen(features_file, "wb");
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
}
