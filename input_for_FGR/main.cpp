#include <iostream>
#include <string>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/extract_indices.h>

#include "pre_process.h"
#include "pcn2n.h"
#include "pcn2pc.h"
#include "n2pc.h"
#include "filter_normals.h"
#include "getFeature_usc.h"
#include "removeNan.h"

typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{

    std::string pcd_file1( argv[1] );

    pcl::PointCloud<pcl_point>::Ptr cloud_in(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();
    pre_process(pcd_file1, 50000, 0, cloud_in, transform_init, normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_in, *normals, *pointNormals);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals, *pointNormals, indices);

    pcn2pc(pointNormals, cloud_in);
    pcn2n(pointNormals, normals);

    if( atoi(argv[2]))

    {
        pcl::PointCloud<pcl_point>::Ptr normals_pc(new pcl::PointCloud<pcl_point>);
        n2pc(normals, normals_pc);

        float radius =0.07;
        float perc=0.02;
        pcl::PointIndices::Ptr removed_indices (new pcl::PointIndices());
        filter_normals(normals_pc, radius, perc, removed_indices);

        pcl::ExtractIndices<pcl_point> eifilter (true);
        eifilter.setNegative (true);
        eifilter.setInputCloud (cloud_in);
        eifilter.setIndices (removed_indices);
        eifilter.filter (*cloud_in);

        pcl::ExtractIndices<pcl::Normal> eifilter_n (true);
        eifilter_n.setNegative (true);
        eifilter_n.setInputCloud (normals);
        eifilter_n.setIndices (removed_indices);
        eifilter_n.filter (*normals);
    }

    pcl::io::savePCDFileASCII ("cloud.pcd", *cloud_in);
    pcl::io::savePCDFileASCII ("normals.pcd", *normals);

    float feature_radius_= atof(argv[3]);

//    pcl::FPFHEstimationOMP<pcl_point, pcl::Normal, pcl::FPFHSignature33> fest;
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_features(new pcl::PointCloud<pcl::FPFHSignature33>());
//    fest.setRadiusSearch(feature_radius_);
//    fest.setInputCloud(cloud_in);
//    fest.setInputNormals(normals);
//    fest.compute(*cloud_features);

    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr cloud_features(new pcl::PointCloud<pcl::UniqueShapeContext1960>);
    getFeature_usc(cloud_in, feature_radius_, cloud_features);

    removeNan(cloud_in,cloud_features);

    FILE* fid = fopen("features.bin", "wb");
    int nV = cloud_in->size();
//    int nDim = 33;
    int nDim=1960;
    fwrite(&nV, sizeof(int), 1, fid);
    fwrite(&nDim, sizeof(int), 1, fid);
    for (int v = 0; v < nV; v++)
    {
        const pcl_point &pt = cloud_in->points[v];
        float xyz[3] = {pt.x, pt.y, pt.z};
        fwrite(xyz, sizeof(float), 3, fid);
//        const pcl::FPFHSignature33 &feature = cloud_features->points[v];
//        fwrite(feature.histogram, sizeof(float), nDim, fid);
        const pcl::UniqueShapeContext1960 &feature = cloud_features->points[v];
        fwrite(feature.descriptor, sizeof(float), nDim, fid);
    }
    fclose(fid);
}
