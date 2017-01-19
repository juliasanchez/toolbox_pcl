//ROUTINE TO REGISTER 2 POINTCLOUDS

#include "display_clouds.h"
#include "display_normals.h"
#include "cloud.h"
#include <pcl/features/usc.h>
#include <string>
#include <iostream>
#include <pcl/impl/point_types.hpp>
#include<pcl/visualization/histogram_visualizer.h>
#include <pcl/features/fpfh.h>

//typedef pcl::PointXYZRGB pcl_point;
typedef pcl::PointXYZI pcl_point;
typedef Eigen::Matrix4f Matrix;
typedef pcl::FPFHSignature33 descriptor;
//typedef pcl::UniqueShapeContext1960 descriptor;

void getFeature_3dsc(pcl::PointCloud<pcl_point>::Ptr cloudin,pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::ShapeContext1980>* features);
void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>* features);
void getFeature_fpfh(   pcl::PointCloud<pcl_point>::Ptr cloudin,    pcl::PointCloud<pcl_point>::Ptr keypoints,    pcl::PointCloud<pcl::Normal>::Ptr normals,   pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius,   pcl::PointCloud<pcl::FPFHSignature33>* features);

//...............................................................................................................................

int main(int argc, char** argv)
{
    int n=1;
        std::string pcd_file1( argv[n] );
        n++;
        std::string pcd_file2( argv[n] );
        n++;

//class cloud implemented to get normals, size and to load and display the cloud
        pcl::PointCloud<pcl_point>::Ptr cloud_in(new pcl::PointCloud<pcl_point>);
        pcl::PointCloud<pcl_point>::Ptr keypoint(new pcl::PointCloud<pcl_point>);

        cloud<pcl_point> cloud_src;
        cloud_src.setInputCloud(cloud_in);
        cloud_src.load(pcd_file1);

        cloud<pcl_point> keypoint0;
        keypoint0.setInputCloud(keypoint);
        keypoint0.load(pcd_file2);
        int color1[3]={20,230,20};
        int color2[3]={20,20,230};

        cloud_src.clean();
        pcl::search::KdTree<pcl_point>::Ptr tree (new pcl::search::KdTree<pcl_point>);
        cloud_src.getTree(tree);
        display_clouds(cloud_in, keypoint, color1, color2, 1, 10);

        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        cloud_src.getNormals(0.15, normals);

        pcl::PointCloud<descriptor> features;

   // getFeature_usc(cloud_in, keypoint, tree, atof(argv[n]), &features);
        getFeature_fpfh( cloud_in,  keypoint, normals, tree, atof(argv[n]), &features);

//int count=0;
//    for(int i=0; i<1960;++i)
//    {
//        if(features.at(0).descriptor[i]!=0)
//        {
//            count++;
//        }
//        std::cout<<features.at(0).descriptor[i]<<std::endl;
//    }

//    std::cout<<"number of non zero elements : "<<count<<std::endl;

    pcl::visualization::PCLHistogramVisualizer viewer;
    //viewer.addFeatureHistogram(features, 33);
    viewer.addFeatureHistogram(features, 33 , "cloud1", 640, 200);

    viewer.spin();

    return 0;
}

void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>* features)
{
        pcl::UniqueShapeContext<pcl_point, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> feature_estimation;
        feature_estimation.setSearchMethod(tree);
        feature_estimation.setRadiusSearch(descriptor_radius); //5 cm recherche des voisins pour faire le descriptor
        feature_estimation.setInputCloud(keypoints);
        feature_estimation.setSearchSurface(cloudin);
        feature_estimation.setRadiusSearch(descriptor_radius); // neigbours to take into account
        feature_estimation.setMinimalRadius(descriptor_radius / 10.0); //to avoid being too sensitive in bins close to center
        feature_estimation.setPointDensityRadius(descriptor_radius / 5.0); //radius for local density
        feature_estimation.setLocalRadius(descriptor_radius); //radius for local frame
        feature_estimation.compute(*features);
}

void getFeature_fpfh(   pcl::PointCloud<pcl_point>::Ptr cloudin,    pcl::PointCloud<pcl_point>::Ptr keypoints,    pcl::PointCloud<pcl::Normal>::Ptr normals,   pcl::search::KdTree<pcl_point>::Ptr tree,float descriptor_radius,    pcl::PointCloud<pcl::FPFHSignature33>* features)
{
        pcl::FPFHEstimation<pcl_point, pcl::Normal, pcl::FPFHSignature33> feature_estimation;

        feature_estimation.setSearchMethod(tree);
        feature_estimation.setRadiusSearch(descriptor_radius); //10 cm recherche des voisins pour faire le descriptor
        feature_estimation.setSearchSurface(cloudin);
        feature_estimation.setInputCloud(keypoints);
        feature_estimation.setInputNormals(normals);
        feature_estimation.compute(*features);
}
