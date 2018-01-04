#include <iostream>
#include <string>
#include <chrono>
#include <pcl/features/fpfh_omp.h>

#include "pre_process.h"

typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{
    std::string file_address;

    for (int i =0; i<41; i++)
    {
    std::stringstream sstm;
    sstm.str("");
//    sstm<<"/home/julia/Documents/data_base/hokuyo/pcd/Hokuyo_";
//    sstm <<i<<".pcd";
    sstm<<"/home/julia/Documents/data_base/Pavin1_ascii/pcd/scan";
    sstm <<i<<"00.pcd";
    file_address = sstm.str();

    pcl::PointCloud<pcl_point>::Ptr cloud_in(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();

    float sample =atof(argv[1]);
    float normal_radius=atof(argv[2]);
    double reso1;
    double reso2;
    pre_process(file_address,sample, normal_radius, atof(argv[4]), cloud_in, transform_init, normals, &reso1);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals(new pcl::PointCloud<pcl::PointNormal>);

    pcl::concatenateFields (*cloud_in, *normals, *pointNormals);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals, *pointNormals, indices);


    cloud_in->width    = pointNormals->points.size();
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (pointNormals->points.size());

    normals->width    = pointNormals->points.size();
    normals->height   = 1;
    normals->is_dense = false;
    normals->points.resize (pointNormals->points.size());


    for (int i=0; i<cloud_in->points.size(); i++)
    {
        cloud_in->points[i].x=pointNormals->points[i].x;
        cloud_in->points[i].y=pointNormals->points[i].y;
        cloud_in->points[i].z=pointNormals->points[i].z;
    }

    for (int i=0; i<cloud_in->points.size(); i++)
    {
        normals->points[i].normal_x=pointNormals->points[i].normal_x;
        normals->points[i].normal_y=pointNormals->points[i].normal_y;
        normals->points[i].normal_z=pointNormals->points[i].normal_z;
    }

    auto t_tot1 = std::chrono::high_resolution_clock::now();

    float feature_radius_= atof(argv[3]);
    pcl::FPFHEstimationOMP<pcl_point, pcl::Normal, pcl::FPFHSignature33> fest;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    fest.setRadiusSearch(feature_radius_);
    fest.setInputCloud(cloud_in);
    fest.setInputNormals(normals);
    fest.compute(*cloud_features);

    auto t_tot2 = std::chrono::high_resolution_clock::now();
    std::cout<<"total time :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count()<<" milliseconds"<<std::endl<<std::endl;

    sstm.str("");
    sstm<<"mkdir "<<argv[5];
    std::string command = sstm.str();
    const char* c=command.c_str();
    system(c);


    sstm.str("");
    sstm<< argv[5]<<"/scan";
    sstm <<i<<"00.bin";
//    sstm<< argv[5]<<"/Hokuyo_";
//    sstm <<i<<".bin";
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

     sstm.str("");
     sstm<<i<<".time";
    std::ofstream fout(sstm.str());
    fout<<std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count();
    fout.close();
    }
}
