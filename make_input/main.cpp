#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/extract_indices.h>
#include "pre_process.h"

using namespace std;

int main(int argc, char *argv[])
{
    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);

    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();

    float sample =atof(argv[2]);
    float normal_radius=atof(argv[3]);
    double reso1;
    pre_process(argv[1],sample, normal_radius, 200, cloud_src, transform_init, normals_src, &reso1);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);

    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_src, *pointNormals_src, indices);

    std::string file_name1;
    std::string file_name=argv[1];
    size_t lastindex_point = file_name.find_last_of(".");
    size_t lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
    {
       lastindex_slash = 0;
    }

    file_name1 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
    std::string path=argv[4];

    std::stringstream sstm;
    sstm.str("");
    sstm<<path<<file_name1<<".pcd";
    std::string file_name_tot = sstm.str();

    pcl::io::savePCDFileASCII (file_name_tot , *pointNormals_src);
}
