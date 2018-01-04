#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <pcl/registration/transformation_estimation_svd.h>
#include <omp.h>
#include <iterator>
#include <chrono>
#include <pcl/filters/extract_indices.h>

#include "pre_process.h"
#include "save_vector.h"
#include "save_axis.h"
#include "save_cluster.h"
#include "pcn2pc.h"
#include "filter_normals.h"
#include "filter_walls.h"
#include "density_filter.h"
#include "get_hist_axis.h"
#include "MeanShift.h"
#include "size_cluster.h"


bool comp_clus(Cluster clus1, Cluster clus2 );


typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{

    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);

    float sample =atof(argv[2]);
    float normal_radius=atof(argv[3]);
    double reso1;
    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();
    transform_init(0,0)=0.982156;
    transform_init(0,1)=-0.187977;
    transform_init(0,2)=-0.00584022;
    transform_init(1,0)= 0.187973;
    transform_init(1,1)=0.982174;
    transform_init(1,2)=-0.00130722;
    transform_init(2,0)=0.00598191;
    transform_init(2,1)=0.000186106 ;
    transform_init(2,2)=0.999982;

    pre_process(argv[1],sample, normal_radius, cloud_src, transform_init, normals_src, &reso1);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_src, *pointNormals_src, indices);

    ///filter points to get points with normal laying in clusters
    ///
    //1_create a pointcloud representing normals as points on a sphere

    pcl::PointCloud<pcl_point>::Ptr normals1(new pcl::PointCloud<pcl_point>);
    pcn2pc(pointNormals_src, normals1);

    //2_ filter normals to enhance clusters   HEAVY STEP
    pcl::RandomSample<pcl_point> rand_filter(true);

    float perc = atof(argv[4]);
    float radius =atof(argv[5]);

    int neigh=normals1->size()*perc;
    std::cout<<"minimum neighbors : "<<neigh<<std::endl;
    density_filter(normals1, radius, (int)neigh/2);
    density_filter(normals1, radius, neigh);

    int meanshift_points=atoi(argv[6]);

    rand_filter.setSample (meanshift_points);
    if(meanshift_points<normals1->points.size())
    {
        rand_filter.setInputCloud (normals1);
        rand_filter.filter(*normals1);
    }

    std::vector< vector<double> > vec_normals_src(normals1->points.size(), std::vector<double>(3, 0.0));

    double kernel_bandwidth = 0.1;
    MeanShift *msp = new MeanShift();
    for (int i=0; i<normals1->points.size(); i++)
    {
        vec_normals_src[i][0]=normals1->points[i].x;
        vec_normals_src[i][1]=normals1->points[i].y;
        vec_normals_src[i][2]=normals1->points[i].z;
    }

    vector<Cluster> clusters1 = msp->cluster(vec_normals_src, kernel_bandwidth);

    for (int i=0; i<clusters1.size(); i++)
    {
        float size=size_cluster(clusters1[i]);
        if(size<0.97)
        {
            clusters1[i].original_points.resize(0);
        }
    }

    ///sort clusters and keep only the 6th most important

    sort(clusters1.begin(), clusters1.end(),comp_clus);

    int n_clus1=std::min(6,(int)clusters1.size());

    vector<Cluster> clusters1_test;
    for(int i=0; i<n_clus1; i++)
    {
        clusters1_test.push_back(clusters1[i]);
    }
    clusters1.clear();
    clusters1=clusters1_test;

    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        float modu=sqrt(clusters1[clus].mode[0]*clusters1[clus].mode[0]+clusters1[clus].mode[1]*clusters1[clus].mode[1]+clusters1[clus].mode[2]*clusters1[clus].mode[2]);
        for (int m=0; m<3; m++)
        {
            clusters1[clus].mode[m]=clusters1[clus].mode[m]/(modu);
        }
    }

    ///save normals

    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        std::stringstream sstm;
        sstm<<"cluster1_mode"<<clus<<".csv";
        std::string cluster_name = sstm.str();
         save_cluster(clusters1[clus].mode,cluster_name);
    }

    /// keep all normals in a vector

    std::vector<std::vector<double>> vec_normals1(clusters1.size(), std::vector<double>(3));

    for(int clus = 0; clus < clusters1.size(); clus++)
    {
        vec_normals1[clus]=clusters1[clus].mode;
    }

    /// filter pointclouds to keep only planar sections (which are used in translation process)

    filter_walls(pointNormals_src, vec_normals1);
    clusters1[1].mode[0]=0.00970171;
    clusters1[1].mode[1]=0.00241581;
    clusters1[1].mode[2]=0.000201702;

    get_hist_axis(clusters1[0].mode, pointNormals_src, "proj1.txt");
    get_hist_axis(clusters1[1].mode, pointNormals_src, "proj2.txt");
    get_hist_axis(clusters1[2].mode, pointNormals_src, "proj3.txt");

}

bool comp_clus(Cluster clus1, Cluster clus2 )
{
    if ( clus1.original_points.size()>clus2.original_points.size() )
    {
        return true;
    }
    return false;
}
