#ifndef CLOUD_H
#define CLOUD_H

#include <vector>
#include <string>
#include <iostream>

// PCL INCLUDES

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/random_sample.h>
#include "filter_far.h"

#if defined _MSC_VER
#pragma warning (disable: 4996) // MT
#endif

template<typename points>
class cloud
{
public:
	cloud();
        void setInputCloud(typename pcl::PointCloud<points>::Ptr);
        void setTree();
        void getScale(float*);
        void clean(float far);
	void getInputCloud(typename pcl::PointCloud<points>::Ptr);
        void getNormals(float, typename pcl::PointCloud<pcl::Normal>::Ptr) const;
        void getSize(int*) const;
	void load (std::string);
        double computeCloudResolution () const;
        void sample(float samp);
        void rand_sample(float samp);
        void transform( Eigen::Matrix4f matrix_transform);


private:
        typename pcl::PointCloud<points>::Ptr cloud_in;
	typename pcl::search::KdTree<points> tree;
};

#include "cloud.inl"

#endif // CLOUD
