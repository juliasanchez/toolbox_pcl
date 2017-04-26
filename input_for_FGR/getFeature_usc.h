#ifndef GET_FEATURE_USC
#define GET_FEATURE_USC

#include <iostream>
#include <pcl/features/usc.h>

typedef pcl::PointXYZ pcl_point;

void get_feature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr features);

#include "getFeature_usc.inl"

#endif // GET_FEATURE_USC
