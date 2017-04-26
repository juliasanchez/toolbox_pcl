#ifndef REMOVENAN
#define REMOVENAN

#include<iostream>

typedef pcl::PointXYZ pcl_point;

void removeNan(pcl::PointCloud<pcl_point>::Ptr cloud, pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr features);

#include "removeNan.inl"

#endif // REMOVENAN
