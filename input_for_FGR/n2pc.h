#ifndef N2PC
#define N2PC

#include<iostream>

typedef pcl::PointXYZ pcl_point;

void n2pc(pcl::PointCloud<pcl::Normal>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr cloudout);

#include "n2pc.inl"

#endif // N2PC
