#ifndef PCN2N
#define PCN2N

#include<iostream>

typedef pcl::PointXYZ pcl_point;

void pcn2n(pcl::PointCloud<pcl::PointNormal>::Ptr cloudin, pcl::PointCloud<pcl::Normal>::Ptr cloudout);

#include "pcn2n.inl"

#endif // PCN2N
