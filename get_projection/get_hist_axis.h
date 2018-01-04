#ifndef GET_HIST_AXIS
#define GET_HIST_AXIS

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void get_hist_axis(std::vector<double> axis, pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string name);
#include "get_hist_axis.inl"

#endif // GET_HIST_AXIS
