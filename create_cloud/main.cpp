#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "/home/julia/Desktop/my_programs/toolbox_pcl/display/display_clouds.h"

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // Fill in the cloud data
  cloud->width    = 5000;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    int test =rand() % 100;
    if(test>50)
    {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 0;
    }
    else
    {
        cloud->points[i].z = (1024 * rand () / (RAND_MAX + 1.0f));
        float fix=cos(M_PI/3)*(1024 * rand () / (RAND_MAX + 1.0f));
        cloud->points[i].x = -cos(M_PI/3)-fix;
        cloud->points[i].y = sin(M_PI/3)-fix*tan(M_PI/3);
    }
  }

  int color[3]={20,230,20};
    display_clouds(cloud, cloud, color, color, 1, 1);


  pcl::io::savePCDFileASCII ("new_cloud.pcd", *cloud);

  return (0);
}
