#include <iostream>
#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>
#include "/home/julia/Desktop/my_programs/toolbox_pcl/display/display_clouds.h"

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

  // Fill in the cloud data
  cloud->width    = 40000;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  float theta = M_PI/2;

  // Define random generator with Gaussian distribution
  float mean = 0.0;
  float sigma = 0.005;
  std::default_random_engine generator;
  std::normal_distribution<float> dist(mean, sigma);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    int test =rand() % 100;
    if(test>50)
    {
        cloud->points[i].x = abs(1024 * rand () / (RAND_MAX + 1.0f));
        cloud->points[i].z = abs(1024 * rand () / (RAND_MAX + 1.0f));
        cloud->points[i].y = 0;//dist(generator);
        cloud->points[i].normal_x = 0;
        cloud->points[i].normal_y = 1;
        cloud->points[i].normal_z = 0;

    }
    else
    {
        cloud->points[i].z = abs(1024 * rand () / (RAND_MAX + 1.0f));
        cloud->points[i].y = sin(theta) * abs(1024 * rand () / (RAND_MAX + 1.0f));
        cloud->points[i].x = cloud->points[i].y / tan(theta);// + dist(generator);
        cloud->points[i].normal_x = 1;
        cloud->points[i].normal_y = 0;
        cloud->points[i].normal_z = 0;

    }
  }

  pcl::UniformSampling<pcl::PointNormal> uniform_sampling_n;
  uniform_sampling_n.setRadiusSearch (0.01);
  uniform_sampling_n.setInputCloud (cloud);
  uniform_sampling_n.filter (*cloud);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    if(cloud->points[i].y == 0)
    {
        cloud->points[i].y = dist(generator);
    }
    else
    {
        cloud->points[i].x +=  dist(generator);
    }
  }

//  int color[3]={20,230,20};
//    display_clouds(cloud, cloud, color, color, 1, 1);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  float theta_init = 0;
  std::vector<float> rot_ax = {1,0,0};

  transform  (0,0) = rot_ax[0]*rot_ax[0]*(1-cos (theta_init))+cos (theta_init);
  transform  (0,1) = rot_ax[0]*rot_ax[1]*(1-cos (theta_init))-rot_ax[2]*sin (theta_init);
  transform  (0,2) = rot_ax[0]*rot_ax[2]*(1-cos (theta_init))+rot_ax[1]*sin (theta_init);
  transform  (1,0) = rot_ax[1]*rot_ax[0]*(1-cos (theta_init))+rot_ax[2]*sin (theta_init);
  transform  (1,1) = rot_ax[1]*rot_ax[1]*(1-cos (theta_init))+cos (theta_init);
  transform  (1,2) = rot_ax[1]*rot_ax[2]*(1-cos (theta_init))-rot_ax[0]*sin (theta_init);
  transform  (2,0) = rot_ax[2]*rot_ax[0]*(1-cos (theta_init))-rot_ax[1]*sin (theta_init);
  transform  (2,1) = rot_ax[2]*rot_ax[1]*(1-cos (theta_init))+rot_ax[0]*sin (theta_init);
  transform  (2,2) = rot_ax[2]*rot_ax[2]*(1-cos (theta_init))+cos (theta_init);
  transform  (0,3) = 1;
  transform  (1,3) = 1;
  transform  (2,3) = 1;

  pcl::transformPointCloudWithNormals (*cloud, *cloud, transform);

  pcl::io::savePCDFileASCII ("new_cloud.pcd", *cloud);

  return (0);
}
