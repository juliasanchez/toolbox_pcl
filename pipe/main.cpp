#include <iostream>
#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/transforms.h>


int main(int argc, char *argv[])
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

    // Fill in the cloud data
    int N = 100000;
    float R = 0.1;
    float angle = M_PI/2;
    cloud->width    = N;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

    float l = 1.0;

    std::random_device rd;
    std::default_random_engine generator (rd());
    std::default_random_engine generator_line (rd());
    std::uniform_real_distribution<float> distribution(0,2*M_PI);
    std::uniform_real_distribution<float> distribution_line(-l/2,l/2);


    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    std::vector<float> rot_ax = {0,1,0};

    transform  (0,0) = rot_ax[0]*rot_ax[0]*(1-cos (angle))+cos (angle);
    transform  (0,1) = rot_ax[0]*rot_ax[1]*(1-cos (angle))-rot_ax[2]*sin (angle);
    transform  (0,2) = rot_ax[0]*rot_ax[2]*(1-cos (angle))+rot_ax[1]*sin (angle);
    transform  (1,0) = rot_ax[1]*rot_ax[0]*(1-cos (angle))+rot_ax[2]*sin (angle);
    transform  (1,1) = rot_ax[1]*rot_ax[1]*(1-cos (angle))+cos (angle);
    transform  (1,2) = rot_ax[1]*rot_ax[2]*(1-cos (angle))-rot_ax[0]*sin (angle);
    transform  (2,0) = rot_ax[2]*rot_ax[0]*(1-cos (angle))-rot_ax[1]*sin (angle);
    transform  (2,1) = rot_ax[2]*rot_ax[1]*(1-cos (angle))+rot_ax[0]*sin (angle);
    transform  (2,2) = rot_ax[2]*rot_ax[2]*(1-cos (angle))+cos (angle);
    transform  (0,3) = 1;
    transform  (1,3) = 1;
    transform  (2,3) = 1;

    float mean = 0.0;
    float sigma = 0.002;
    std::default_random_engine generator_noise;
    std::normal_distribution<float> dist_noise(mean, sigma);

    std::vector<int> change(N);

    for (unsigned i = 0; i < N; ++i)
    {

      float theta = distribution(generator), ll = distribution_line(generator_line);
      if (ll < 0 && ll > -std::sin(angle)/(1+std::cos(angle))*std::cos(theta)*R)
      {
        ll +=  std::sin(angle)/(1+std::cos(angle))*std::cos(theta)*R;
        theta += M_PI;
        change[i] = 100;
      }
      else if (ll > 0 && ll < std::sin(angle)/(1+std::cos(angle))*std::cos(theta)*R)
      {
        ll += -std::sin(angle)/(1+std::cos(angle))*std::cos(theta)*R;
        theta += M_PI;
        change[i] = 100;
      }

      cloud->points[i].x = std::cos(theta)*R;
      cloud->points[i].y = std::sin(theta)*R;
      cloud->points[i].z = ll;

      cloud->points[i].normal_x = std::cos(theta) + change[i];
      cloud->points[i].normal_y = std::sin(theta)+ change[i];
      cloud->points[i].normal_z = 0;

    }

    pcl::UniformSampling<pcl::PointNormal> uniform_sampling_n;
    uniform_sampling_n.setRadiusSearch (0.005);
    uniform_sampling_n.setInputCloud (cloud);
    uniform_sampling_n.filter (*cloud);

    for (unsigned i = 0; i < cloud->size(); ++i)
    {
        float noise = dist_noise(generator_noise);
        cloud->points[i].x += (cloud->points[i].x/R)*noise ;
        cloud->points[i].y += (cloud->points[i].y/R)*noise;

      if (cloud->points[i].z > 0 && cloud->points[i].normal_x<10 || cloud->points[i].z < 0 && cloud->points[i].normal_x>10)
      {
          float x = cloud->points[i].x;
          float y = cloud->points[i].y;
          float z = cloud->points[i].z;

          float nx = cloud->points[i].normal_x;
          float ny = cloud->points[i].normal_y;
          float nz = cloud->points[i].normal_z;

          cloud->points[i].x = transform  (0,0)*x + transform  (0,1)*y + transform  (0,2)*z;
          cloud->points[i].y = transform  (1,0)*x + transform  (1,1)*y + transform  (1,2)*z;
          cloud->points[i].z = transform  (2,0)*x + transform  (2,1)*y + transform  (2,2)*z;

          cloud->points[i].normal_x = transform  (0,0)*nx + transform  (0,1)*ny + transform  (0,2)*nz;
          cloud->points[i].normal_y = transform  (1,0)*nx + transform  (1,1)*ny + transform  (1,2)*nz;
          cloud->points[i].normal_z = transform  (2,0)*nx + transform  (2,1)*ny + transform  (2,2)*nz;
      }
    }


    pcl::io::savePCDFileASCII ("new_cloud.pcd", *cloud);


}
