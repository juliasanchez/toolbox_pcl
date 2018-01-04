#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>

#include <pcl/registration/ndt.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main (int argc, char** argv)
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file1 \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from file1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file2 \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from file2.pcd" << std::endl;

  auto t_tot1 = std::chrono::high_resolution_clock::now();

  float sample= atof(argv[3]);

  if(sample!=0)
  {

    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud (input_cloud);
    uniform_sampling.setRadiusSearch (sample);
    std::cout << "before : " << input_cloud->size ()<<std::endl;
    uniform_sampling.filter (*input_cloud);
    std::cout << " after sampling : " <<input_cloud->size () << std::endl<<std::endl;

    uniform_sampling.setInputCloud (target_cloud);
    uniform_sampling.setRadiusSearch (sample);
    std::cout << "before : " << target_cloud->size ()<<std::endl;
    uniform_sampling.filter (*target_cloud);
    std::cout << " after sampling : " << target_cloud->size () << std::endl<<std::endl;
  }

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;


  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.0000001);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (atof(argv[6]));
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (atof(argv[5]));

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (atoi(argv[4]));
  // Setting point cloud to be aligned.
  ndt.setInputSource (input_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl<<std::endl;

  transform=ndt.getFinalTransformation();
  auto t_tot2 = std::chrono::high_resolution_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::milliseconds>(t_tot2-t_tot1).count();

  std::cout<<transform<<std::endl;
  std::cout<<"time : "<<time<<std::endl;



    std::string file_name;
    std::string file_name1;
    std::string file_name2;
    file_name=argv[1];
    size_t lastindex_point = file_name.find_last_of(".");
    size_t lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
    {
    lastindex_slash = 0;
    }

    file_name1 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
    file_name=argv[2];
    lastindex_point = file_name.find_last_of(".");
    lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
    {
    lastindex_slash = 0;
    }
    file_name2 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
    std::stringstream sstm;
    sstm.str("");
    sstm<<"/home/julia/Desktop/my_programs/toolbox_pcl/build-NDT/transformations/"<<file_name2<<"_"<<file_name1<<".txt";
    std::string file_name_tot = sstm.str();
    ofstream file (file_name_tot);
    file<<transform;
    file<<"\n";
    file<<time;


  return (0);
}
