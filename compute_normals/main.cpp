#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);

    if (pcl::io::loadPCDFile<pcl::PointNormal> (argv[1], *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setViewPoint (0, 0, 0);

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
  ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setKSearch(100);

  // Compute the features
  ne.compute (*cloud);

  int n = 0;

  for (int i = 0 ; i<cloud->size(); ++i)
  {
    if(abs(cloud->points[i].normal_x)<0.9 && abs(cloud->points[i].normal_y)<0.9)
      ++n;
  }

  std::cout<<"number of false normals :"<<n<<std::endl<<std::endl;
  pcl::io::savePCDFileASCII ("result.pcd", *cloud);
}
