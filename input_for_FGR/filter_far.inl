void filter_far(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float thresh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i=0; i<cloud_in->points.size(); i++)
    {
       float dist=sqrt(cloud_in->points[i].x*cloud_in->points[i].x + cloud_in->points[i].y*cloud_in->points[i].y +cloud_in->points[i].z*cloud_in->points[i].z);
       if(dist<thresh)
       {
           pcl::PointXYZ point;
           point.x=cloud_in->points[i].x;
           point.y=cloud_in->points[i].y;
           point.z=cloud_in->points[i].z;
           cloud_filtered->points.push_back(point);
       }
    }
    cloud_filtered->width = (uint32_t)  cloud_filtered->points.size();
    cloud_filtered->height = 1;
    pcl::copyPointCloud(*cloud_filtered,*cloud_in);
}

