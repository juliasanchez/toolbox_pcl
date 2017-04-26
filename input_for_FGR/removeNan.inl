void removeNan(pcl::PointCloud<pcl_point>::Ptr cloud, pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr features)
{
    pcl::PointCloud<pcl_point>::Ptr cloud_temp(new pcl::PointCloud<pcl_point>);
     pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr feat_temp (new pcl::PointCloud<pcl::UniqueShapeContext1960>);

    for (int i=0; i<features->points.size(); i++)
    {
        if(features->points[i].descriptor[0]==features->points[i].descriptor[0])
        {
            feat_temp->points.push_back(features->points[i]);
            cloud_temp->points.push_back(cloud->points[i]);
        }
    }

    cloud_temp->width    = cloud_temp->points.size();
    cloud_temp->height   = 1;
    cloud_temp->is_dense = false;
    cloud_temp->points.resize (cloud_temp->width * cloud_temp->height);

    feat_temp->width    = cloud_temp->points.size();
    feat_temp->height   = 1;
    feat_temp->is_dense = false;
    feat_temp->points.resize (feat_temp->width * feat_temp->height);

    *cloud=*cloud_temp;
    *features=*feat_temp;

}
