void pcn2n(pcl::PointCloud<pcl::PointNormal>::Ptr cloudin, pcl::PointCloud<pcl::Normal>::Ptr cloudout)
{
    cloudout->width    = cloudin->points.size();
    cloudout->height   = 1;
    cloudout->is_dense = false;
    cloudout->points.resize (cloudin->width * cloudin->height);


    for (int i=0; i<cloudin->points.size(); i++)
    {
        cloudout->points[i].normal_x=cloudin->points[i].normal_x;
        cloudout->points[i].normal_y=cloudin->points[i].normal_y;
        cloudout->points[i].normal_z=cloudin->points[i].normal_z;
    }

}
