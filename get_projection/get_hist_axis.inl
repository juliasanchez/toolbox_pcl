void get_hist_axis(std::vector<double> axis, pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string name)
{
    float v_axis;
    std::vector<float> projection(cloud->points.size ());


    for (size_t i = 0; i < cloud->points.size (); ++i)
    {        
        v_axis = cloud->points[i].x*axis[0] + cloud->points[i].y*axis[1]+cloud->points[i].z*axis[2];
        projection[i]=v_axis;
    }
    save_vector(projection, name);
}
