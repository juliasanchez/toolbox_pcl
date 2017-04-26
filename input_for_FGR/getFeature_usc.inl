void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr features)
{
        pcl::UniqueShapeContext<pcl_point, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> feature_estimation;
        feature_estimation.setSearchMethod(pcl::search::KdTree<pcl_point>::Ptr(new pcl::search::KdTree<pcl_point>));
        feature_estimation.setRadiusSearch(descriptor_radius);
        feature_estimation.setInputCloud(cloudin);
        feature_estimation.setRadiusSearch(descriptor_radius); // neigbours to take into account
        feature_estimation.setMinimalRadius(descriptor_radius / 10.0); //to avoid being too sensitive in bins close to center
        feature_estimation.setPointDensityRadius(descriptor_radius / 2.0); //radius for local density
        feature_estimation.setLocalRadius(descriptor_radius); //radius for local frame
        feature_estimation.compute(*features);
}
