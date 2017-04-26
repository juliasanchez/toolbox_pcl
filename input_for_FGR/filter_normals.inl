void filter_normals(pcl::PointCloud<pcl_point>::Ptr normals, float radius, float perc, pcl::PointIndices::Ptr removed_indices)
{
    pcl::RadiusOutlierRemoval<pcl_point> rorfilter(true);
    rorfilter.setInputCloud (normals);
    rorfilter.setRadiusSearch (radius);
    rorfilter.setMinNeighborsInRadius (floor(normals->points.size()*perc )  );
    rorfilter.filter (*normals);
    pcl::IndicesConstPtr ind=rorfilter.getRemovedIndices();
    removed_indices->indices = *ind;
}
