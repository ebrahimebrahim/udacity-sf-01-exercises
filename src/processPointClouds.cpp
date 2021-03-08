// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud{new pcl::PointCloud<PointT>};
 
    pcl::CropBox<PointT> cb;
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(cloud);
    cb.filter(*filtered_cloud);

    cb.setMin(Eigen::Vector4f(-2.75,-1.5,-1.5,1));
    cb.setMax(Eigen::Vector4f(2.75,1.5,0.5,1));
    cb.setInputCloud(filtered_cloud);
    cb.setNegative(true);
    cb.filter(*filtered_cloud);

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*filtered_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr cloud_noad{new pcl::PointCloud<PointT>}, cloud_road{new pcl::PointCloud<PointT>};
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*cloud_road);
    extract.setNegative(true);
    extract.filter(*cloud_noad);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_noad, cloud_road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
        std::cerr << "Could not estimate a planar model.\n";
    


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> clusters_indices;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree{new pcl::search::KdTree<PointT>};
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters_indices);
    for (auto & cluster_indices : clusters_indices) {
        typename pcl::PointCloud<PointT>::Ptr cluster{new pcl::PointCloud<PointT>};
        for (auto index : cluster_indices.indices){
            cluster->push_back((*cloud)[index]);
        }
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    const auto & pts = cluster->points;
    Eigen::Matrix<float,Eigen::Dynamic,2> pts_mat(pts.size(),2);
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto & pt : pts) centroid += pt.getVector3fMap();
    centroid /= float(pts.size());
    for (int row = 0; row < pts.size(); ++row) {
        auto q = pts[row].getVector3fMap() - centroid;
        pts_mat(row,0) = q[0];
        pts_mat(row,1) = q[1];
    }
    Eigen::EigenSolver<Eigen::Matrix2f> es;
    es.compute(pts_mat.transpose() * pts_mat);
    Eigen::Matrix2f ev = es.eigenvectors().real();
    auto pts_tnsfm = pts_mat * ev.inverse().transpose();

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    float x1 = pts_tnsfm.col(0).minCoeff();
    float x2 = pts_tnsfm.col(0).maxCoeff();
    float y1 = pts_tnsfm.col(1).minCoeff();
    float y2 = pts_tnsfm.col(1).maxCoeff();
    float z1 = minPoint.z - centroid[2];
    float z2 = maxPoint.z - centroid[2];

    Eigen::Vector2f xy_center = ev * Eigen::Vector2f((x1+x2)/2.0 , (y1+y2)/2.0);
    

    BoxQ box;
    box.cube_length = x2-x1;
    box.cube_width = y2-y1;
    box.cube_height = z2 - z1;
    box.bboxTransform = centroid + Eigen::Vector3f(xy_center[0] , xy_center[1] , (z1 + z2)/2.0);
    box.bboxQuaternion = Eigen::Quaternionf( Eigen::AngleAxisf(atan2(ev(1,0),ev(0,0)), Eigen::Vector3f::UnitZ()) );

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}