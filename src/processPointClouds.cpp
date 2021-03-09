// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"
#include <unordered_set>


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::DownsampleCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes)
{
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud{new pcl::PointCloud<PointT>};

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*filtered_cloud);

    return filtered_cloud;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::CropCloud(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud{new pcl::PointCloud<PointT>};
 
    pcl::CropBox<PointT> cb;
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(cloud);
    cb.filter(*filtered_cloud);

    // Remove roof points
    cb.setMin(Eigen::Vector4f(-2.75,-1.5,-1.5,1));
    cb.setMax(Eigen::Vector4f(2.75,1.5,0.5,1));
    cb.setInputCloud(filtered_cloud);
    cb.setNegative(true);
    cb.filter(*filtered_cloud);

    return filtered_cloud;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    auto filtered_cloud = DownsampleCloud( CropCloud(cloud,minPoint,maxPoint), filterRes );

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

// Converts a point that has x,y,z members to an Eigen library vector
template<typename PointT>
Eigen::Vector3f pt_to_vec(const PointT & pt) {
    return Eigen::Vector3f(pt.x, pt.y, pt.z);
}

// Should give the distance from pt to the plane containing p1, p2, and p3
static float dist_pt_to_plane(const Eigen::Vector3f &  pt, const Eigen::Vector3f & p1, const Eigen::Vector3f & p2, const Eigen::Vector3f & p3) {
	auto normal = (p2 - p1).cross(p3 - p1);
	return fabs(normal.dot(pt-p1))/normal.norm();
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));

    Eigen::Vector3f best_p1, best_p2, best_p3;
	int most_inliers = 0;

    // Downsample cloud just for the purpose of fitting a plane
    auto dcloud = DownsampleCloud(cloud, 3.0);

    // Turn all cloud points into Eigen library vectors
    std::vector<Eigen::Vector3f> pts;
    for (const auto & pt : dcloud->points)
        pts.push_back(pt_to_vec<PointT>(pt));

	for (int j=0; j<maxIterations; ++j){
		
		// Choose three random distinct indices
		std::unordered_set<int> initial_indices{};
		while(initial_indices.size() < 3)
			initial_indices.insert(rand() % pts.size());

		auto itr = initial_indices.begin();
		auto & p1 = pts[*itr];
		++itr;
		auto & p2 = pts[*itr];
		++itr;
		auto & p3 = pts[*itr];

		int num_inliers = 0;
		for (const auto & pt : pts){
			if (dist_pt_to_plane(pt, p1, p2, p3) < distanceThreshold)
				num_inliers++;
		}
		if (num_inliers > most_inliers){
			most_inliers = num_inliers;
			best_p1 = p1;
			best_p2 = p2;
			best_p3 = p3;
		}

        // If there are "enough"  inliers already, then stop iterating
        if (float(most_inliers)/float(dcloud->points.size()) > 0.8)
            break;

	}

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	for (int i=0; i<cloud->points.size(); ++i)
		if (dist_pt_to_plane(pt_to_vec(cloud->points[i]), best_p1, best_p2, best_p3) < distanceThreshold)
			inliers->indices.push_back(i);

    std::cout << float(most_inliers)/float(dcloud->points.size()) << " of segmented pts are inliers\n";
	
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
static void proximity_recurse(typename pcl::PointCloud<PointT>::Ptr cloud, const KdTree* tree, float distanceTol, int point_index, std::vector<int> & cluster, std::vector<bool> & processed) {
	processed[point_index] = true;
	cluster.push_back(point_index);
	for (auto j : tree->search({cloud->points[point_index].x, cloud->points[point_index].y, cloud->points[point_index].z},distanceTol)) 
		if (!processed[j])
			proximity_recurse<PointT>(cloud, tree, distanceTol, j, cluster, processed);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree* tree = new KdTree;
  
    for (int i=0; i<cloud->points.size(); i++) 
    	tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);

    std::vector<std::vector<int>> clusters_indices;

	std::vector<bool> processed(cloud->points.size(),false);
	for (int i=0; i< cloud->points.size(); ++i) {
		if (processed[i])
			continue;
		std::vector<int> cluster{};
		proximity_recurse<PointT>(cloud, tree, clusterTolerance, i, cluster, processed);
		clusters_indices.push_back(cluster);
	}	
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (auto & cluster_indices : clusters_indices) {
        if ((cluster_indices.size() < minSize) || (cluster_indices.size() > maxSize))
            continue;
        typename pcl::PointCloud<PointT>::Ptr cluster{new pcl::PointCloud<PointT>};
        for (auto index : cluster_indices){
            cluster->push_back((*cloud)[index]);
        }
        clusters.push_back(cluster);
    }

    delete tree;

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