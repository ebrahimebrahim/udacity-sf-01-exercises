/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <Eigen/Dense>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

// Should give the distance from pt to the line joining p1 and p2
// This will assume that the z-coordinate of all the points is 0, treating them basically as 2D points
float dist_pt_to_line(pcl::PointXYZ pt, pcl::PointXYZ p1, pcl::PointXYZ p2) {
	float A = p1.y - p2.y;
	float B = p2.x - p1.x;
	float C = p1.x * p2.y  -  p2.x * p1.y;
	return fabs(A*pt.x + B*pt.y + C)/sqrt(A*A + B*B);
}

// Should give the distance from pt to the plane containing p1, p2, and p3
float dist_pt_to_plane(pcl::PointXYZ pt, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) {
	pcl::Vector3fMap p1v = p1.getVector3fMap(); // This converts points to Eigen library vectors
	pcl::Vector3fMap p2v = p2.getVector3fMap();
	pcl::Vector3fMap p3v = p3.getVector3fMap();
	pcl::Vector3fMap ptv = pt.getVector3fMap();
	auto v1 = p2v - p1v;
	auto v2 = p3v - p1v;
	auto normal = v1.cross(v2);
	float D = - normal.dot(p1v);
	return fabs(normal.dot(ptv) + D)/normal.norm();
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	pcl::PointXYZ best_p1, best_p2, best_p3;
	int most_inliers = 0;

	for (int j=0; j<maxIterations; ++j){
		
		// Choose three random distinct indices
		std::unordered_set<int> initial_indices{};
		while(initial_indices.size() < 3)
			initial_indices.insert(rand() % cloud->points.size());

		auto itr = initial_indices.begin();
		pcl::PointXYZ p1 = cloud->points[*itr];
		++itr;
		pcl::PointXYZ p2 = cloud->points[*itr];
		++itr;
		pcl::PointXYZ p3 = cloud->points[*itr];

		int num_inliers = 0;
		for (auto & point : cloud->points){
			if (dist_pt_to_plane(point, p1, p2, p3) < distanceTol)
				num_inliers++;
		}
		if (num_inliers > most_inliers){
			most_inliers = num_inliers;
			best_p1 = p1;
			best_p2 = p2;
			best_p3 = p3;
		}

	}

	for (int i=0; i<cloud->points.size(); ++i){
		if (dist_pt_to_plane(cloud->points[i], best_p1, best_p2, best_p3) < distanceTol)
			inliersResult.insert(i);
	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
