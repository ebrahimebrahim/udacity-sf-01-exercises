/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars, 0);

    auto point_cloud = lidar->scan();

    // renderRays(viewer, lidar->position,point_cloud);
    // renderPointCloud(viewer, point_cloud, "peup");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> process_point_clouds{};
    auto seg_pair = process_point_clouds.SegmentPlane(point_cloud,100,0.2);
    auto cloud_noad = seg_pair.first;
    auto cloud_road = seg_pair.second;

    renderPointCloud(viewer, cloud_noad, "noad",Color(1,0,1));
    renderPointCloud(viewer, cloud_road, "road",Color(1,1,1));

    auto clusters = process_point_clouds.Clustering(cloud_noad,1.0, 3, 9999);
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(0,1,1), Color(1,0,1)};
    for (int i=0; i<clusters.size(); ++i){
        const Color & color = colors[i%colors.size()];
        renderPointCloud(viewer, clusters[i], "obstacle_"+std::to_string(i),color);

        BoxQ box = process_point_clouds.BoundingBoxQ(clusters[i]);
        renderBox(viewer,box,i,color);
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    
    ProcessPointClouds<pcl::PointXYZI> process_point_clouds{};

    auto cloud = process_point_clouds.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    auto filtered_cloud = process_point_clouds.FilterCloud(cloud, 0.4, Eigen::Vector4f(-20,-15,-5,1),Eigen::Vector4f(40,15,5,1));
    // renderPointCloud(viewer, filtered_cloud, "cloud");
    
    auto seg_pair = process_point_clouds.SegmentPlane(filtered_cloud,100,0.2);
    auto cloud_noad = seg_pair.first;
    auto cloud_road = seg_pair.second;

    renderPointCloud(viewer, cloud_noad, "noad",Color(1,0,1));
    renderPointCloud(viewer, cloud_road, "road",Color(1,1,1));
    
    

}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}