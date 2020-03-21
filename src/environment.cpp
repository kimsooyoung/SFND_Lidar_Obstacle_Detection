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
    
    // CLUSTERING OPTIONS
    bool doClustering = true;
    // RENDER OPTIONS
    bool renderScene = false;
    // PCA Box Optoins
    bool PCA_Box = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar =  new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =  pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    if (doClustering){
        int clusterId = 0;
        std::vector<Color> colors = {Color(1,0,1), Color(1,1,0), Color(0,1,1)};

        for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size ";   
            pointProcessor->numPoints(cluster);

            if(PCA_Box){
                BoxQ boxQ = pointProcessor->PCABoundingBox(cluster);
                renderBox(viewer,boxQ,clusterId);
            } else {
                Box box = pointProcessor->BoundingBox(cluster);
                renderBox(viewer,box,clusterId);
            }

            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

            ++clusterId;
        }
    }
    else {
        renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(0,1,0));
        renderPointCloud(viewer, segmentCloud.second, "ground", Color(1,0,0));
    }

    delete pointProcessor;
    delete lidar;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  renderPointCloud(viewer,inputCloud,"inputCloud");
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // Experiment with the ? values and find what works best
    float X = 30.0; 
    float Y = 6.5;
    float Z = 2.5;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f , Eigen::Vector4f(-(X / 2), -Y, -Z, 1), Eigen::Vector4f(X, Y, Z, 1));
    
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RANSAC3DSegment(filterCloud, 18, 0.3);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.33, 20, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";   
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessorI->BoundingBox(cluster);
        // todo: PCA Box with PointXYZI type
        // BoxQ boxQ = pointProcessorI->PCABoundingBox(cluster);
        
        // renderBox(viewer,boxQ,clusterId);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }

    // renderPointCloud(viewer,filterCloud,"filterCloud");
    // renderPointCloud(viewer,inputCloud,"inputCloud");
    // renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(0,0,1));
    renderPointCloud(viewer, segmentCloud.second, "ground", Color(0,1,0));

    // Box box;
    // box.x_min = -1.5;
    // box.x_max = 2.6;
    // box.y_min = -1.7;
    // box.y_max = 1.7;
    // box.z_min = -1;
    // box.z_max = -0.4;

    // renderBox(viewer,box,0);
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
    std::string MODE = "cityBlock";
    std::string STREAM_FILE = "../src/sensors/data/pcd/data_2";

    std::cout << "starting enviroment, MODE : " << MODE << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    if (MODE != "cityBlock"){
        if (MODE == "simpleHighway")
            simpleHighway(viewer);
        else if (MODE == "simpleCityBlock")
            cityBlock(viewer);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        }
        return 0;
    }
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(STREAM_FILE);
    auto streamIterator = stream.begin();
        
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }

    delete pointProcessorI;
}