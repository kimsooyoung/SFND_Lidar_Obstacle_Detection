/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "render/render.h"
#include <unordered_set>
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		int cloud_size = cloud->points.size();

		// Randomly sample subset and fit line
		while (inliers.size() < 3)
			inliers.insert(rand() % (cloud_size));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		float A, B, C, D;
		float point_x, point_y, point_z, d;
		int count = 0;
		int best_count = 0;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;

		++itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		++itr;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		D = -1 * (A * x1 + B * y1 + C * z1);

		for (int i = 0; i < cloud_size; i++)
		{
			point_x = cloud->points[i].x;
			point_y = cloud->points[i].y;
			point_z = cloud->points[i].z;

			// Measure distance between every point and fitted line
			d = fabs(A * point_x + B * point_y + C * point_z + D) / sqrt(A * A + B * B + C * C);
			// If distance is smaller than threshold count it as inlier
			if (d < distanceTol)
				inliers.insert(i);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	std::cout << inliersResult.size() << std::endl;

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		int cloud_size = cloud->points.size();

		// Randomly sample subset and fit line
		int rand_1 = rand() % (cloud_size);
		int rand_2 = rand() % (cloud_size);

		while (rand_1 == rand_2)
			rand_2 = rand() % (cloud_size);

		// std::cout << cloud_size << " , " << rand_1 << " , " << rand_2 << std::endl;

		inliers.insert(rand_1);
		inliers.insert(rand_2);

		float x1, x2, y1, y2;
		float A, B, C;
		float point_x, point_y, d;
		int count = 0;
		int best_count = 0;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;

		++itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		A = (y1 - y2);
		B = (x2 - x1);
		C = (x1 * y2 - x2 * y1);

		for (int i = 0; i < cloud_size; i++)
		{
			point_x = cloud->points[i].x;
			point_y = cloud->points[i].y;

			// Measure distance between every point and fitted line
			d = fabs(A * point_x + B * point_y + C) / sqrt(A * A + B * B);

			// If distance is smaller than threshold count it as inlier
			if (d < distanceTol)
				inliers.insert(i);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	std::cout << inliersResult.size() << std::endl;

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 500, 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, 200, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}