/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <math.h>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// Fill in this function
	int m_in=0;
	
	// For max iterations 
	for(int i=0;i<maxIterations;i++){
		int count=0;
		// Randomly sample subset and fit line
		int index=  (rand() % static_cast<int>(cloud->points.size() ));
		pcl::PointXYZ start= cloud->points[index];
		int index2=  (rand() % static_cast<int>(cloud->points.size() ));
		pcl::PointXYZ end= cloud->points[index2];
		float a= start.y -end.y;
		float b = end.x -start.x;
		float c= start.x*end.y - end.x*start.y;

		// Measure distance between every point and fitted line
		std::unordered_set<int> inl;
		for(int j=0;j<cloud->points.size();j++){
			float dist= fabs(a*cloud->points[j].x + b*cloud->points[j].y+c)/sqrt(pow(a,2) + pow(b,2));
				// If distance is smaller than threshold count it as inlier
			if(dist <= distanceTol){
				count++;
				inl.insert(j);

			}
		}
		if (count>m_in){
			inliersResult = inl;
		}

	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// Fill in this function
	int m_in=0;
	int index[3];
	
	// For max iterations 
	for(int i=0;i<maxIterations;i++){
		int count=0;
		// Randomly sample subset and fit plane
		index[0]=  (rand() % static_cast<int>(cloud->points.size() ));
		pcl::PointXYZ point1= cloud->points[index[0]];
		index[1]=  (rand() % static_cast<int>(cloud->points.size() ));
		pcl::PointXYZ point2= cloud->points[index[1]];
		index[2]=  (rand() % static_cast<int>(cloud->points.size() ));
		pcl::PointXYZ point3= cloud->points[index[2]];

		float a= (point2.y-point1.y)*(point3.z-point1.z) - (point2.z-point1.z)*(point3.y-point1.y);
		float b = (point2.z -point1.z)*(point3.x-point1.x) -(point2.x-point1.x)*(point3.z-point1.z);
		float c= (point2.x-point1.x)*(point3.y-point1.y)-(point2.y-point1.y)*(point3.x-point1.x);
		float d = - (a*point1.x + b*point1.y +c*point1.z);
		// Measure distance between every point and fitted line
		std::unordered_set<int> inl;
		for(int j=0;j<cloud->points.size();j++){
			float dist= fabs(a*cloud->points[j].x + b*cloud->points[j].y+c*b*cloud->points[j].z +d)/sqrt(pow(a,2) + pow(b,2) + pow(c,2));
				// If distance is smaller than threshold count it as inlier
			if(dist <= distanceTol){
				count++;
				inl.insert(j);
			}
		}
		if (count>m_in){
			inliersResult = inl;
		}

	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 1.0);

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
