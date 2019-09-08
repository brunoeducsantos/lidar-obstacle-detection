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


    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);    
    
    //Region based interest

    pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud_filtered);
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>());
    boxFilter.filter(*cloud_filtered2);

    // TODO Crop roof top points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1.,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud(cloud_filtered2);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (const auto elem: indices) 
        inliers->indices.push_back(elem);
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);
    
    std::cout<<"POINTS"<<cloudRegion->points.size()<<std::endl;


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud ) 
{   
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT> ());

    for (int index: inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    //Extract inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segs(obsCloud,planeCloud);
    return segs;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
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
		PointT point1= cloud->points[index[0]];
		index[1]=  (rand() % static_cast<int>(cloud->points.size() ));
		PointT point2= cloud->points[index[1]];
		index[2]=  (rand() % static_cast<int>(cloud->points.size() ));
		PointT point3= cloud->points[index[2]];

        Eigen::Vector3d v1(point2.x-point1.x,point2.y-point1.y,point2.z-point1.z);
        Eigen::Vector3d v2(point3.x-point1.x,point3.y-point1.y,point3.z-point1.z);
        Eigen::Vector3d result(v1.cross(v2));
        float a = result(0);
        float b= result(1);
        float c= result(2);
        float d = -result.dot(Eigen::Vector3d(point1.x,point1.y,point1.z));

		// Measure distance between every point and fitted line
		std::unordered_set<int> inl;
		for(int j=0;j<cloud->points.size();j++){
			float dist= fabs(a*cloud->points[j].x + b*cloud->points[j].y+c*cloud->points[j].z +d)/sqrt(pow(a,2) + pow(b,2) + pow(c,2));
				// If distance is smaller than threshold count it as inlier
			if(dist <= distanceThreshold){
				count++;
				inl.insert(j);
			}
		}
		if (count>m_in){
			inliersResult = inl;
		}
	}

    //store inside inliers point indices format
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (const auto& elem: inliersResult) {
        inliers->indices.push_back(elem);
    }


	// use inlierars to separate clouds
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int indice, std::vector<int> &cluster, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree,  float distanceTol, std::vector<bool> &processed){
		processed[indice]= true;
		cluster.push_back(indice);
        std::vector<float> pt; 
        pt.push_back(cloud->points[indice].x);
        pt.push_back(cloud->points[indice].y);
        pt.push_back(cloud->points[indice].z);
		std::vector<int> nearby_points= tree->search(pt,distanceTol);
		for (auto index: nearby_points){
			if(!processed[index]){
				proximity(index, cluster, cloud, tree, distanceTol, processed);
			}
		}
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

     // Creating the KdTree object for the search method of the extraction
    KdTree *tree(new KdTree);
    std::vector<std::vector<int>> clusters_indices;
	std::vector<bool> processed(cloud->points.size());
	std::fill (processed.begin(),processed.end(),false);
	for(int i=0;i<cloud->points.size();i++){
        std::vector<float> point{cloud->points[i].x,cloud->points[i].y, cloud->points[i].z};
        tree->insert(point,i);
    }

    for(int i=0;i<cloud->points.size();i++){
		if(processed[i]) continue;
		std::vector <int> clust;
		proximity(i, clust, cloud, tree, clusterTolerance, processed);
        if((clust.size()>=minSize) &&  (clust.size()<= maxSize)){ 
            clusters_indices.push_back(clust);
        }
	}

    for(const auto& elem: clusters_indices){
        typename pcl::PointCloud<PointT>::Ptr clust(new typename pcl::PointCloud<PointT>);
        
        for( int i : elem)
            clust->points.push_back(cloud->points[i]);

        clust->width= clust->points.size();
        clust->height= 1;
        clust->is_dense = true;
        clusters.push_back(clust);
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