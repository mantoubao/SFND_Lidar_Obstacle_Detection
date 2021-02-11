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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setInputCloud(cloud);
    voxelgrid.setLeafSize(filterRes,filterRes,filterRes);
    voxelgrid.filter(*filtered_cloud);

    typename pcl::PointCloud<PointT>::Ptr cloudregion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi(true);
    roi.setMax(maxPoint);
    roi.setMin(minPoint);
    roi.setInputCloud(filtered_cloud);
    roi.filter(*cloudregion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setInputCloud(cloudregion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(int index:indices){
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudregion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudregion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl; 

    return cloudregion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                                typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT>());

    // for(auto index: inliers->indices)
    // {
    //     roadCloud->points.push_back(cloud->points[index]);
    // }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    extract.setNegative(false);
    extract.filter(*roadCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    //inliers = RansacPlane(cloud,maxIterations,distanceThreshold);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);

    if(inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> points;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    KdTree* tree (new KdTree);
    for(int i = 0; i <cloud->points.size();++i)
    {
        const std::vector<float> point{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point,i);
        points.push_back(point);
    }

    std::vector<std::vector<int>> clusterIndices;
    clusterIndices=euclideanCluster(points,tree,clusterTolerance,minSize,maxSize);


    for(auto getIndices: clusterIndices )
    {
        if(getIndices.size()< minSize || getIndices.size()> maxSize)
            continue;

        typename pcl::PointCloud<PointT>::Ptr cloudcluster (new pcl::PointCloud<PointT>);

        for(auto index: getIndices)
        {
            cloudcluster->points.push_back(cloud->points[index]);
        }
        
        cloudcluster->width=cloudcluster->points.size();
        cloudcluster->height=1;
        cloudcluster->is_dense = true;

        clusters.push_back(cloudcluster);

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


template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>  
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    typename pcl::PointCloud<PointT>::Ptr pcinliers (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr pcoutliers (new pcl::PointCloud<PointT>());

	srand(time(NULL));	

while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
		{
			inliers.insert(rand()%cloud->points.size());
		}

		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		auto itr = inliers.begin();
		x1= cloud->points[*itr].x;
		y1= cloud->points[*itr].y;
		z1= cloud->points[*itr].z;

		advance(itr,1);

		x2= cloud->points[*itr].x;
		y2= cloud->points[*itr].y;
		z2= cloud->points[*itr].z;

		advance(itr,1);

		x3= cloud->points[*itr].x;
		y3= cloud->points[*itr].y;
		z3= cloud->points[*itr].z;

		float a = (y2-y1)*(z3-z1)- (z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1)- (x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1)- (y2-y1)*(x3-x1);
        float d = -(a*x1+b*y1+c*z1);


		for(int index=0;index<cloud->points.size();index++)
		{			
			PointT point_sample = cloud->points[index];
			float x=point_sample.x;
			float y=point_sample.y; 
			float z=point_sample.z;

			float d=fabs(a*x+b*y+c*z+d)/(sqrt(a*a+b*b+c*c));

			if (d<=distanceTol)
			{
				inliers.insert(index);
			}
		}

        if(inliers.size()>inliersResult.size())
		    inliersResult = inliers;

	}

    if(!inliersResult.empty()){ 
        for(int index=0;index<cloud->points.size();++index){
            if(inliersResult.count(index)){
                pcinliers->points.push_back(cloud->points.at(index));
            }
            else{
                pcoutliers->points.push_back(cloud->points.at(index));
            }
        }

    }
	
	return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(pcinliers,pcoutliers);

}



template <typename PointT>
std::vector<std::vector<int>> 
ProcessPointClouds<PointT>::euclideanCluster(std::vector<std::vector<float>> points, KdTree* tree, float distanceTol,int minSize, int maxSize)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>>  clusters;
	std::vector<bool> processed(points.size(),false);

	for(int i=0;i<points.size();i++){
		if(processed[i])
			continue;

		std::vector<int> cluster;
		proximity(tree,points,cluster,processed,i, distanceTol);
        if (cluster.size()>=minSize && cluster.size()<=maxSize){
                clusters.push_back(cluster);
        }		
	}
 
	return clusters;
}


template <typename PointT>
void ProcessPointClouds<PointT>::proximity(KdTree* tree, std::vector<std::vector<float>> points, 
                                                std::vector<int>& cluster, std::vector<bool>& processed, int index, float distanceTol){
	processed[index]=true;
	cluster.push_back(index);
	std::vector<int> nearby=tree->search(points[index],distanceTol); 

	for(int id:nearby){
		if(!processed[id])
			proximity(tree,points,cluster,processed,id,distanceTol);
	}

}