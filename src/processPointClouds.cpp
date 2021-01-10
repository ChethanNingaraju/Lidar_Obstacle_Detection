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

    //create voxel grid filtering object
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud);

    pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setInputCloud(cloud);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.filter(*cloud);

    //removing roof points
    std::vector<int> roofIndices;
    pcl::CropBox<PointT> roof(true);
    roof.setInputCloud(cloud);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.filter(roofIndices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: roofIndices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr inlierCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr restCloud(new pcl::PointCloud<PointT>());

    int inlierIndex = 0;
    std::sort(inliers->indices.begin(),inliers->indices.end());
    for(int i = 0; i < cloud->size(); i++)
    {
        if(i == inliers->indices[inlierIndex])
        {
            inlierCloud->push_back(cloud->at(i));
            inlierIndex++;
        }
        else
            restCloud->push_back(cloud->at(i));

    }
    std::cout << "Ground plane size = " << inlierCloud->size() << " Obstacle point cloud size = " << restCloud->size() << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(restCloud, inlierCloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    int max_inlier = 0;
    float ABest = 0, BBest = 0, CBest = 0, DBest = 0;


    // For max iterations
    for (int i = 0; i < maxIterations; i++)
    {
        PointT P1 = cloud->at((rand() % cloud->size()));
        PointT P2 = cloud->at((rand() % cloud->size()));
        PointT P3 = cloud->at((rand() % cloud->size()));

        Vect3 a((P2.x - P1.x) , (P2.y - P1.y), (P2.z - P1.z));
        Vect3 b((P3.x - P1.x) , (P3.y - P1.y), (P3.z - P1.z));

        Vect3 crossProduct(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);

        float A = crossProduct.x;
        float B = crossProduct.y;
        float C = crossProduct.z;
        float D = -(A * P1.x + B * P1.y + C * P1.z);

        int tot_inlier = 0;
        float den = sqrt(A * A + B * B + C * C);
        if(den == 0)
            continue;
        for(int j = 0; j < cloud->size(); j++)
        {
             if((fabs((A * cloud->at(j).x + B * cloud->at(j).y + C * cloud->at(j).z + D)/den)) <  distanceTol)
             {
                 tot_inlier++;
             }
        }
        if(tot_inlier > max_inlier)
        {
            max_inlier = tot_inlier;
            ABest = A;
            BBest = B;
            CBest = C;
            DBest = D;
        }
        //std::cout << "Total Inlier = " << tot_inlier << std::endl;

    }

    float den = sqrt(ABest * ABest + BBest * BBest + CBest * CBest);
    for(int j = 0; j < cloud->size(); j++)
    {
        if((fabs((ABest * cloud->at(j).x + BBest * cloud->at(j).y + CBest * cloud->at(j).z + DBest)/den)) <  distanceTol)
        {
            inliersResult.insert(j);
        }
    }

    return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    //Get indices of ground plane
    std::unordered_set<int> groundPlaneIndices = Ransac<PointT>(cloud,maxIterations,distanceThreshold);

    std::cout << "groundPlaneIndices.size() " << groundPlaneIndices.size() << std::endl;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    for(auto itr = groundPlaneIndices.begin(); itr != groundPlaneIndices.end(); ++itr)
    {
        inliers->indices.push_back(*itr);
    }
    std::cout << "cloud->points.size() " << cloud->points.size() << "inliers->indices.size() " << inliers->indices.size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


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

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //creating Kd-tree object
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

      for (int index: it->indices)
      {
            cloud_cluster->points.push_back(cloud->points[index]); //*
      }
          cloud_cluster->width = cloud_cluster->size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
      clusters.push_back(cloud_cluster);

          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
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

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MyClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointIndices> cluster_indices;
    KdTree<PointT>* tree = new KdTree<PointT>;

    for (int i=0; i < cloud->points.size(); i++)
        tree->insert(cloud->points[i],i);
    cluster_indices = euclideanCluster(cloud, tree, 0.5);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        if(it->indices.size() < minSize || it->indices.size() > maxSize )
            continue;
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

      for (int index: it->indices)
      {
            cloud_cluster->points.push_back(cloud->points[index]); //*
      }
          cloud_cluster->width = cloud_cluster->size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          clusters.push_back(cloud_cluster);

          //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
