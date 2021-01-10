#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"


template<typename PointT>
void render2DTree(Node<PointT>* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

    if(node!=NULL)
    {
        Box upperWindow = window;
        Box lowerWindow = window;
        // split on x axis
        if(depth%2==0)
        {
            viewer->addLine(pcl::PointXYZ(node->point.x, window.y_min, 0),pcl::PointXYZ(node->point.x, window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
            lowerWindow.x_max = node->point.x;
            upperWindow.x_min = node->point.x;
        }
        // split on y axis
        else
        {
            viewer->addLine(pcl::PointXYZ(window.x_min, node->point.y, 0),pcl::PointXYZ(window.x_max, node->point.y, 0),1,0,0,"line"+std::to_string(iteration));
            lowerWindow.y_max = node->point.y;
            upperWindow.y_min = node->point.y;
        }
        iteration++;

        render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
        render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


    }

}


template<typename PointT>
void clusterHelper(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices& cluster, KdTree<PointT>* tree,int idx, float distanceTol, std::vector<bool>& isProcessed)
{
    isProcessed[idx] = true;
    cluster.indices.push_back(idx);

    std::vector<int> nearby = tree->search(cloud->points[idx],distanceTol);

    for(int index: nearby)
    {
        if(isProcessed[index])
            continue;
        isProcessed[index] = true;
        cluster.indices.push_back(index);
        clusterHelper<PointT>(cloud,cluster,tree,index,distanceTol,isProcessed);
    }

}


template<typename PointT>
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::cout << "*******************************************************************\n";
    std::vector<pcl::PointIndices> clusters;
    std::vector<bool> isProcessed(cloud->points.size(), false);
    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(!isProcessed[i])
        {
            isProcessed[i] = true;
            pcl::PointIndices cluster;
            clusterHelper<PointT>(cloud,cluster,tree,i,distanceTol,isProcessed);
            clusters.push_back(cluster);
        }

    }

    return clusters;

}
