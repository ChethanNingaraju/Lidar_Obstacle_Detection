/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <ctime>

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
             if((fabs((A * cloud->at(j).x + B * cloud->at(j).y + C * cloud->at(i).z + D)/den)) <  distanceTol)
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
        std::cout << "Total Inlier = " << tot_inlier << std::endl;

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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    int max_inlier = 0;
    float ABest = 0, BBest = 0, CBest = 0;

    std::cout << "cloud size = " << cloud->at(3).x << " " << cloud->points[3].x << std::endl;;
	// For max iterations 
    for (int i = 0; i < maxIterations; i++)
    {
        pcl::PointXYZ P1 = cloud->at((rand() % cloud->size()));
        pcl::PointXYZ P2 = cloud->at((rand() % cloud->size()));
        float A = (P1.y - P2.y);
        float B = P2.x - P1.x;
        float C = P1.x * P2.y - P2.x * P1.y;

        int tot_inlier = 0;
        float den = sqrt(A * A + B * B);
        if(den == 0)
            continue;
        for(int j = 0; j < cloud->size(); j++)
        {
            std::cout << "Distabce = " << distanceTol << " " << fabs((A * cloud->at(j).x + B * cloud->at(j).y + C)/den) << std::endl;
             if((fabs((A * cloud->at(j).x + B * cloud->at(j).y + C)/den)) <  distanceTol)
                 tot_inlier++;
        }
        if(tot_inlier > max_inlier)
        {
            max_inlier = tot_inlier;
            ABest = A;
            BBest = B;
            CBest = C;
        }
        std::cout << "Total Inlier = " << tot_inlier << std::endl;

    }

    float den = sqrt(ABest * ABest + BBest * BBest);
    for(int j = 0; j < cloud->size(); j++)
    {
        if((abs((ABest * cloud->at(j).x + BBest * cloud->at(j).y + CBest)/den)) <  distanceTol)
        {
            inliersResult.insert(j);
        }
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
    std::unordered_set<int> inliers = Ransac<pcl::PointXYZ>(cloud, 100, 0.3);

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
