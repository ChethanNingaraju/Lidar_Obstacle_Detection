/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef __KDTREE_H
#define __KDTREE_H

#include "../../render/render.h"


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
    PointT point;
	int id;
	Node* left;
	Node* right;

    Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};
template<typename PointT>
struct KdTree
{
    Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

    void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

        Node<PointT> *newNode = new Node<PointT>(point, id);
        if(root == NULL)
        {
            root = newNode;
            return;
        }
        //traverse through the tree to find suitable insertion point
        Node<PointT> *cur_node = root;
        int curDepth = 0;
        bool insertDone = false;
        while(!insertDone)
        {
            bool condition = false;
            if(curDepth % 2 == 0) //alternating between comparing x and y
                condition = point.x > cur_node->point.x;
            else
                condition = point.y > cur_node->point.y;

            if(condition)
            {
                if(cur_node->right == NULL)//insert if no node on right
                {
                    cur_node->right = newNode;
                    insertDone = true;
                }
                else
                {
                    cur_node = cur_node->right;
                    curDepth++;
                }

            }
            else//go left
            {
                if(cur_node->left == NULL)//insert if no node on right
                {
                    cur_node->left = newNode;
                    insertDone = true;
                }
                else
                {
                    cur_node = cur_node->left;
                    curDepth++;
                }
            }
        }
	}
    void searchHelper(PointT target, Node<PointT>* curNode, int curDepth, float distanceTol, std::vector<int>& ids)
    {

        if(curNode != NULL)
        {
            float curDist = sqrt((curNode->point.x - target.x) * (curNode->point.x - target.x) + (curNode->point.y - target.y) * (curNode->point.y - target.y));
            if(curDist <= distanceTol)
                ids.push_back(curNode->id);

            if(curDepth % 2 == 0)
            {
                if(target.x - distanceTol <= curNode->point.x)
                    searchHelper(target,curNode->left,curDepth+1,distanceTol,ids);
                if(target.x + distanceTol > curNode->point.x)
                    searchHelper(target,curNode->right,curDepth+1,distanceTol,ids);
            }
            else
            {
                if(target.y - distanceTol <= curNode->point.y)
                    searchHelper(target,curNode->left,curDepth+1,distanceTol,ids);
                if(target.y + distanceTol > curNode->point.y)
                    searchHelper(target,curNode->right,curDepth+1,distanceTol,ids);
            }

        }
    }
	// return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
        //start with root node
        searchHelper(target, root,0,distanceTol,ids);


		return ids;
	}
	

};
#endif



