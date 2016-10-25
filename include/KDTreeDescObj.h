#ifndef __KDTreeDescObj_H__
#define __KDTreeDescObj_H__

// Standard libs
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

// OpenCV libs
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

// Custom libs
#include <enum_header.h>

// ROS libs
#include <ros/ros.h>

#define INVALID_VALUE -1

using namespace std;


class NodeObj{

	public:
		NodeObj(const cv::Point target, const colors color, const shapes shape, const float area, const int ID, const ros::Time stamp){
			
			left_ = NULL;
			right_ = NULL;
					
			target_ = target;

			color_ = color;

			shape_ = shape;
			area_  = area;
			ID_  = ID;

			stamp_ = stamp;
		}

		NodeObj(){

			left_ = NULL;
			right_ = NULL;
					
			target_.x = 100000;
			target_.y = 100000;

			color_ = BLUE;

			shape_ = SQUARE;
			area_  = INVALID_VALUE;
			ID_  = INVALID_VALUE;

			stamp_ = ros::Time::now();
		}

		~NodeObj(){
			delete left_;
			delete right_;
		}

		NodeObj* left_;
		NodeObj* right_;
				
		cv::Point target_;
		colors color_;
		shapes shape_;
		float area_;
		int ID_;

		ros::Time stamp_;
};

class KDTreeDescObj{

	public:

		KDTreeDescObj();
		// KDTreeDescObj(const cv::Point, const colors, const shapes, const float);
		~KDTreeDescObj();

		void push(const cv::Point, const colors, const  shapes, const float);
		// void pop(ID);

		cv::Point nearestObj(const cv::Point, const colors, const  shapes, const float);

		vector<NodeObj> neighbourhoodObj(const cv::Point, const float, NodeObj* KDTree = NULL);
		
		void updateKalman();
		bool empty();
		void clear();
		void print();
		void singleNodePrint(NodeObj*);

	private:

		void recursivePrint(NodeObj*);
		vector<NodeObj> recursiveNeighbourhoodObj(const cv::Point, const float, NodeObj* KDTree, bool layer);

		NodeObj* first_node_;
		int numNode;
};



#endif 

