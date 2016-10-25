#ifndef __SortList_H__
#define __SortList_H__

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;

class NodeSL{

	public:
		NodeSL(const cv::Point target, const double value){
			target_ = target;
			value_ = value;
			next_ = NULL;
		}

		NodeSL(){
			target_.x = 100000;
			target_.y = 100000;

			value_ = -100000;
		}

		~NodeSL(){
		}

		NodeSL* next_;

	    cv::Point target_;
	    double value_;

};

class SortList{

	public:

		SortList();
		SortList(const cv::Point, const double);
		~SortList();

		void push(const cv::Point, const double);
		cv::Point get_max();
		cv::Point get_min();
		bool empty();
		void clear();
		void print();


	private:

		NodeSL* first_node_;
};



#endif 

