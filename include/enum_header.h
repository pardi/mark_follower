#ifndef __enum_header_H__
#define __enum_header_H__


enum th_color { 
	CAM_WHITE,
	CAM_BLACK
};

enum colors {
	BLUE,
	RED,
	GREEN,
	BLACK
};

enum shapes {
	SQUARE,
	RECTANGLE,
	CIRCLE
};

struct descriptor{
	cv::Mat img;
	cv::Point origin ;
	double alpha;
	shapes shape;
	colors color;
	bool is_square;
};

#endif