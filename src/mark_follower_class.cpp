#include "mark_follower_class.h" 

using namespace cam_vid;

mark_follower_class::mark_follower_class(ros::NodeHandle* n, const bool verbose){

	// TRIAL

	// KDTreeDescObj p;

	// p.push(Point(0, 0), BLUE, CIRCLE, 10.3);
	// p.push(Point(0, 50), RED, CIRCLE, 10.3);
	// p.push(Point(0, 10), RED, CIRCLE, 10.3);
	// p.push(Point(0, 100), BLUE, CIRCLE, 10.3);

	// cout << "nearest " <<  p.nearestObj(Point(0, 6), RED, CIRCLE, 10.3) << endl;

	// cout << "neighbourhood " << endl; 
	// vector<NodeObj> v = p.neighbourhoodObj(Point( 0, 2), 120);
	// cout << v.size() << endl;
	// for (int i = 0; i < v.size(); i++)
	// 	cout << v[i].target_ << " ";

	// cout << endl << "print " << endl;

	// p.print();
	// cout << "postPrint" << endl;

	// --- 

	// Initialize vars

	verbose_ = verbose;

	ocvMat_ = Mat(CAMERA_VID_WIDTH, CAMERA_VID_HEIGTH, CV_8UC3);

	// Kalman Filter Init bool vars
	state_kf_ = false;


	// ---------------------------------------------------------------------------------------- >> VIDEO REC << ----------------------------------------------------------------------------------------

	// int ex = static_cast<int>(CV_FOURCC('M','P','4','V')); 

	// // Transform from int to char via Bitwise operators
	// char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
	
	// string number;
	// stringstream strstream;
	// strstream << (long int) time(NULL);
	// strstream >> number;

	// String str = VIDEO_NAME;
	// str += "_" + number + ".avi";

	// outVid_.open(str.c_str(), ex, 15.0, Size( CAMERA_VID_HEIGTH, CAMERA_VID_WIDTH ), true);

	// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


	// Ros operations
 
	n_ = n;

	// Get params

	n_->param("/mark_follower/challenge", challenge_, false);
	n_->param("/mark_follower/sitl", sitl_, false);

	ROS_INFO("Starting mark follower: %s, SITL: %s", (challenge_ == FIRST_CHALLENGE)?"First Challenge":"Third Challenge", (sitl_ == true)?"Enabled":"Disable");

	// >> Init variable <<

    	// Thrust Budget
	budgetResidual_ = 0;

	// Current altitude
	altitude_ = 0;

	// Load template for the first challenge
	if (challenge_ == FIRST_CHALLENGE)
		tmpl_ = imread("../img/template.jpg", CV_LOAD_IMAGE_COLOR );
	
	// >> Subscribers <<

	image_transport::ImageTransport it(*n_);

	// --- SITL ---

	if (sitl_ == true){	
		// Takes image from erlecopter simulation camera 
		if (challenge_ == FIRST_CHALLENGE)
			image_tr_sub_ = it.subscribe("/erlecopter/bottom/image_raw", 1, &mark_follower_class::imageFirstChallengeCallback, this);
		else // Third Challenge
			image_tr_sub_ = it.subscribe("/erlecopter/bottom/image_raw", 1, &mark_follower_class::imageThirdChallengeCallback, this);
	}else{
		// Takes image from ids camera 
		if (challenge_ == FIRST_CHALLENGE)
			image_tr_sub_ = it.subscribe("/ids_viewer/image", 1, &mark_follower_class::imageFirstChallengeCallback, this);
		else // Third Challenge
			image_tr_sub_ = it.subscribe("/ids_viewer/image", 1, &mark_follower_class::imageThirdChallengeCallback, this);
	}

	// -----------

	// Ros moving uav

    	altitude_sub_ = n_->subscribe("/mavros/global_position/rel_alt", 1, &mark_follower_class::altitudeCallback, this);
	
	// >> Publishers <<

	// Target positions

	target_pos_pub_ = n_->advertise<geometry_msgs::Pose2D>("/camera_class/target_pose", 10);
	
	// Ros moving uav topic

    	overrideRCIn_pub_ = n_->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    	markTarget_pub_ = n_->advertise<mark_follower::markPoseStamped>("/ids_rec/pose", 10);

	// ---------------------------------------------------------------------------------------- >> TRIAL << ----------------------------------------------------------------------------------------
	//    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	// 		iLowH = CAM_RED_LH;
	// 		iHighH = CAM_RED_HH;

	// 		iLowS = CAM_RED_LS;
	// 		iHighS = CAM_RED_HS;

	// 		iLowV = CAM_RED_LV;
	// 		iHighV = CAM_RED_HV;
	 // // Create trackbars in "Control" window
	 // cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 255)
	 // cvCreateTrackbar("HighH", "Control", &iHighH, 255);

	 // cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	 // cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	 // cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	 // cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	 //    cap_ = new VideoCapture("/home/solaris/recognition_719.avi"); // open the default camera
	    
	 //    if(!cap_->isOpened())  // check if we succeeded
	 //        return ;
	// while (waitKey(30) == -1) 
	// 	trial();
	// spin();

	// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


	// Ros loop

	ros::spin();        

}

mark_follower_class::~mark_follower_class(){}


// Get Frame from webcam 

void mark_follower_class::get_frame(const char* str){

	VideoCapture inputVideo(str);              // Open input
	if (!inputVideo.isOpened())
	{
		ROS_INFO("Could not open the input video: %s", str);
		return;
	}

	inputVideo >> ocvMat_;

}

// Get Frame offline

void mark_follower_class::get_static_frame(){

	ocvMat_ = imread("../img/img2.png", CV_LOAD_IMAGE_COLOR);

}

float mark_follower_class::approx_perpendicular(double m1, double m2){

	return -1.0 - (m1 * m2);
}

void mark_follower_class::find_best_perpendicular_match(vector<double> m){

	double best = 1000;
	int idx = 0;

	Mat match = Mat::zeros(m.size(), m.size(), CV_8UC1);

	for (int j = 0; j < m.size(); j++)
		for (int i = 0; i < m.size(); i++){
			match.at<double>(i,j) = approx_perpendicular(m[j], m[i]);
			// cout << match.at<double>(i,j) << " " << approx_perpendicular(m[j], m[i]) << endl;
		}

	// cout << match << endl;

	for (int j = 0; j < m.size(); j++){

		for (int i = 0; i < m.size(); i++){
			if (match.at<double>(i, j) < best){
				best = match.at<double>(i, j);
				idx = i;
			}
		}
		if (best != 1000)
			// cout << match.at<double>(idx, j) << " " << j << " " << idx << endl;

		best = 1000;
		idx = 0;
	}


}

void mark_follower_class::sameQ(vector<double> &m, vector<double>& q){

	vector<double> m_buf;
	vector<double> q_buf;
	vector<int> idx;

	bool nearest = true;

	while(nearest){

		nearest = false;

		double M = m[0];
		double Q = q[0];

		idx.push_back(0);

		for (int i = 1; i < m.size(); i++){

		   if (fabs(M - m[i]) < .5){
				if (fabs(Q - q[i]) < 50){
					nearest = true;
					idx.push_back(i);
				}
				else{
					m_buf.push_back(m[i]);
					q_buf.push_back(q[i]);
				}
			}
			else{
					m_buf.push_back(m[i]);
					q_buf.push_back(q[i]);
			}
		}

		double sum = 0;
		
		for (int i = 0; i < idx.size(); i++)
			sum += q[idx[i]];

		sum /= idx.size();

		m_buf.push_back(M);
		q_buf.push_back(sum);


		// cout << "bufS" << m_buf.size() << " " << sum << endl;

		// for (int i = 0; i < q_buf.size(); ++i)
		//     cout << q_buf[i] << " ";

		// cout << endl;

		if (nearest == true){
			m = m_buf;
			q = q_buf;
		}


		m_buf.clear();
		q_buf.clear();
		idx.clear();
	}
}


Mat mark_follower_class::otsuTH(Mat src, bool inv){

	cvtColor(src, src, COLOR_RGB2GRAY);

	if (inv == true)
		cv::threshold(src, src, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);    
	else
		cv::threshold(src, src, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);    

	return src;

}

void mark_follower_class::t_matching(descriptor dsc, Mat tmpl){
 
	cout << "XXX1_t" << endl;

	// imshow("Not Rotate", dsc.img);

	Mat src = adjust_rotation(dsc);

	// imshow("Rotate", src);

	// Choose  match method

	int match_method = CV_TM_CCOEFF; 
	
	Mat result;

	// Source image to display
	Mat img_display;

	src.copyTo( img_display );

	int coef_resize = 0;

	// Define the coefficient to resize the template

	if (src.cols > src.rows)
		coef_resize = src.rows;
	else
		coef_resize = src.cols;
	
	cout << coef_resize << endl;

	Mat newTemplate;

	// Resize template

	resize(tmpl, newTemplate, Size((int) (coef_resize), (int)(coef_resize)));

	tmpl = newTemplate;

	/// Create the result matrix
	int result_cols = src.cols + 1;
	int result_rows = src.rows + 1;
	
	//cout << result_cols << " " << result_rows << " " << result_cols * result_rows <<endl;
	// cout << MIN_BLOB_SIZE << " " << result_cols * result_rows << endl; 
	// Check on double dimension of the image and its area

	cout << "XXX2_t" << endl;
	if (result_rows <= 0 || result_cols <= 0 || result_cols * result_rows < MIN_BLOB_SIZE || result_cols * result_rows > MAX_BLOB_SIZE)
		return;


	// Allocate new dimensions
	result.create( result_rows, result_cols, CV_32FC1 );

	/// Do the Matching and Normalize
	matchTemplate( src, tmpl, result, match_method );
	// normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

	/// Localizing the best match with minMaxLoc
	double minVal; 
	double maxVal; 
	Point minLoc; 
	Point maxLoc;
	Point matchLoc;

	minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	cout << "XXX3_t" << endl;
	// The higher value is the best matching result
	matchLoc = maxLoc; 

	/// Show me what you got
	rectangle( img_display, matchLoc, Point( matchLoc.x + tmpl.cols , matchLoc.y + tmpl.rows ), Scalar::all(0), 2, 8, 0 );
	rectangle( result, matchLoc, Point( matchLoc.x + tmpl.cols , matchLoc.y + tmpl.rows ), Scalar::all(0), 2, 8, 0 );

	// Show result image
	// imshow( "Result window", img_display );
	// imshow( "Result operation windows", result );

	// Center
	matchLoc.x = dsc.origin.x + maxLoc.x + tmpl.cols / 2;
	matchLoc.y = dsc.origin.y + maxLoc.y + tmpl.rows / 2;

	// Push point the sorted list
	sl.push(matchLoc, maxVal);

	// Create point with custom dimensions
	Point fdsc(dsc.origin.x + dsc.img.cols, dsc.origin.y + dsc.img.rows);

	// Draw square around the blob
	mark_obj("hit", dsc.origin, fdsc, Scalar(0, 0, 255));

}
	
vector<descriptor> mark_follower_class::get_blob(const Mat src){

	vector<vector<Point> > contours, cb;
	vector<Vec4i> hierarchy;

	float epsilon, area;
	int minX, minY;
	int maxX, maxY;
	int cateto_a, cateto_b;

	Mat drawing = Mat::zeros( src.size(), CV_8UC3 );

	Mat blob;
	descriptor bd;
	vector<descriptor> blob_descriptor;
	vector<shapes> shape_dir;

	// Find contours
	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Check out contours appling a 5% approximation on the area
	for (int i = 0; i < contours.size(); ++i){
		area = arcLength(contours[i], true);

		epsilon = 0.05 * area;

		approxPolyDP(contours[i], contours[i], epsilon, true);

		
		// Save contours only if area is bigger the 50 and the section has 4 vertex
		// cout << "area " << area << " "<< contours[i].size() << endl;

		if (contours[i].size() == 4 && area > 15){ // 100 

			double d1, d2, d3, d4, diag1, diag2;
			double P;

			d1 = sqrt((pow(contours[i][0].x - contours[i][1].x, 2)) +  (pow(contours[i][0].y - contours[i][1].y, 2)) );
			d2 = sqrt((pow(contours[i][1].x - contours[i][2].x, 2)) +  (pow(contours[i][1].y - contours[i][2].y, 2)) );
			d3 = sqrt((pow(contours[i][2].x - contours[i][3].x, 2)) +  (pow(contours[i][2].y - contours[i][3].y, 2)) );
			d4 = sqrt((pow(contours[i][3].x - contours[i][0].x, 2)) +  (pow(contours[i][3].y - contours[i][0].y, 2)) );

			diag1 = sqrt((pow(contours[i][0].x - contours[i][2].x, 2)) +  (pow(contours[i][0].y - contours[i][2].y, 2)) );
			diag2 = sqrt((pow(contours[i][1].x - contours[i][3].x, 2)) +  (pow(contours[i][1].y - contours[i][3].y, 2)) );

			if (challenge_ == FIRST_CHALLENGE){

				// Is it a square?

				// cout << " " <<d1 << " " << d2 << " " << d3 << " " << d4 << " " << diag1 * cos(M_PI / 4) << " " << diag2 * cos(M_PI / 4) << endl;
				if (fabs(d1 - d2) < 20 && fabs(d1 - d2) < 20 && fabs(d1 - d3) < 20 && fabs(diag1  - diag2) < 10)
					cb.push_back(contours[i]);

			}else{ // THIRD_CHALLENGE

				// Is it a square?
				// cout << " " <<d1 << " " << d2 << " " << d3 << " " << d4 << " " << diag1 * cos(M_PI / 4) << " " << diag2 * cos(M_PI / 4) << endl;

				// cout << "area " << area << endl;
				// if (altitude_ < 2.0){
				// 	if (area > 600){
				// // if (fabs(d1 - d2) < 50 && fabs(d1 - d2) < 50 && fabs(d1 - d3) < 50 && fabs(diag1  - diag2) < 40){
				// 	shape_dir.push_back(SQUARE);
				// 	cb.push_back(contours[i]);
				// // }
				// 	}
				// }
				// else{
					shape_dir.push_back(SQUARE);
					cb.push_back(contours[i]);
				// }
				// Is it a rectangle?

				if (fabs(d1 - d3) < 20 && fabs(d2 - d4) < 20 && fabs(d1 - d4) > 20 && fabs(diag1  - diag2) < 10)
				{
					shape_dir.push_back(RECTANGLE);
					cb.push_back(contours[i]);
				}

				// Is it a circle?

				// TODO

			}
		}
	}

	
	// Draw contours

	if (cb.size() > 0)
	{
		// Draw line (blue)

		for( int i = 0; i< contours.size(); i++ )		
			drawContours( drawing, contours, i, Scalar( 255, 0, 0 ), 2, 8, hierarchy, 0, Point() );
		
		// Draw vertex (red)

		for (int i = 0; i < cb.size(); i++ )
			for (int j = 0; j < cb[i].size(); j++ )
				circle( drawing, cb[i][j], 2, Scalar(0, 0, 255), 3, 16);

	}
	
	// imshow("draw", drawing);

	if (challenge_ == FIRST_CHALLENGE){

		for (int i = 0; i < cb.size(); i++){
			
			minX = CAMERA_VID_HEIGTH;
			minY = CAMERA_VID_WIDTH;
			maxX = 0;
			maxY = 0;

			cateto_a = 0;
			cateto_b = 0;

			for (int j = 0; j < cb[i].size(); j++){
				
				if (cb[i][j].x > maxX)
					maxX = cb[i][j].x;            

				if (cb[i][j].x < minX){
					minX = cb[i][j].x;
					cateto_a = cb[i][j].y;
				}

				if (cb[i][j].y > maxY)
					maxY = cb[i][j].y;        

				if (cb[i][j].y < minY){
					minY = cb[i][j].y;
					cateto_b = cb[i][j].x;
				}
			}
			
			// Save the blob from original image

			blob = ocvMat_(Rect(minX, minY, maxX - minX, maxY - minY));

			// Store information about the blob in a blob directory

			bd.img = blob;
			bd.origin.x = minX;
			bd.origin.y = minY;

			bd.alpha = atan2(cateto_b, cateto_a);

			blob_descriptor.push_back(bd);

		}
	}
	else{ // THIRD_CHALLENGE

		for (int i = 0; i < cb.size(); i++){
			
			minX = CAMERA_VID_HEIGTH;
			minY = CAMERA_VID_WIDTH;
			maxX = 0;
			maxY = 0;

			cateto_a = 0;
			cateto_b = 0;

			for (int j = 0; j < cb[i].size(); j++){
				
				if (cb[i][j].x > maxX)
					maxX = cb[i][j].x;            

				if (cb[i][j].x < minX){
					minX = cb[i][j].x;
					cateto_a = cb[i][j].y;
				}

				if (cb[i][j].y > maxY)
					maxY = cb[i][j].y;        

				if (cb[i][j].y < minY){
					minY = cb[i][j].y;
					cateto_b = cb[i][j].x;
				}
			}
			
			// Save the blob from original image

			blob = ocvMat_(Rect(minX, minY, maxX - minX, maxY - minY));

			// Store information about the blob in a blob directory

			bd.img = blob;
			bd.origin.x = minX;
			bd.origin.y = minY;

			bd.alpha = atan2(cateto_b, cateto_a);

			bd.shape = shape_dir[i];

			blob_descriptor.push_back(bd);

		}

	}

	return blob_descriptor;
}


Mat mark_follower_class::morph_operation(Mat src, const bool inv){

	int dilate_iter;

	if (challenge_ == FIRST_CHALLENGE){

		if (inv){

			if (altitude_ > 17.5 )
				dilate_iter = 2;
			else
				if (altitude_ > 12.5 )
					dilate_iter = 4;	
				else
					if (altitude_ > 7.5 )
						dilate_iter = 6;
					else
						dilate_iter = 8;


			for (int i = 0; i <  dilate_iter; ++i)
				dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			
			erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		}
		else{
			erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		}
	}
	else{
		if (inv){
			// External
			erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		}else{
			// Internal
			dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		}
	}
	return src;
}

Mat mark_follower_class::adjust_rotation(descriptor dsc){

	Mat dst;

	// // Get center of image
	// Point2f src_center(dsc.img.cols / 2.0F, dsc.img.rows / 2.0F);

	// // Get matrix rotation 
	// Mat rot_mat = getRotationMatrix2D(src_center, cdsc.alpha * RAD2DEG, 1.0);
	
	// Mat dst;
	
	// warpAffine(dsc.img, dst, rot_mat, dsc.img.size());

	dst = dsc.img;

	return dst;    
}


void mark_follower_class::mark_obj(string str, Point p1, Point p2, const Scalar color){

	// Check on bounds

	if (p1.x > CAMERA_VID_HEIGTH)
	   p1.x = CAMERA_VID_HEIGTH;

	if (p1.y > CAMERA_VID_WIDTH)
	   p1.y = CAMERA_VID_WIDTH;

	if (p1.x < 0)
	   p1.x = 0;

	if (p1.y < 0)
	   p1.y = 0;

	if (p2.x > CAMERA_VID_HEIGTH)
	   p1.x = CAMERA_VID_HEIGTH;

	if (p2.y > CAMERA_VID_WIDTH)
	   p2.y = CAMERA_VID_WIDTH;

	if (p2.x < 0)
	   p1.x = 0;

	if (p2.y < 0)
	   p1.y = 0;

	// Draw rectangle on target

	rectangle(ocvMat_, p1, p2, color, 3);
 
	p1.y -= 10;

	if (p1.y < 0)
	   p1.y = 0;

	//Write text above the rectangle

	putText(ocvMat_, str, p1, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.8, color);
}

Mat mark_follower_class::color_detection(const Mat src, const colors col){

	if (src.dims == 0){
		cerr << "[WARNING] color_detection(): Empty matrix." << endl;
		return src;
	}
	
	Mat imgHSV, imgTh;

	// Choose your color

	switch(col){

		case BLUE: {

			iLowH = CAM_BLUE_LH;
			iHighH = CAM_BLUE_HH;

			iLowS = CAM_BLUE_LS;
			iHighS = CAM_BLUE_HS;

			iLowV = CAM_BLUE_LV;
			// if (altitude_ < 0.7)
			// 	iLowV -= 65;

			iHighV = CAM_BLUE_HV;

		}break;
		case RED: {

			// iLowH = CAM_RED_LH;
			// iHighH = CAM_RED_HH;

			// iLowS = CAM_RED_LS;sdf
			// iHighS = CAM_RED_HS;

			// iLowV = CAM_RED_LV;
			// iHighV = CAM_RED_HV;

		}break;
		case BLACK: {

			// iLowH = CAM_BLACK_LH;
			// iHighH = CAM_BLACK_HH;

			// iLowS = CAM_BLACK_LS;
			// iHighS = CAM_BLACK_HS;

			// iLowV = CAM_BLACK_LV;
			// iHighV = CAM_BLACK_HV;

		}break;
		default:{ // Green

			iLowH = CAM_GREEN_LH;
			iHighH = CAM_GREEN_HH;

			iLowS = CAM_GREEN_LS;
			iHighS = CAM_GREEN_HS;

			iLowV = CAM_GREEN_LV;
			iHighV = CAM_GREEN_HV;
		}
	}


	cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV 

	// Search pixel in the range
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgTh); //Threshold the image

	return imgTh;
}

void mark_follower_class::init_kalman(const Point target){
	
	KF_ = new KalmanFilter(4, 2, 0);

	// intialization of KF...

	KF_->transitionMatrix = (Mat_<float>(4, 4) << 1,0,50,0,   0,1,0,50,  0,0,1,0,  0,0,0,1);
 
	KF_->statePost.at<float>(0) = target.x;
	KF_->statePost.at<float>(1) = target.y;
	KF_->statePost.at<float>(2) = 0;
	KF_->statePost.at<float>(3) = 0;

	// WRONG! ONLY TEST
	// KF_->statePre.at<float>(0) = target.x;
	// KF_->statePre.at<float>(1) = target.y;
	// KF_->statePre.at<float>(2) = 0;
	// KF_->statePre.at<float>(3) = 0;

	setIdentity(KF_->measurementMatrix);
	setIdentity(KF_->processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF_->measurementNoiseCov, Scalar::all(10));
	setIdentity(KF_->errorCovPost, Scalar::all(.1));


}

Point mark_follower_class::kalman(const Point target){


	Mat_<float> measurement(2, 1); 
	measurement.setTo(Scalar(0));
 
	// First predict, to update the internal statePre variable
	Mat prediction = KF_->predict();              

	// Get mouse point
	measurement(0) = target.x;
	measurement(1) = target.y; 

	// The update phase 
	Mat estimated = KF_->correct(measurement);

	Point statePt(estimated.at<float>(0), estimated.at<float>(1));
	
	return statePt;
}

double mark_follower_class::diff_ms(const timeval t1, const timeval t2)
{ 

	return ((t1.tv_usec + (t1.tv_sec - t2.tv_sec) * 1000000) - t2.tv_usec) / 1000;
}

Mat mark_follower_class::remove_field(Mat src){

	Mat imgHSV, imgTh;

	cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV 

	// Search pixel in the range
	inRange(imgHSV, Scalar( CAM_GREEN_LH, CAM_GREEN_LS, CAM_GREEN_LV), Scalar(CAM_GREEN_HH, CAM_GREEN_HS, CAM_GREEN_HV), imgTh); //Threshold the image

	//morphological opening (remove small objects from the foreground)

 	imgTh = morph_operation(imgTh); 

	// //morphological closing (fill small holes in the foreground)
	imgTh = morph_operation(imgTh, true);

	bitwise_not ( imgTh, imgTh );

	return imgTh;

}

void mark_follower_class::imageFirstChallengeCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{

	 //    	// BEGIN HOUGH TR

	 //    	Mat src, dst, color_dst;
		
		// src = ocvMat_;

	 //    	Canny( src, dst, 50, 200, 3 );
	 //    	cvtColor( dst, color_dst, CV_GRAY2BGR );


	 //    vector<Vec4i> lines;

	 //    HoughLinesP( dst, lines, 1, CV_PI / 180, 80, 40, 10 );
	 
	 //    cout << lines.size() << endl;

	 //    std::vector<double> m, q;

	 //    for( size_t i = 0; i < lines.size(); i++ )
	 //    {
	 //        	line( color_dst, Point(lines[i][0], lines[i][1]),
	 //           Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
	 //    }

	 //    namedWindow( "Source", 1 );
	 //    imshow( "Source", src );

	 //    namedWindow( "Detected Lines", 1 );
	 //    imshow( "Detected Lines", color_dst );

	    // END HOUGH TR

	    // FIRST CHALLENGE WORK

		// Get the msg image
		ocvMat_ = cv_bridge::toCvShare(msg, "bgr8")->image;

		Mat thr;

		vector<descriptor> blob_desc;
	
		/// Remove field by color

		thr = remove_field(ocvMat_);

		// // Show removing field result
		imshow("Removed Field", thr );

		// Morphologic operations
		thr = morph_operation(thr);

		// When image is empty go to the next frame
		if (thr.empty()){
			ROS_INFO("Empty after the morphological operations");
			return;
		}
		
		// Define message to send

		mark_follower::markPoseStamped msg2pub;

		// Get contours of the blobs

		// blob_desc = get_blob(thr);

		// // Call matching function on the blob descriptor discovered by get_blob()

		// for (int i = blob_desc.size(); i--;)
		// 	t_matching(blob_desc[i], tmpl_);

		// // Find out the best match
		// if (!sl.empty()){
		// 	target_ = sl.get_max();
		// 	// Clear list
		// 	sl.clear();
			
		// 	// Kalman Filter 
		// 	if (!state_kf_){
		// 		init_kalman(target_);
		// 		state_kf_ = true;
		// 	}
		// 	else
		// 		target_ = kalman(target_);  
			
		// 	// Draw point
		// 	circle( ocvMat_, target_, 2, Scalar(255, 0, 0), 3, 16);

		// 	budgetResidual_ = 10;

		// 	refVariance_++;

		// }else {
		// 	budgetResidual_--;

		// 	if (budgetResidual_ < 0)
		// 		refVariance_ = 0;
		// }

		// Populate message

		// msg2pub.stamp = ros::Time::now();
		
		// msg2pub.x = target_.x;
		// msg2pub.y = target_.y;

		// msg2pub.budgetResidual = budgetResidual_;
		// msg2pub.variance = refVariance_;

		// markTarget_pub_.publish(msg2pub);


		cv::imshow("view", ocvMat_);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	
}

void mark_follower_class::imageThirdChallengeCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		VideoCapture cap = *cap_;
		cap >> ocvMat_;

		// Get the msg image
		// ocvMat_ = cv_bridge::toCvShare(msg, "bgr8")->image;



		Mat thr, thr_red, thr_black, thr_blue;
		vector<descriptor> blob_desc;

		// Last pos
		cv::Point lastTarget = target_;
		
		/// Remove field by color

		thr_blue = color_detection(ocvMat_, BLUE);
	
		// if (!thr_blue.empty())
		// 	ROS_INFO("Something blue finded");

		 thr_red = color_detection(ocvMat_, RED);

		// if (!thr_red.empty())
		// 	ROS_INFO("Something red finded");

		 thr_black = color_detection(ocvMat_, BLACK);

		// if (!thr_black.empty())
		// 	ROS_INFO("Something black finded");

		cv::addWeighted(thr_blue, 1.0, thr_red, 1.0, 0.0, thr);
		cv::addWeighted(thr, 1.0, thr_black, 1.0, 0.0, thr);

		// thr = thr_blue;
		// Morphologic operations
		thr = morph_operation(thr);

		// Show removing field result
		imshow("Removed Field", thr );

		// When image is empty go to the next frame
		if (thr.empty()){
			ROS_INFO("Empty after the morphological operations");
			return;
		}
		// Define message to send

		mark_follower::markPoseStamped msg2pub;

		// Get contours of the blobs

		blob_desc = get_blob(thr);

		if (blob_desc.empty()){
			budgetResidual_--;

			if (budgetResidual_ < 0)
				refVariance_ = 0;
		}
		else
			for (int i = blob_desc.size(); i--;){

				Point fdsc(blob_desc[i].origin.x + blob_desc[i].img.cols, blob_desc[i].origin.y + blob_desc[i].img.rows);

				if (blob_desc[i].shape == SQUARE)
					mark_obj("square", blob_desc[i].origin, fdsc, Scalar(0, 0, 255));
			
				if (blob_desc[i].shape == RECTANGLE)
					mark_obj("rectangle", blob_desc[i].origin, fdsc, Scalar(0, 0, 255));
			
				if (blob_desc[i].shape == CIRCLE)
					mark_obj("square", blob_desc[i].origin, fdsc, Scalar(0, 0, 255));


				// target_ = blob_desc[i].origin;
				// target_.x += blob_desc[i].img.cols / 2 ;
				// target_.y += blob_desc[i].img.rows / 2 ;

				// refVariance_++;
				// budgetResidual_ = 10;
			}

			// XXX test only
		blob_desc = get_blob(thr_blue );

		if (blob_desc.empty()){
			budgetResidual_--;

			if (budgetResidual_ < 0)
				refVariance_ = 0;
		}
		else
			for (int i = blob_desc.size(); i--;){
				

				target_ = blob_desc[i].origin;
				target_.x += blob_desc[i].img.cols / 2 ;
				target_.y += blob_desc[i].img.rows / 2 ;

				refVariance_++;
				budgetResidual_ = 10;
			}

		// Call matching function on the blob descriptor discovered by get_blob()

		// Find out the best match

		// Kalman Filter 
		if (!state_kf_){
			init_kalman(target_);
			state_kf_ = true;
		}
		else
			target_ = kalman(target_);  
		
		// Draw point
		circle( ocvMat_, target_, 2, Scalar(255, 0, 0), 3, 16);

		// Populate message

		// msg2pub.stamp = ros::Time::now();
		
		// msg2pub.x = target_.x;
		// msg2pub.y = target_.y;

		// msg2pub.budgetResidual = budgetResidual_;
		// msg2pub.variance = refVariance_;

		// markTarget_pub_.publish(msg2pub);


		cv::imshow("view", ocvMat_);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	
}

void mark_follower_class::altitudeCallback(const std_msgs::Float64Ptr &msg)
{
	altitude_ = msg->data;
}



// void mark_follower_class::trial()
// {
// 	try
// 	{
// 		VideoCapture cap = *cap_;
// 		cap >> ocvMat_;
		
// 		// Get the msg image
// 		// ocvMat_ = cv_bridge::toCvShare(msg, "bgr8")->image;

// 		Mat thr, thr_red, thr_black, thr_blue;
// 		vector<descriptor> blob_desc;

// 		// Last pos
// 		cv::Point lastTarget = target_;
		
// 		/// Remove field by color

// 		thr_blue = color_detection(ocvMat_, BLUE);
	
// 		// if (!thr_blue.empty())
// 		// 	ROS_INFO("Something blue finded");
// 		// thr_red = color_detection(ocvMat_, RED);

// 		// // if (!thr_red.empty())
// 		// // 	ROS_INFO("Something red finded");

// 		// thr_black = color_detection(ocvMat_, BLACK);

// 		// if (!thr_black.empty())
// 		// 	ROS_INFO("Something black finded");

// 		// cv::addWeighted(thr_blue, 1.0, thr_red, 1.0, 0.0, thr);
// 		// cv::addWeighted(thr, 1.0, thr_black, 1.0, 0.0, thr);
// 		thr = thr_blue;
// 		// Morphologic operations
// 		thr = morph_operation(thr);
// 		// Show removing field result
// 		imshow("Removed Field", thr );

// 		// When image is empty go to the next frame
// 		if (thr.empty()){
// 			ROS_INFO("Empty after the morphological operations");
// 			return;
// 		}

// 		// Get contours of the blobs

// 		blob_desc = get_blob(thr);

// 		if (blob_desc.empty()){
// 			budgetResidual_--;

// 			if (budgetResidual_ < 0)
// 				refVariance_ = 0;
// 		}
// 		else
// 			for (int i = blob_desc.size(); i--;){

// 				Point fdsc(blob_desc[i].origin.x + blob_desc[i].img.cols, blob_desc[i].origin.y + blob_desc[i].img.rows);

// 				if (blob_desc[i].shape == SQUARE)
// 					mark_obj("square", blob_desc[i].origin, fdsc, Scalar(0, 0, 255));
			
// 				if (blob_desc[i].shape == RECTANGLE)
// 					mark_obj("rectangle", blob_desc[i].origin, fdsc, Scalar(0, 0, 255));
			
// 				if (blob_desc[i].shape == CIRCLE)
// 					mark_obj("square", blob_desc[i].origin, fdsc, Scalar(0, 0, 255));


// 			}

// 		cv::imshow("view", ocvMat_);

// 		outVid_ << ocvMat_;
// 		cv::waitKey(30);
// 	}
// 	catch (cv_bridge::Exception& e){
// 		ROS_ERROR("Could not convert from to 'bgr8'.");
// 	}
	
// }