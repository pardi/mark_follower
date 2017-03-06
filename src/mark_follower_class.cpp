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
	// 	cout << v[i].target_ << ""; 

	// cout << endl << "print " << endl;

	// p.print();
	// cout << "postPrint" << endl;

	// --- 

	// Initialize vars

	verbose_ = verbose;

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

	// outVid_.open(str.c_str(), ex, 15.0, Size( CAMERA_VID_WIDTH, CAMERA_VID_HEIGHT  ), true);

	// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


	// Ros operations
 
	n_ = n;

	// Get params

	std::string ns = ros::this_node::getNamespace();
	
	n_->param(ns + "/mark_follower/challenge", challenge_, false);
	n_->param(ns + "/mark_follower/sitl", sitl_, false);

	ROS_INFO("Starting mark follower: %s, SITL: %s", (challenge_ == FIRST_CHALLENGE)?"First Challenge":"Third Challenge", (sitl_ == true)?"Enabled":"Disable");

	// >> Init variable <<

    	// Thrust Budget
	budgetResidual_ = 0;

	// Current altitude
	altitude_ = 0;

	// Init tau variables
	medianWinTau_ = Mat::zeros(3, 1, CV_32F);
	tauCount_ = 0;

	// Load template for the first challenge
	if (challenge_ == FIRST_CHALLENGE)
		tmpl_ = imread("/home/solaris/catkin_ws/src/mark_follower/img/template.jpg", CV_LOAD_IMAGE_COLOR );
		// tmpl_ = imread("/home/odroid/catkin_ws/src/mark_follower/img/template.jpg", CV_LOAD_IMAGE_COLOR );


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
			image_tr_sub_ = it.subscribe(ns + "/ids_viewer/image", 1, &mark_follower_class::imageFirstChallengeCallback, this);
		else // Third Challenge
			image_tr_sub_ = it.subscribe(ns + "/ids_viewer/image", 1, &mark_follower_class::imageThirdChallengeCallback, this);
	}

	// -----------

	// Ros moving uav

    	altitude_sub_ = n_->subscribe("/mavros/global_position/rel_alt", 1, &mark_follower_class::altitudeCallback, this);
	
	// >> Publishers <<

	// Target positions

	target_pos_pub_ = n_->advertise<geometry_msgs::Pose2D>(ns + "/camera_class/target_pose", 10);
	
	// Ros moving uav topic

    	overrideRCIn_pub_ = n_->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    	markTarget_pub_ = n_->advertise<mark_follower::markPoseStamped>(ns + "/mark_follower/target_pose", 10);


	// Service to get params

	ros::ServiceClient client = n_->serviceClient<ids_viewer::IDSparams>(ns + "/ids_viewer/params");

	ids_viewer::IDSparams srv;

	if (client.call(srv)){

		width_ = srv.response.width;
		height_ = srv.response.height;

	}else{
		width_ = CAMERA_VID_WIDTH;
		height_ = CAMERA_VID_HEIGHT;		
	}

	// ----------------------------------------------------------------------------------------- >> FPV << -----------------------------------------------------------------------------------------

	// Generate image transport element
  	// image_transport::ImageTransport it_IA(*n_);
  	// image_aug_pub_ = it_IA.advertise("mark_follower/image_aug", 1);

	// ---------------------------------------------------------------------------------------- >> TRIAL << ----------------------------------------------------------------------------------------
	 //   namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

		// 	iLowH = CAM_BLUE_LH;
		// 	iHighH = CAM_BLUE_HH;

		// 	iLowS = CAM_BLUE_LS;
		// 	iHighS = CAM_BLUE_HS;

		// 	iLowV = CAM_BLUE_LV;
		// 	iHighV = CAM_BLUE_HV;
	 // // Create trackbars in "Control" window
	 // cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 255)
	 // cvCreateTrackbar("HighH", "Control", &iHighH, 255);

	 // cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	 // cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	 // cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	 // cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	//     // cap_ = new VideoCapture("/home/solaris/recognition_719.avi"); // open the default camera
	    
	//     // if(!cap_->isOpened())  // check if we succeeded
	//     //     return ;
	// while (waitKey(30) == -1) 
		// trial();

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

	ocvMat_ = imread("/home/solaris/catkin_ws/src/mark_follower/img/red_blue.png", CV_LOAD_IMAGE_COLOR);

}

float mark_follower_class::approx_perpendicular(double m1, double m2){

	return -1.0 - (m1 * m2);
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
	
	Mat newTemplate;

	// Resize template
	resize(tmpl, newTemplate, Size((int) (coef_resize), (int)(coef_resize)));

	tmpl = newTemplate;

	/// Create the result matrix
	int result_cols = src.cols + 1;
	int result_rows = src.rows + 1;
	
	// cout << result_cols << " " << result_rows << " " << result_cols * result_rows <<endl;
	// cout << MIN_BLOB_SIZE << " " << result_cols * result_rows << endl; 
	// Check on double dimension of the image and its area

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
	// The higher value is the best matching result
	// matchLoc = maxLoc; 

	/// Show me what you got
	// rectangle( img_display, matchLoc, Point( matchLoc.x + tmpl.cols , matchLoc.y + tmpl.rows ), Scalar::all(0), 2, 8, 0 );
	// rectangle( result, matchLoc, Point( matchLoc.x + tmpl.cols , matchLoc.y + tmpl.rows ), Scalar::all(0), 2, 8, 0 );

	// Show result image
	// imshow( "Result window", img_display );
	// imshow( "Result operation windows", result );

	// Center
	matchLoc.x = dsc.origin.x + maxLoc.x + tmpl.cols / 2;
	matchLoc.y = dsc.origin.y + maxLoc.y + tmpl.rows / 2;

	// Push point the sorted list
	sl.push(matchLoc, maxVal, dsc.is_square);

	// Create point with custom dimensions
	// Point fdsc(dsc.origin.x + dsc.img.cols, dsc.origin.y + dsc.img.rows);

	// Draw square around the blob
	// mark_obj("hit", dsc.origin, fdsc, Scalar(0, 0, 255));

}

vector<descriptor> mark_follower_class::get_blobFirstCh(const Mat src){
	
	vector<vector<Point> > contours, cb, cb1;
	vector<Vec4i> hierarchy;

	float epsilon, area;
	int minX, minY;
	int maxX, maxY;

	Mat drawing = Mat::zeros( src.size(), CV_8UC3 );

	Mat blob;
	descriptor bd;
	vector<descriptor> blob_descriptor;
	vector<shapes> shape_dir;
	vector<bool> is_square;

	// Find contours
	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


	// Check out contours appling a 5% approximation on the area
	for (int i = 0; i < contours.size(); ++i){
		area = arcLength(contours[i], true);

		epsilon = 0.05 * area;

		approxPolyDP(contours[i], contours[i], epsilon, true);

		cb1.push_back(contours[i]);

		// Save contours only if area is bigger the 50 and the section has 4 vertex
		// cout << "area " << area << " "<< contours[i].size() << endl;

		if (( contours[i].size() == 4 ) && area > 20){ // 15 100 

			double d1, d2, d3, d4, diag1, diag2;
			double P;

			d1 = sqrt((pow(contours[i][0].x - contours[i][1].x, 2)) +  (pow(contours[i][0].y - contours[i][1].y, 2)) );
			d2 = sqrt((pow(contours[i][1].x - contours[i][2].x, 2)) +  (pow(contours[i][1].y - contours[i][2].y, 2)) );
			d3 = sqrt((pow(contours[i][2].x - contours[i][3].x, 2)) +  (pow(contours[i][2].y - contours[i][3].y, 2)) );
			d4 = sqrt((pow(contours[i][3].x - contours[i][0].x, 2)) +  (pow(contours[i][3].y - contours[i][0].y, 2)) );

			diag1 = sqrt((pow(contours[i][0].x - contours[i][2].x, 2)) +  (pow(contours[i][0].y - contours[i][2].y, 2)) );
			diag2 = sqrt((pow(contours[i][1].x - contours[i][3].x, 2)) +  (pow(contours[i][1].y - contours[i][3].y, 2)) );

			// Is it a square?

			// cout << " " <<d1 << " " << d2 << " " << d3 << " " << d4 << " " << diag1 * cos(M_PI / 4) << " " << diag2 * cos(M_PI / 4) << endl;
			// cout << area << endl;	

			if (!(fabs(d1 - d2) < 20 && fabs(d1 - d2) < 20 && fabs(d1 - d3) < 20 && fabs(diag1  - diag2) < 10)){

				bool corner[4];
				
				corner[0] = (contours[i][0].x < 10) || (contours[i][0].x > (src.cols - 10)) || (contours[i][0].y < 10 )  || (contours[i][0].y > (src.rows - 10));
				corner[1] = (contours[i][1].x < 10) || (contours[i][1].x > (src.cols - 10)) || (contours[i][1].y < 10 )  || (contours[i][1].y > (src.rows - 10));
				corner[2] = (contours[i][2].x < 10) || (contours[i][2].x > (src.cols - 10)) || (contours[i][2].y < 10 )  || (contours[i][2].y > (src.rows - 10));
				corner[3] = (contours[i][3].x < 10) || (contours[i][3].x > (src.cols - 10)) || (contours[i][3].y < 10 )  || (contours[i][3].y > (src.rows - 10));


				if ( (corner[0] && corner[1]) || (corner[0] && corner[2]) || (corner[0] && corner[3]) || (corner[1] && corner[2]) || (corner[1] && corner[3])
				     || (corner[2] && corner[3]) ){

					is_square.push_back(false);

					cb.push_back(contours[i]);

					// cout << contours[i][0] << contours[i][1] <<  contours[i][2]  << contours[i][3]<< endl ;
				}
			}
			else{
				is_square.push_back(true);
				cb.push_back(contours[i]);
			}
		


			// is_square.push_back(fabs(d1 - d2) < 20 && fabs(d1 - d2) < 20 && fabs(d1 - d3) < 20 && fabs(diag1  - diag2) < 10);

			// cb.push_back(contours[i]);
		}
	}
	
	// ------------------------> Draw contours <------------------------

	if (cb1.size() > 0)
	{
		// Draw line (blue)
	
	for( int i = 0; i< contours.size(); i++ )		
		drawContours( drawing, contours, i, Scalar( 255, 0, 0 ), 2, 8, hierarchy, 0, Point() );
		
		// Draw vertex (red)

		for (int i = 0; i < cb1.size(); i++ )
			for (int j = 0; j < cb1[i].size(); j++ )
				circle( drawing, cb1[i][j], 2, Scalar(0, 0, 255), 3, 16);

	}
	
	// imshow("draw", drawing);

	// ---------------------------------------------------------------------------

	for (int i = 0; i < cb.size(); i++){
		
		minX = height_;
		minY = width_;
		maxX = 0;
		maxY = 0;

		for (int j = 0; j < cb[i].size(); j++){
			
			if (cb[i][j].x > maxX)
				maxX = cb[i][j].x;            

			if (cb[i][j].x < minX)
				minX = cb[i][j].x;

			if (cb[i][j].y > maxY)
				maxY = cb[i][j].y;        

			if (cb[i][j].y < minY)
				minY = cb[i][j].y;
		}
		
		// Save the blob from original image

		blob = ocvMat_(Rect(minX, minY, maxX - minX, maxY - minY));

		// Store information about the blob in a blob directory

		bd.img = blob;
		bd.origin.x = minX;
		bd.origin.y = minY;
		bd.is_square = is_square[i];
				
		blob_descriptor.push_back(bd);

	}

	return blob_descriptor;
}

std::vector<descriptor> mark_follower_class::get_blobThirdCh(const Mat src_color){


	descriptor bd;
	std::vector<descriptor> blob_descriptor;

	Mat src_gray;
	int thresh = 100;

	cvtColor( src_color, src_gray, COLOR_BGR2GRAY );
	blur( src_gray, src_gray, Size(3,3) );

	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;


	Canny( src_gray, canny_output, thresh, thresh*2, 3 );

	findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

	Moments mu;

	for( size_t i = contours.size(); i--; ){

		mu = moments( contours[i], false );

		bd.origin = Point2f( static_cast<float>(mu.m10/mu.m00) , static_cast<float>(mu.m01/mu.m00) );

		if (bd.origin.x >= 0 && bd.origin.x < src_color.cols && bd.origin.y >= 0 && bd.origin.y < src_color.rows){
			bd.color = get_color(src_color, bd.origin);
			// string str = (bd.color == 0)?"BLUE":(bd.color == 1)?"RED":(bd.color == 5)?"WHITE":"ERROR";
			// cout << str << endl;
			blob_descriptor.push_back(bd);
		}
	}


	// ------------------------> Draw contours <------------------------

	// Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	// RNG rng(12345);

	// for( size_t i = 0; i< blob_descriptor.size(); i++ )
	//    {
	//      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	//      drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
	//      descriptor bd_tmp = blob_descriptor[i];
	//      circle( drawing, bd_tmp.origin, 4, color, -1, 8, 0 );
	// }
	// namedWindow( "Contours", WINDOW_AUTOSIZE );

	// imshow("Contours", drawing);

	// ---------------------------------------------------------------------------


	return blob_descriptor;

}

	
// vector<descriptor> mark_follower_class::get_blobThirdCh(const Mat src, const Mat src_color){

// 	vector<vector<Point> > contours, cb;
// 	vector<Vec4i> hierarchy;

// 	float epsilon, area;
// 	int minX, minY;
// 	int maxX, maxY;
// 	int cateto_a, cateto_b;

// 	Mat drawing = Mat::zeros( src.size(), CV_8UC3 );

// 	Mat blob;
// 	descriptor bd;
// 	vector<descriptor> blob_descriptor;
// 	vector <bool> is_square;
// 	// Find contours
// 	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

// 	// Check out contours appling a 5% approximation on the area
// 	for (int i = 0; i < contours.size(); ++i){
// 		area = arcLength(contours[i], true);

// 		epsilon = 0.05 * area;

// 		approxPolyDP(contours[i], contours[i], epsilon, true);

// 		// Save contours only if area is bigger the 50 and the section has 4 vertex
// 		// cout << "area " << area << " "<< contours[i].size() << endl;

// 		if (( contours[i].size() == 4 ) && area > 15){ // 15 

// 			double d1, d2, d3, d4, diag1, diag2;
// 			double P;

// 			d1 = sqrt((pow(contours[i][0].x - contours[i][1].x, 2)) +  (pow(contours[i][0].y - contours[i][1].y, 2)) );
// 			d2 = sqrt((pow(contours[i][1].x - contours[i][2].x, 2)) +  (pow(contours[i][1].y - contours[i][2].y, 2)) );
// 			d3 = sqrt((pow(contours[i][2].x - contours[i][3].x, 2)) +  (pow(contours[i][2].y - contours[i][3].y, 2)) );
// 			d4 = sqrt((pow(contours[i][3].x - contours[i][0].x, 2)) +  (pow(contours[i][3].y - contours[i][0].y, 2)) );

// 			diag1 = sqrt((pow(contours[i][0].x - contours[i][2].x, 2)) +  (pow(contours[i][0].y - contours[i][2].y, 2)) );
// 			diag2 = sqrt((pow(contours[i][1].x - contours[i][3].x, 2)) +  (pow(contours[i][1].y - contours[i][3].y, 2)) );

// 			// Is it a square?

// 			// cout << " " <<d1 << " " << d2 << " " << d3 << " " << d4 << " " << diag1 * cos(M_PI / 4) << " " << diag2 * cos(M_PI / 4) << endl;
// 			// cout << area << endl;	

// 			if (!(fabs(d1 - d2) < 20 && fabs(d1 - d2) < 20 && fabs(d1 - d3) < 20 && fabs(diag1  - diag2) < 10)){

// 				bool corner[4];
				
// 				corner[0] = (contours[i][0].x < 10) || (contours[i][0].x > (src.cols - 10)) || (contours[i][0].y < 10 )  || (contours[i][0].y > (src.rows - 10));
// 				corner[1] = (contours[i][1].x < 10) || (contours[i][1].x > (src.cols - 10)) || (contours[i][1].y < 10 )  || (contours[i][1].y > (src.rows - 10));
// 				corner[2] = (contours[i][2].x < 10) || (contours[i][2].x > (src.cols - 10)) || (contours[i][2].y < 10 )  || (contours[i][2].y > (src.rows - 10));
// 				corner[3] = (contours[i][3].x < 10) || (contours[i][3].x > (src.cols - 10)) || (contours[i][3].y < 10 )  || (contours[i][3].y > (src.rows - 10));


// 				if ( (corner[0] && corner[1]) || (corner[0] && corner[2]) || (corner[0] && corner[3]) || (corner[1] && corner[2]) || (corner[1] && corner[3])
// 				     || (corner[2] && corner[3]) ){

// 					is_square.push_back(false);
// 					cb.push_back(contours[i]);

// 					// cout << contours[i][0] << contours[i][1] <<  contours[i][2]  << contours[i][3]<< endl ;
// 				}
// 			}
// 			else{
// 				is_square.push_back(true);
// 				cb.push_back(contours[i]);
// 			}
// 		}
// 	}

// 	// ------------------------> Draw contours <------------------------

// 	// if (cb.size() > 0)
// 	// {
// 	// 	// Draw line (blue)
	
// 	// for( int i = 0; i< contours.size(); i++ )		
// 	// 	drawContours( drawing, contours, i, Scalar( 255, 0, 0 ), 2, 8, hierarchy, 0, Point() );
		
// 	// 	// Draw vertex (red)

// 	// 	for (int i = 0; i < cb.size(); i++ )
// 	// 		for (int j = 0; j < cb[i].size(); j++ )
// 	// 			circle( drawing, cb[i][j], 2, Scalar(0, 0, 255), 3, 16);

// 	// }
	
// 	// imshow("draw", drawing);

// 	// ---------------------------------------------------------------------------

// 	for (int i = 0; i < cb.size(); i++){
		
// 		minX = height_;
// 		minY = width_;
// 		maxX = 0;
// 		maxY = 0;

// 		cateto_a = 0;
// 		cateto_b = 0;

// 		for (int j = 0; j < cb[i].size(); j++){
			
// 			if (cb[i][j].x > maxX)
// 				maxX = cb[i][j].x;            

// 			if (cb[i][j].x < minX){
// 				minX = cb[i][j].x;
// 				cateto_a = cb[i][j].y;
// 			}

// 			if (cb[i][j].y > maxY)
// 				maxY = cb[i][j].y;        

// 			if (cb[i][j].y < minY){
// 				minY = cb[i][j].y;
// 				cateto_b = cb[i][j].x;
// 			}
// 		}
		
// 		// Store information about the blob in a blob directory

// 		bd.origin.x = minX + (maxX + minX) / 2;
// 		bd.origin.y = minY + (maxY + minY) / 2;


// 		bd.color = get_color(src_color, bd.origin);

// 		blob_descriptor.push_back(bd);

// 	}

// 	return blob_descriptor;
// }

colors mark_follower_class::get_color(Mat src, Point origin){
	
	Mat imgHSV;

	cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV 

	Vec3b c = imgHSV.at<Vec3b>(origin);

	// cout << "Color " << (int) c.val[0] << " " << (int) c.val[1] << " " << (int) c.val[2] << endl;
	// cout << "THColorB " << CAM_BLUE_LH << " " << CAM_BLUE_HH << " " <<  CAM_BLUE_LS << " " <<  CAM_BLUE_HS << " " << CAM_BLUE_LV << " " << CAM_BLUE_HV<< endl;
	// cout << "THColorR " << CAM_RED_LH << " " << CAM_RED_HH << " " <<  CAM_RED_LS << " " <<  CAM_RED_HS << " " << CAM_RED_LV << " " << CAM_RED_HV<< endl;

	if ((int) c.val[0] >= CAM_BLUE_LH && (int) c.val[0] <= CAM_BLUE_HH && c.val[1] >= CAM_BLUE_LS && (int) c.val[1] <= CAM_BLUE_HS && (int) c.val[2] >= CAM_BLUE_LV && (int) c.val[2] <= CAM_BLUE_HV)
		return BLUE;

	if ((int) c.val[0] >= CAM_RED_LH && (int) c.val[0] <= CAM_RED_HH && (int) c.val[1] >= CAM_RED_LS && (int) c.val[1] <= CAM_RED_HS && (int) c.val[2] >= CAM_RED_LV && (int) c.val[2] <= CAM_RED_HV)
		return RED;

	if ((int) c.val[0] >= CAM_YELLOW_LH && (int) c.val[0] <= CAM_YELLOW_HH && (int) c.val[1] >= CAM_YELLOW_LS && (int) c.val[1] <= CAM_YELLOW_HS && (int) c.val[2] >= CAM_YELLOW_LV && (int) c.val[2] <= CAM_YELLOW_HV)
		return YELLOW;
	else
		return WHITE;


	
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
				// if (fabs(d1 - d2) < 20 && fabs(d1 - d2) < 20 && fabs(d1 - d3) < 20 && fabs(diag1  - diag2) < 10)
				
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
			
			minX = height_;
			minY = width_;
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
			
			minX = height_;
			minY = width_;
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
 
	// erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	int dilate_iter;

	if (altitude_ > 17.5 )
		dilate_iter = 1;
	else
		if (altitude_ > 12.5 )
			dilate_iter = 1;	
		else
			if (altitude_ > 7.5 )
				dilate_iter = 1;
			else
				if (altitude_ > 4.5 )
					dilate_iter = 3;
				else
					dilate_iter = 4;



	for (int i = 0; i <  dilate_iter; ++i)
		erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	
	for (int i = 0; i <  dilate_iter; ++i)
		dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


	return src;


	// int dilate_iter;

	// if (challenge_ == FIRST_CHALLENGE){

	// 	if (inv){

	// 		if (altitude_ > 17.5 )
	// 			dilate_iter = 2;
	// 		else
	// 			if (altitude_ > 12.5 )
	// 				dilate_iter = 4;	
	// 			else
	// 				if (altitude_ > 7.5 )
	// 					dilate_iter = 6;
	// 				else
	// 					dilate_iter = 8;


	// 		for (int i = 0; i <  dilate_iter; ++i)
	// 			dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			
	// 		erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 		erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 		erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 	}
	// 	else{
	// 		erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 		dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	// 	}
	// }
	// else{
	// 	if (inv){
	// 		// External
	// 		erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 		dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 	}else{
	// 		// Internal
	// 		dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 		erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// 	}
	// }
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

	if (p1.x > height_)
	   p1.x = height_;

	if (p1.y > width_)
	   p1.y = width_;

	if (p1.x < 0)
	   p1.x = 0;

	if (p1.y < 0)
	   p1.y = 0;

	if (p2.x > height_)
	   p1.x = height_;

	if (p2.y > width_)
	   p2.y = width_;

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
			// 	iLowV -= 65;sdfa

			iHighV = CAM_BLUE_HV;

		}break;
		case RED: {

			iLowH = CAM_RED_LH;
			iHighH = CAM_RED_HH;

			iLowS = CAM_RED_LS;
			iHighS = CAM_RED_HS;

			iLowV = CAM_RED_LV;
			iHighV = CAM_RED_HV;

		}break;
		case WHITE: {

			iLowH = CAM_WHITE_LH;
			iHighH = CAM_WHITE_HH;

			iLowS = CAM_WHITE_LS;
			iHighS = CAM_WHITE_HS;

			iLowV = CAM_WHITE_LV;
			iHighV = CAM_WHITE_HV;

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
	inRange(imgHSV, Scalar( CAM_GREEN_LH, CAM_GREEN_LS, CAM_GREEN_LV), Scalar(CAM_GREEN_HH, CAM_GREEN_HS, CAM_GREEN_HV), imgTh); 

	// //morphological opening (remove small objects from the foreground)`

 	// 	imgTh = morph_operation(imgTh); 

	// // //morphological closing (fill small holes in the foreground)
	// imgTh = morph_operation(imgTh, true);

	bitwise_not ( imgTh, imgTh );

	return imgTh;

}

void mark_follower_class::calculate_histogram(cv::Mat src, const int n_bin, int* bin){

	cvtColor(src, src, COLOR_RGB2GRAY);

	const int step = 256 / n_bin;

	for(int i = 0; i < src.rows; ++i)
	{
    		const uchar* srci = src.ptr<uchar>(i);
    		
    		for(int j = 0; j < src.cols; ++j)
				// if ((int) srci[i] <= 90)
				// 	bin[0]++;
				// else
				// 	if (((int) srci[j] > 90) && ((int) srci[j] <= 110))
				// 		bin[1]++;	
				// 	else
				// 		bin[2]++;
    			for (int k = n_bin; k--;)
				if (((int) srci[i] >= k * step) && ((int) srci[i] < (k + 1) * step))
					bin[k]++;
	}
}

Point mark_follower_class::get_median(Mat src){

	cvtColor(src, src, COLOR_RGB2GRAY);

	// imshow("median", src);

	Point p(0, 0);
	int k = 0; 

	for(int i = 0; i < src.rows; ++i)
	{
    		const uchar* srci = src.ptr<uchar>(i);
    		
    		for(int j = 0; j < src.cols; ++j){
			if ((int) srci[j] > 128){
				p.x += j;
				p.y += i;
    				k++;
			}
		}
	}

	p.x /= k;
	p.y /= k;
	
	return p;
}

Point2f mark_follower_class::estimateFOE(vector<Point2f> preP,vector<Point2f> postP){
	Point2f foeTmp;
	int counter = 0;
	float den = 0.0;

	for (uint i = postP.size(); i--; ){
		den = sqrtf(pow(postP[i].x - preP[i].x, 2)+pow(postP[i].y - preP[i].y, 2));
		if (den != 0.0)
			counter++;
	}

	Mat A = Mat::zeros(counter, 2, CV_32F);
	Mat C = Mat::zeros(counter, 1, CV_32F);
	Mat P = Mat::zeros(2, 1, CV_32F);
	counter = 0;

	for (uint i = postP.size(); i--; ){
		den = sqrtf(pow(postP[i].x - preP[i].x, 2) + pow(postP[i].y - preP[i].y, 2));
		if (den != 0.0){
			A.at<float>(counter, 0) = -(postP[i].y - preP[i].y) / den; 
			A.at<float>(counter, 1)=(postP[i].x - preP[i].x) / den;
			C.at<float>(counter, 0) = ( (postP[i].x - preP[i].x) * preP[i].y - (postP[i].y - preP[i].y) * preP[i].x ) / den;
			counter++;
		}
	}

	P= (A.t() * A).inv() * A.t() * C;

	foeTmp.x = P.at<float>(0, 0); 
	foeTmp.y = P.at<float>(1, 0);

	return foeTmp;
}

float mark_follower_class::estimateTau(vector<Point2f> preP, vector<Point2f> postP, Point2f foe){
	
	float Tau = 0.0, den = 0.0;
	int counter = 0;

	for (uint i = 0; i < postP.size(); i++ ){
		den = sqrtf(pow(postP[i].x - preP[i].x, 2) + pow(postP[i].y - preP[i].y, 2));
		if (den >= pow(10, -3) || den == 0.0){
			Tau += sqrtf(pow(foe.x-preP[i].x,2)+pow(foe.y-preP[i].y,2))/den;
			counter++;
		}
	}

	return Tau / counter;
}


float mark_follower_class::tau_extraction(const cv::Mat frame){

    
	// Previous image for optical flow
	Mat gray;

	Point2f foe;

	const int MAX_COUNT = 80;

	float meanTau = 0.0;
	float medianTau = 0.0;

	Mat tmpWinTau = Mat::zeros(3, 1, CV_32F);

	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	// ----------------- here my modifications start ---------------------- //
	cvtColor(frame, gray, COLOR_BGR2GRAY);

	//equalizeHist(gray,gray);
	medianBlur(gray, gray, 5);

	//  --- set appropriate ROI
	//Rect region_of_interest = Rect(640-200, 360-100, 400, 200);
	//gray=gray(region_of_interest);


	if( points_[0].empty()){

		goodFeaturesToTrack(gray, points_[1], MAX_COUNT, 0.01, 10, Mat(), 3, 1, 0.04);
		if (points_[1].empty())
			return -100;

		cornerSubPix(gray, points_[1], subPixWinSize, Size(-1,-1), termcrit);
	}else{

		goodFeaturesToTrack(gray, points_[0], MAX_COUNT, 0.01, 10, Mat(), 3, 1, 0.04);
		
		if (points_[0].empty())
			return -100;

		cornerSubPix(gray, points_[0], subPixWinSize, Size(-1,-1), termcrit);

		vector<uchar> status;
		vector<float> err;
		
		if(prevGray_.empty())
			gray.copyTo(prevGray_);

		calcOpticalFlowPyrLK(prevGray_, gray, points_[0], points_[1], status, err, winSize, 3, termcrit, 0, 0.001);

		// --- FOE estimation via LSM
		foe = estimateFOE(points_[0], points_[1]);

		if (foe.x >15 || foe.y > 15){
			meanTau = estimateTau(points_[0], points_[1], foe);
			int iii = tauCount_%3;
			medianWinTau_.at<float>(tauCount_%3, 0) = meanTau;
			cv::sort(medianWinTau_, tmpWinTau, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
			++tauCount_;
			medianTau = tmpWinTau.at<float>(1, 0);
		}
		else
			medianTau = -1;

		if (meanTau > 500)
			meanTau = 500;

		if (medianTau > 500)
			medianTau = 500;

		// cout << "mean " << meanTau << " " << medianTau << endl;

		// fprintf(pFile,"%3.2f %3.2f\n", meanTau, medianTau);
		// printf("Foe: %3.2f %3.2f - Tau: %3.2f - medianTau: %3.2f\n", foe.x, foe.y, meanTau, medianTau);

	}

	swap(points_[1], points_[0]);
	swap(prevGray_, gray);

	// -----> Draw <----  
	// if (!gray.empty()){

	// 	for (uint i=0; i < points_[1].size(); i++ ){
	// 		circle( gray, points_[1][i], 2, Scalar(0, 255, 0), -1, 8);
	// 		line( gray, points_[0][i], points_[1][i], Scalar(255,255,255) );
	// 	}

	// 	circle( gray, foe, 10, Scalar(0,0,255), 1, 8);
	// 	 imshow("gray frame", gray);
	// }
	// ------------------------


	return medianTau;

}

void mark_follower_class::imageFirstChallengeCallback(const sensor_msgs::ImageConstPtr& msg)
{

	/// ----------------------------------------------------> Time manager <----------------------------------------------------

    	// Get current time  
    	gettimeofday(&time_before, NULL);

	/// -------------------------------------------------------------------------------------------------------------------------------------

	try
	{

		// Get the msg image
		ocvMat_ = cv_bridge::toCvShare(msg, "bgr8")->image;


		// --------------->Pyramid<-------------- 

		// Half resolution image
		resize(ocvMat_, ocvMat_lv1_, Size(ocvMat_.cols / 2, ocvMat_.rows / 2), 0, 0, INTER_NEAREST);

		// Quad resolution image
		resize(ocvMat_, ocvMat_lv2_, Size(ocvMat_.cols / 4, ocvMat_.rows / 4), 0, 0, INTER_NEAREST);
		
		// ----------------------------------------------- 


		Mat thr;

		/// Remove field by color
		thr = remove_field(ocvMat_lv1_);
		// thr = color_detection(ocvMat_lv1_, WHITE);

		imshow("morph", thr);

//	    	BEGIN HOUGH TR

	 //    	Mat src, dst, color_dst;
		
		// src = thr;

	 //    	Canny( src, dst, 50, 200, 3 );
		
		// Skeleton
		// Mat img = thr;
		// cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
		// cv::Mat temp;
		// cv::Mat eroded;
		 
		// cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
		 
		// bool done;		
		// do
		// {
		//   cv::erode(img, eroded, element);
		//   cv::dilate(eroded, temp, element); // temp = open(img)
		//   cv::subtract(img, temp, temp);
		//   cv::bitwise_or(skel, temp, skel);
		//   eroded.copyTo(img);
		 
		//   done = (cv::countNonZero(img) == 0);
		// } while (!done);
		// END Skeleton

		// Filter
		
		Mat dst;
		cvtColor(ocvMat_lv1_, dst, COLOR_RGB2GRAY);

		/// Initialize arguments for the filter
		Point anchor = Point( -1, -1 );
		int delta = 0;
		int ddepth = -1;
	      Mat blured;
		cv::GaussianBlur(ocvMat_lv1_, blured, cv::Size(0, 0), 3);
		cv::addWeighted(ocvMat_lv1_, 1.5, blured, -0.5, 0, blured);


		imshow("blured", blured);
		// inRange(dst, iLowV, iHighV, dst); //Threshold the image

		dst = blured;

		// Mat kernel = (Mat_<double>(3,3) << 0, -1, 0, -1, 4, -1, 0, -1, 0);
		Mat kernel = (Mat_<double>(3,3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
		/// Apply filter
		
		// filter2D(ocvMat_lv1_, dst, ddepth , kernel, anchor, delta, BORDER_DEFAULT );
		filter2D(dst, dst, ddepth , kernel.t(), anchor, delta, BORDER_DEFAULT );

		Scalar value = Scalar( 0,0,0 );
	      copyMakeBorder( dst, dst, 1,1,1,1,BORDER_CONSTANT, value );


	      // dst.convertTo(dst, CV_32F);
	      Mat up =  - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(1, 0, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );


	 //      Mat bottom = - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(1, 2, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );
	 //      Mat right = - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(2, 1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );
	 //      Mat left = - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(0, 1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );
	 //      Mat ur = - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(2, 0, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );
	 //      Mat ul = - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(0, 0, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );
		// Mat br = - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(2, 2, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );
		// Mat bl = - dst(Rect(1,1, ocvMat_lv1_.cols, ocvMat_lv1_.rows) ) + dst(Rect(0, 2, ocvMat_lv1_.cols, ocvMat_lv1_.rows) );

		// Mat trial = (Mat_<double>(3,3) << -255, 0, 1, -4, 0, 2, 255, 0, 1);

		// std::cout << trial << std::endl;
		// Mat bgr[3];   //destination array
		
		// split(up, bgr);//split source  

		// double min, max;
		// cv::minMaxLoc(bgr[0], &min, &max);


		// bgr[0] *= 2;
		// // bgr[0] -= min;
		// // bgr[0] *= 255.0 / (max + abs(min));
		
		// cv::minMaxLoc(bgr[0], &min, &max);



		// cv::minMaxLoc(bgr[1], &min, &max);
		// // bgr[1] -= min;
		// // bgr[1] *= 255.0 / (max + abs(min));
		
		// bgr[1] *= 2;
				

		// cv::minMaxLoc(bgr[2], &min, &max);
		// // bgr[2] -= min;
		// // bgr[2] *= 255.0 / (max + abs(min));
		// bgr[2] *= 2;
		// // bgr[2].convertTo(bgr[2], 1, 180);

		// imshow( "bgr0", bgr[0] );
		// imshow( "bgr1", bgr[1] );
		// imshow( "bgr2", bgr[2] );
		// imshow( "up", up);


	      double min, max;

		Mat simpler(64, 36, CV_32FC1,  cvScalar(0.));

		int count;

		int step_x = (up.cols - 2) / 64.0;
	      int step_y = (up.rows - 2) / 36.0;

		for (int i = 0; i < 64; i++)
			for (int j = 0; j < 36; j++){

				count = sum(up(Rect(  ( i * step_x + 1), (j * step_y + 1), step_x, step_y) ))[0];
				simpler.at<float>(i, j) = count;

			}

		cv::minMaxLoc(simpler, &min, &max);

		simpler -= min;
		simpler *= 255.0 / max;
		// std::cout << " p " <<  min  <<" "<< max << endl;

		// cv::minMaxLoc(simpler, &min, &max);

// 		std::cout << min  <<" "<< max << endl;
// cout << simpler << endl;
		// resize(simpler, simpler, Size(640,360), 0, 0, INTER_NEAREST);


		Mat templ (3, 3, CV_32FC1);

		templ.at<float>(0, 0) = 1;
		templ.at<float>(0,1) = 0;
		templ.at<float>(0,2) = 1;
		templ.at<float>(1,0) = 0;
		templ.at<float>(1,1) = 1;
		templ.at<float>(1,2) = 0;
		templ.at<float>(2,0) = 1;
		templ.at<float>(2,1) = 0;
		templ.at<float>(2,2) = 1;


		int match_method = CV_TM_CCOEFF;


  /// Create the result matrix
	int result_cols =  simpler.cols - templ.cols + 1;
	int result_rows = simpler.rows - templ.rows + 1;

	Mat result( result_rows, result_cols, CV_32FC1 );

	/// Do the Matching and Normalize
	matchTemplate( simpler, templ, result, match_method );

	 /// Localizing the best match with minMaxLoc
  	Point minLoc; Point maxLoc;
  	
  	minMaxLoc( result, &min, &max, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  
  /// Show me what you got
  // rectangle( simpler_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, maxLoc, Point( maxLoc.x + templ.cols , maxLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  // imshow( image_window, simpler_display );
  imshow( "result_window", result );

		imshow("d", simpler);
		// std::cout << simpler<< std::endl;



		// imshow("dst", dst);

		// merge(bgr, 3, up);

		// split(bottom, bgr);
		// cv::minMaxLoc(bgr[0], &min, &max);
		// bgr[0] -= min;
		// bgr[0] *= 255.0 / (max + abs(min));

		// cv::minMaxLoc(bgr[1], &min, &max);
		// bgr[1] -= min;
		// bgr[1] *= 255.0 / (max + abs(min));

		// cv::minMaxLoc(bgr[2], &min, &max);
		// bgr[2] -= min;
		// bgr[2] *= 255.0 / (max + abs(min));

		// merge(bgr, 3, bottom);

		// split(right, bgr);
		// cv::minMaxLoc(bgr[0], &min, &max);
		// bgr[0] -= min;
		// bgr[0] *= 255.0 / (max + abs(min));

		// cv::minMaxLoc(bgr[1], &min, &max);
		// bgr[1] -= min;
		// bgr[1] *= 255.0 / (max + abs(min));

		// cv::minMaxLoc(bgr[2], &min, &max);
		// bgr[2] -= min;
		// bgr[2] *= 255.0 / (max + abs(min));

		// merge(bgr, 3, right);

		// split(left, bgr);
		// cv::minMaxLoc(bgr[0], &min, &max);
		// bgr[0] -= min;
		// bgr[0] *= 255.0 / (max + abs(min));

		// cv::minMaxLoc(bgr[1], &min, &max);
		// bgr[1] -= min;
		// bgr[1] *= 255.0 / (max + abs(min));

		// cv::minMaxLoc(bgr[2], &min, &max);
		// bgr[2] -= min;
		// bgr[2] *= 255.0 / (max + abs(min));

		// merge(bgr, 3, left);


		// std::cout << min <<" " << max << std::endl;
		
		// inRange(up, 20, 255, up); //Threshold the image
		// inRange(bottom, 20, 255, bottom); //Threshold the image
		// inRange(right, 20, 255, right); //Threshold the image
		// inRange(left, 20, 255, left); //Threshold the image
		// inRange(ur, 20, 255, ur); //Threshold the image
		// inRange(ul, 20, 255, ul); //Threshold the image
		// inRange(br, 20, 255, br); //Threshold the image
		// inRange(bl, 20, 255, bl); //Threshold the image


		// cv::addWeighted(up, 1.0, bottom, 1.0, 0.0, dst);
		// cv::addWeighted(dst, 1.0, right, 1.0, 0.0, dst);
		// cv::addWeighted(dst, 1.0, left, 1.0, 0.0, dst);
		// cv::addWeighted(dst, 1.0, ur, 1.0, 0.0, dst);
		// cv::addWeighted(dst, 1.0, ul, 1.0, 0.0, dst);
		// cv::addWeighted(dst, 1.0, br, 1.0, 0.0, dst);		
		// cv::addWeighted(dst, 1.0, bl, 1.0, 0.0, dst);

		// imshow( "up", up );
		// imshow( "bt", bottom );
		// imshow( "r", right );
		// imshow( "l", left);
		// imshow( "ul", ul);
		// imshow( "ur", ur);
		// imshow( "br", br);
		// imshow( "bl", bl);

		// imshow("dst", dst);


    // Filter end

	 //      vector<Vec4i> lines;

	 //      HoughLinesP( dst, lines, 1, CV_PI / 180, 80, 40, 20 );
	 	
	      // color_dst = ocvMat_lv1_;

		// std::vector<Point> pointIntersectV;

		// Intersect Point
		Point T_Intersect(-1, -1); 

		// pointIntersectV = getIntersection(lines);

		// T_Intersect = searchNPG(pointIntersectV);

		// if ((T_Intersect.x != -1) && (T_Intersect.y == -1)){

		// 	for( size_t i = 0; i < pointIntersectV.size(); i++ )
		// 		circle( color_dst, pointIntersectV[i], 2, Scalar(150, 255, 100), 3, 16);

		// 	// Plot target Intersect 
		// 	circle( color_dst, T_Intersect, 2, Scalar(255, 150, 100), 3, 16);	
		// }
	
		// imshow( "Detected Lines", color_dst );
	    	// imshow("canny", skel);
		
	     // END HOUGH TR
	
	     // FIRST CHALLENGE WORK

		// Get the msg image
		// ocvMat_ = cv_bridge::toCvShare(msg, "bgr8")->image;

		// // --------------->Pyramid<-------------- 

		// // Half resolution image
		// resize(ocvMat_, ocvMat_lv1_, Size(ocvMat_.cols / 2, ocvMat_.rows / 2), 0, 0, INTER_NEAREST);

		// // Quad resolution image
		// resize(ocvMat_, ocvMat_lv2_, Size(ocvMat_.cols / 4, ocvMat_.rows / 4), 0, 0, INTER_NEAREST);
		
		// // ----------------------------------------------- 

		// Check histogram
		int binH[2] = {0, 0};

		calculate_histogram(ocvMat_lv1_, 2, &binH[0]);
		// cout << binH[0] << " " << binH[1]<< " " << binH[2] << " " << binH[0] + binH[1]+ binH[2] << endl;

		Point T_Near; 

		if ((double) binH[1] / binH[0] > 3.5){
			T_Near = get_median(ocvMat_lv1_);	

			T_Intersect = T_Near;

			// cout << "T_Intersect" << T_Intersect << endl;
		}

		
		// if ((T_Intersect.x != -1) && (T_Intersect.y == -1))
		// 	// Plot target Intersect 
		// 	circle( color_dst, T_Intersect, 2, Scalar(255, 150, 100), 3, 16);	

		// imshow( "Detected Lines", color_dst );

		// Mat thr;

		vector<descriptor> blob_desc;
	
		/// Remove field by color
		// thr = remove_field(ocvMat_lv1_);

		// Show removing field result
		// imshow("Removed Field", thr );

		// Morphologic operations
		thr = morph_operation(thr); 

		// imshow("Morphologic", thr );

		// When image is empty go to the next frame
		if (thr.empty()){
			ROS_INFO("Empty after the morphological operations");
			return;
		}
		
		// Define message to send

		mark_follower::markPoseStamped msg2pub;

		// Get contours of the blobs

		blob_desc = get_blobFirstCh(thr);

		// Call matching function on the blob descriptor discovered by get_blob()

		for (int i = blob_desc.size(); i--;)
			t_matching(blob_desc[i], tmpl_);

		// Find out the best match

		// Take target from matching
		bool is_square = false;
		Point T_matching = sl.get_max(&is_square);

		// // Clear sorted list
		sl.clear();

		// No target found

		if (( (T_Intersect.x == -1) && (T_Intersect.y == -1)) && ( (T_matching.x == -1) && (T_matching.y == -1))){
			budgetResidual_--;

			if (budgetResidual_ < 0)
				refVariance_ = 0;
		} 
		else{
	
			double w1 = 0, w2 = 0;

			// One target found at least
			// Modulate weight

			if (( (T_Intersect.x != -1) && (T_Intersect.y != -1)) && ( (T_matching.x != -1) && (T_matching.y != -1))){

				w1 = (altitude_ - 1 ) / 2.5;

				if (w1 > 1.0)
					w1 = 1.0;
				else 
					if (w1 < 0.0)
						w1 = 0.0;


			} else {

				if ((T_Intersect.x != -1) && (T_Intersect.y != -1))
					w1 = 0;
				else
					if ((T_matching.x != -1) && (T_matching.y != -1))
						w1 = 1;
			}

			w2 = 1 - w1;			

			//  Weighted sum

			target_.x = 2 * (T_matching.x * w1 + T_Intersect.x * w2);
			target_.y = 2 * (T_matching.y * w1 + T_Intersect.y * w2);
			
			// Kalman Filter 
			if (!state_kf_){
				init_kalman(target_);
				state_kf_ = true;
			}
			else
				target_ = kalman(target_);  
			
			// Draw point
			circle( ocvMat_, target_, 2, Scalar(255, 0, 0), 3, 16);


			budgetResidual_ = 10;

			// cout << is_square << " " << ((T_Intersect.x != -1) && (T_Intersect.y != -1)) << endl;	

			if (is_square || ((T_Intersect.x != -1) && (T_Intersect.y != -1)) )
				refVariance_++;
			else
				refVariance_ -= .25;

			// Saturation

			if(refVariance_ > 130)
				refVariance_ = 130;

		}

		// int ROI_x, ROI_y, ROI_Tx, ROI_Ty;
		// int dim = 300;

		// if (target_.x - dim / 2 < 0){
		// 	ROI_Tx = target_.x;

		// 	if (target_.x + dim/2 > width_){
		// 		ROI_x = width_ - target_.x;
		// 		ROI_Tx = dim / 2 + (dim - ROI_x);
		// 	}
		// 	else
		// 		ROI_x = dim;
		// }
		// else{

		// 	ROI_Tx = dim/2;

		// 	if (target_.x + dim/2 > width_){
		// 		ROI_x = width_ - target_.x;
		// 		ROI_Tx = dim / 2 + (dim - ROI_x);
		// 	}
		// 	else
		// 		ROI_x = dim;
		// }
	
		// if (target_.y - dim/2 < 0){
		// 	ROI_Ty = target_.y;

		// 	if (target_.y + dim/2 > height_){ 
		// 		ROI_y = height_ - target_.y;
		// 		ROI_Ty = dim / 2 + (dim - ROI_y);
		// 	}
		// 	else
		// 		ROI_y = dim;
		// }
		// else{
			
		// 	ROI_Ty = dim/2;

		// 	if (target_.y + dim/2 > height_){
		// 		ROI_y = height_ - target_.y;
		// 		ROI_Ty = dim / 2 + (dim / 2  - ROI_y);
		// 		// cout <<CAMERA_VID_HEIGHT << " " << ROI_y << " " << ROI_Ty << endl;
		// 	}
		// 	else
		// 		ROI_y = dim;
		// }

		// // cout << target_ << endl;
		// // cout <<(int) (target_.x - ROI_Tx) << " " <<  (int) (target_.y - ROI_Ty) << " " << (int) (target_.x - ROI_Tx) + ROI_x << " "  <<  (int) (target_.y - ROI_Ty) + ROI_y << endl;

		// ROI_ = ocvMat_(Rect((int) (target_.x - ROI_Tx), (int) (target_.y - ROI_Ty), dim, dim));

		// // cout << ROI_.size() << endl;

		// imshow("ROI", ROI_);

		// cout << " XXX" << endl;
		// Populate message

		msg2pub.stamp = ros::Time::now();
		
		msg2pub.x = target_.x;
		msg2pub.y = target_.y;

		msg2pub.budgetResidual = budgetResidual_;
		msg2pub.variance = refVariance_;
		// msg2pub.tau = tau_extraction(ROI_); 
//		msg2pub.tau = tau_extraction(ocvMat_lv1_); 
		// cout << msg2pub.tau << endl;
		// cout << budgetResidual_ << " " << target_.x << " " << target_.y << " " << msg2pub.tau << endl;

		markTarget_pub_.publish(msg2pub);




		cv::imshow("view", ocvMat_);
		cv::waitKey(150);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

	/// ----------------------------------------------------> Time manager <----------------------------------------------------

	gettimeofday(&time_after, NULL);

	// cout << "Time: " << diff_ms(time_after, time_before) << endl;

	/// -------------------------------------------------------------------------------------------------------------------------------------

}

void mark_follower_class::imageThirdChallengeCallback(const sensor_msgs::ImageConstPtr& msg)
{
	/// ----------------------------------------------------> Time manager <----------------------------------------------------

    	// Get current time  
    	gettimeofday(&time_before, NULL);

	/// ----
	try
	{

		// Get the msg image
		ocvMat_ = cv_bridge::toCvShare(msg, "bgr8")->image;
		// --------------->Pyramid<-------------- 

		// Half resolution image
		resize(ocvMat_, ocvMat_lv1_, Size(ocvMat_.cols / 2, ocvMat_.rows / 2), 0, 0, INTER_NEAREST);

		// Quad resolution image
		resize(ocvMat_, ocvMat_lv2_, Size(ocvMat_.cols / 4, ocvMat_.rows / 4), 0, 0, INTER_NEAREST);
		
		// ----------------------------------------------- 
		Mat thr, thr_red, thr_black, thr_blue;
		vector<descriptor> blob_desc;

		// Last pos
		cv::Point lastTarget = target_;
		
		/// Remove field by color

		thr_blue = color_detection(ocvMat_lv1_, BLUE);
		thr_red = color_detection(ocvMat_lv1_, RED);

		cv::addWeighted(thr_blue, 1.0, thr_red, 1.0, 0.0, thr);

		// Show removing field result
		imshow("Removed Field", thr );

		// When image is empty go to the next frame
		if (thr.empty()){
			ROS_INFO("Empty after the morphological operations");
			return;
		}
		// Define message to send

		mark_follower::markPoseStamped msg2pub;

		Mat bgr[3];   //destination array
		Mat ocvMat_en;
		split(ocvMat_lv1_, bgr);//split source  

		bitwise_and(bgr[0], thr, bgr[0]);
		bitwise_and(bgr[1], thr, bgr[1]);
		bitwise_and(bgr[2], thr, bgr[2]);

		merge(bgr, 3, ocvMat_en);

		// Get contours of the blobs

		blob_desc = get_blobThirdCh(ocvMat_en);

		double near_dist = -1;

		if (blob_desc.empty()){
			budgetResidual_--;

			if (budgetResidual_ < 0)
				refVariance_ = 0;
		}
		else{
			for (int i = blob_desc.size(); i--;){

				double tmp_dist = sqrt(pow(blob_desc[i].origin.x - thr.cols, 2) + pow(blob_desc[i].origin.y - thr.rows, 2));

				if (near_dist > tmp_dist || (near_dist == -1)){
					near_dist = tmp_dist;
					target_ = 2 * blob_desc[i].origin;
				}

				refVariance_++;
				budgetResidual_ = 10;
				
			}
		}
		// Call matching function on the blob descriptor discovered by get_blob()

		// Find out the best match

		// Kalman Filter 
		if (!state_kf_){
			if (budgetResidual_ > 0){
				init_kalman(target_);
				state_kf_ = true;
			}
		}
		else{
			if (budgetResidual_ > 0)
				target_ = kalman(target_);  
		}
		
		// Draw point
		circle( ocvMat_, target_, 2, Scalar(200, 210, 100), 3, 16);

		// Populate message

		msg2pub.stamp = ros::Time::now();
		
		msg2pub.x = target_.x;
		msg2pub.y = target_.y;

		msg2pub.budgetResidual = budgetResidual_;
		msg2pub.variance = refVariance_;
		msg2pub.tau = 0;

		markTarget_pub_.publish(msg2pub);

		// cout << budgetResidual_ << "\t" << target_.x << "\t'" << target_.y<< endl; 

		line(ocvMat_, Point(440, 160), Point(440, 560), Scalar( 255, 0 , 0));
		line(ocvMat_, Point(440, 160), Point(840, 160), Scalar( 255, 0 , 0));
		line(ocvMat_, Point(840, 160), Point(840, 560), Scalar( 255, 0 , 0));
		line(ocvMat_, Point(440, 560), Point(840, 560), Scalar( 255, 0 , 0));

		cv::imshow("view", ocvMat_);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
		/// ----------------------------------------------------> Time manager <----------------------------------------------------

	gettimeofday(&time_after, NULL);

	// cout << "Time: " << diff_ms(time_after, time_before) << endl;

	/// -------------------------------------------------------------------------------------------------------------------------------------
}

void mark_follower_class::altitudeCallback(const std_msgs::Float64Ptr &msg)
{
	altitude_ = msg->data;
}

void mark_follower_class::trial()
{
		while(ros::ok()){
			// get_static_frame();
			get_static_frame();

		// Half resolution image
		resize(ocvMat_, ocvMat_lv1_, Size(ocvMat_.cols / 2, ocvMat_.rows / 2), 0, 0, INTER_NEAREST);

		// Quad resolution image
		resize(ocvMat_, ocvMat_lv2_, Size(ocvMat_.cols / 4, ocvMat_.rows / 4), 0, 0, INTER_NEAREST);
		
		// ----------------------------------------------- 
		Mat thr, thr_red, thr_black, thr_blue;
		vector<descriptor> blob_desc;

		// Last pos
		cv::Point lastTarget = target_;
		
		/// Remove field by color

		thr_blue = color_detection(ocvMat_lv1_, BLUE);
	
		thr_red = color_detection(ocvMat_lv1_, RED);

		cv::addWeighted(thr_blue, 1.0, thr_red, 1.0, 0.0, thr);

		// Show removing field result
		imshow("Removed Field", thr );

		// When image is empty go to the next frame
		if (thr.empty()){
			ROS_INFO("Empty after the morphological operations");
			return;
		}
		// Define message to send

		mark_follower::markPoseStamped msg2pub;

		Mat bgr[3];   //destination array
		Mat ocvMat_en;
		split(ocvMat_lv1_, bgr);//split source  

		bitwise_and(bgr[0], thr, bgr[0]);
		bitwise_and(bgr[1], thr, bgr[1]);
		bitwise_and(bgr[2], thr, bgr[2]);

		merge(bgr, 3, ocvMat_en);

		// Get contours of the blobs

		blob_desc = get_blobThirdCh(ocvMat_en);

		double near_dist = -1;

		if (blob_desc.empty()){
			budgetResidual_--;

			if (budgetResidual_ < 0)
				refVariance_ = 0;
		}
		else{
			for (int i = blob_desc.size(); i--;){

				double tmp_dist = sqrt(pow(blob_desc[i].origin.x - thr.cols, 2) + pow(blob_desc[i].origin.y - thr.rows, 2));

				if (near_dist > tmp_dist || (near_dist == -1)){
					near_dist = tmp_dist;
					target_ = 2 * blob_desc[i].origin;
				}

				refVariance_++;
				budgetResidual_ = 10;
				
			}
		}
		// Call matching function on the blob descriptor discovered by get_blob()

		// Find out the best match


		// Kalman Filter 
		if (!state_kf_){
			if (budgetResidual_ > 0){
				init_kalman(target_);
				state_kf_ = true;
			}
		}
		else{
			if (budgetResidual_ > 0)
				target_ = kalman(target_);  
		}
		
		// Draw point
		circle( ocvMat_, target_, 2, Scalar(200, 210, 100), 3, 16);

		// Populate message

		msg2pub.stamp = ros::Time::now();
		
		msg2pub.x = target_.x;
		msg2pub.y = target_.y;

		msg2pub.budgetResidual = budgetResidual_;
		msg2pub.variance = refVariance_;
		msg2pub.tau = 0;

		markTarget_pub_.publish(msg2pub);

		line(ocvMat_, Point(540, 260), Point(540, 460), Scalar( 255, 0 , 0));
		line(ocvMat_, Point(540, 260), Point(740, 260), Scalar( 255, 0 , 0));
		line(ocvMat_, Point(740, 260), Point(740, 460), Scalar( 255, 0 , 0));
		line(ocvMat_, Point(540, 460), Point(740, 460), Scalar( 255, 0 , 0));

		cv::imshow("view", ocvMat_);
		cv::waitKey(30);
	}
 }

std::vector<Point> mark_follower_class::getIntersection(std::vector<Vec4i> lines){
		
	std::vector<double> m, q;
	double tmp_m, tmp_q;

	// Get m and q of lines
	for (int i = lines.size(); i--;){

		tmp_m = (double) (lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
		m.push_back( tmp_m );
		
		tmp_q = lines[i][3] - tmp_m * lines[i][2];
		q.push_back(tmp_q);
	}

	double perpendicular_coeff;
	std::vector<Point> crossV;
	Point crossP;

	// Search perpendicular and takes the cross point

	for (int i = m.size(); i--;)
		for (int j = m.size() - 1; j--;){
			
			perpendicular_coeff = approx_perpendicular(m[i], m[j]);

			if (perpendicular_coeff < 0.01 && (perpendicular_coeff > -0.01)){

				crossP.x = (q[j] - q[i]) / (m[i] - m[j]);
				crossP.y = m[i]* crossP.x + q[i];

				crossV.push_back(crossP);

			}
		}
	
	return crossV;
}

Point mark_follower_class::searchNPG(std::vector<Point> IPVec){

	for (int i = IPVec.size(); i--;){
		int count = 0;
		Point IPmean = IPVec[i];
		for (int j = IPVec.size() - 1; j--;){

			double d = sqrt(pow(IPVec[i].x - IPVec[j].x, 2) + pow(IPVec[i].y - IPVec[j].y, 2));

			if (d < 5){
				IPmean.x += IPVec[j].x;
				IPmean.y += IPVec[j].y;
				count++;
			}
		}
		if (count == 3){
			IPmean.x /= 4;
			IPmean.y /= 4;
			return IPmean ;
		}
	}

	return Point(-1,-1);
}