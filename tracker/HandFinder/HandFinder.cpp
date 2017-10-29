#include "HandFinder.h"

#include <numeric> ///< std::iota
#include <fstream> ///< ifstream
#include "util/mylogger.h"
#include "util/opencv_wrapper.h"
#include "util/qfile_helper.h"
#include "tracker/Worker.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Detection/TrivialDetector.h"
//#include "tracker/Legacy/util/Util.h"
#include "./connectedComponents.h" ///< only declared in opencv3

#include "tracker/TwSettings.h"

HandFinder::HandFinder(Camera *camera, Handedness handedness, std::string data_path, int user_num) : camera(camera){
    CHECK_NOTNULL(camera);
	sensor_indicator = new int[upper_bound_num_sensor_points];

	this->handedness = handedness;
    tw_settings->tw_add(settings->show_hand, "show_hand", "group=HandFinder");
    tw_settings->tw_add(settings->show_wband, "show_wband", "group=HandFinder");
    tw_settings->tw_add(settings->wband_size, "wband_size", "group=HandFinder");
    tw_settings->tw_add(settings->depth_range, "depth_range", "group=HandFinder");

#ifdef TODO_TWEAK_WRISTBAND_COLOR
     // TwDefine(" Settings/classifier_hsv_min colormode=hls ");
     TwAddVarRW(tw_settings->anttweakbar(), "rgb_min", TW_TYPE_COLOR3F,  &_settings.hsv_min.data, "group=HandFinder");
     TwAddVarRW(tw_settings->anttweakbar(), "rgb_max", TW_TYPE_COLOR3F,  &_settings.hsv_max.data, "group=HandFinder");
#endif

	 std::string path;
	 if (handedness == right_hand || handedness == both_hands) {
		 if (user_num == 6 || user_num == 11 || user_num == 13 || user_num == 14 || user_num == 15)
			 path = data_path + "wristbands/yellow_wristband.txt";
		 else
			path = data_path + "wristbands/blue_wristband.txt";  //usually blue
		 //path = local_file_path("blue_wristband.txt", true);
	 }
	 else {
		 path = data_path + "wristbands/yellow_wristband.txt";
		 //path = local_file_path("yellow_wristband.txt", true);
	 }
    
    if(!path.empty()){
        std::cout << "Reading Wristband Colors from: " << path << std::endl;
        ifstream myfile(path);
        std::string dump;
        myfile >> dump; ///< "hsv_min:"
        myfile >> settings->hsv_min[0];
        myfile >> settings->hsv_min[1];
        myfile >> settings->hsv_min[2];
        myfile >> dump; ///< "hsv_max:"
        myfile >> settings->hsv_max[0];
        myfile >> settings->hsv_max[1];
        myfile >> settings->hsv_max[2];
        //std::cout << "  hsv_min: " << settings->hsv_min << std::endl;
        //std::cout << "  hsv_max: " << settings->hsv_max << std::endl;
    }
}

Vector3 point_at_depth_pixel(cv::Mat& depth, int x, int y, Camera* camera) {
	Integer z = depth.at<unsigned short>(y, x);
	return camera->depth_to_world(x, y, z);
}

void HandFinder::binary_classification(cv::Mat& depth, cv::Mat& color) {    
    _wristband_found = false;
	
    TIMED_SCOPE(timer, "Worker::binary_classification");

    ///--- Fetch from settings
    cv::Scalar hsv_min = settings->hsv_min;
    cv::Scalar hsv_max = settings->hsv_max;
    Scalar wband_size = _settings.wband_size;
    Scalar depth_range= _settings.depth_range;

    ///--- We look for wristband up to here...
    Scalar depth_farplane = camera->zFar();

    Scalar crop_radius = depth_range;  //WAS 150. Cropping fingertips. 

    ///--- Allocated once
    static cv::Mat color_hsv;
    static cv::Mat in_z_range;

    // TIMED_BLOCK(timer,"Worker_classify::(convert to HSV)")
    {
        cv::cvtColor(color, color_hsv, CV_RGB2HSV);
        cv::inRange(color_hsv, hsv_min, hsv_max, /*=*/ mask_wristband);
        cv::inRange(depth, camera->zNear(), depth_farplane /*mm*/, /*=*/ in_z_range);
        cv::bitwise_and(mask_wristband, in_z_range, mask_wristband);
		//cv::imshow("mask_wristband (pre)", mask_wristband); cv::waitKey(1);
    }

    // TIMED_BLOCK(timer,"Worker_classify::(robust wrist)")
    {
        cv::Mat labels, stats, centroids;
        int num_components = cv::connectedComponentsWithStats(mask_wristband, labels, stats, centroids, 4 /*connectivity={4,8}*/);       

        ///--- Generate array to sort
        std::vector< int > to_sort(num_components);
        std::iota(to_sort.begin(), to_sort.end(), 0 /*start from*/);       

        ///--- Sort accoding to area
        auto lambda = [stats](int i1, int i2){
            int area1 = stats.at<int>(i1,cv::CC_STAT_AREA);
            int area2 = stats.at<int>(i2,cv::CC_STAT_AREA);
            return area1>area2;
        };
        std::sort(to_sort.begin(), to_sort.end(), lambda);

        if(num_components<2 /*not found anything beyond background*/){            		
            _has_useful_data = false;
        }
        else
        {
            if(_has_useful_data==false){
                //std::cout << "NEW useful data => reinit" << std::endl;
                //trivial_detector->exec(frame, sensor_silhouette);
            }
            _has_useful_data = true;
            
            ///--- Select 2nd biggest component
            mask_wristband = (labels==to_sort[1]);
            _wristband_found = true;
        }
    }

	if (_settings.show_wband) {
		cv::imshow("show_wband", mask_wristband);
		cv::waitKey(1);
	}		
    else
        cv::destroyWindow("show_wband");

    // TIMED_BLOCK(timer,"Worker_classify::(crop at wrist depth)")
    {
        ///--- Extract wristband average depth
        std::pair<float, int> avg;
        for (int row = 0; row < mask_wristband.rows; ++row) {
            for (int col = 0; col < mask_wristband.cols; ++col) {
                float depth_wrist = depth.at<ushort>(row,col);
                if(mask_wristband.at<uchar>(row,col)==255){
                     if(camera->is_valid(depth_wrist)){
                         avg.first += depth_wrist;
                         avg.second++;
                     }
                 }
            }
        }
        ushort depth_wrist = (avg.second==0) ? camera->zNear() : avg.first / avg.second; 

        ///--- First just extract pixels at the depth range of the wrist
        cv::inRange(depth, depth_wrist-depth_range, /*mm*/
                           depth_wrist+depth_range, /*mm*/
                           sensor_silhouette /*=*/);
    }

    // cv::imshow("sensor_silhouette (before)", sensor_silhouette);

    _wband_center = Vector3(0,0,0);
    _wband_dir = Vector3(0,0,-1);
    // TIMED_BLOCK(timer,"Worker_classify::(PCA)")
    {
        ///--- Compute MEAN
        int counter = 0;
        for (int row = 0; row < mask_wristband.rows; ++row){
            for (int col = 0; col < mask_wristband.cols; ++col){
                if(mask_wristband.at<uchar>(row,col)!=255) continue;
				_wband_center += point_at_depth_pixel(depth, col, row, camera);
                counter ++;
            }
        }
        _wband_center /= counter;
        std::vector<Vector3> pts; pts.push_back(_wband_center);

        ///--- Compute Covariance
        static std::vector<Vector3> points_pca;
        points_pca.reserve(100000);
        points_pca.clear();		
        for (int row = 0; row < sensor_silhouette.rows; ++row){
            for (int col = 0; col < sensor_silhouette.cols; ++col){
                if(sensor_silhouette.at<uchar>(row,col)!=255) continue;
				Vector3 p_pixel = point_at_depth_pixel(depth, col, row, camera);
                if((p_pixel-_wband_center).norm()<100){
                    // sensor_silhouette.at<uchar>(row,col) = 255;
                    points_pca.push_back(p_pixel);
                } else {
                    // sensor_silhouette.at<uchar>(row,col) = 0;
                }
            }
        }
        if (points_pca.size() == 0) return;
        ///--- Compute PCA
        Eigen::Map<Matrix_3xN> points_mat(points_pca[0].data(), 3, points_pca.size() );       
        for(int i : {0,1,2})
            points_mat.row(i).array() -= _wband_center(i);
        Matrix3 cov = points_mat*points_mat.adjoint();
        Eigen::SelfAdjointEigenSolver<Matrix3> eig(cov);
        _wband_dir = eig.eigenvectors().col(2);

        ///--- Allow wrist to point downward
        if(_wband_dir.y()<0)
            _wband_dir = -_wband_dir;
    }
    // TIMED_BLOCK(timer,"Worker_classify::(in sphere)")
    {
		wband_size = 10;
        Scalar crop_radius_sq = crop_radius*crop_radius;
        Vector3 crop_center = _wband_center + _wband_dir*( crop_radius - wband_size /*mm*/);
		//Vector3 crop_center = _wband_center + _wband_dir*( crop_radius + wband_size /*mm*/);

        for (int row = 0; row < sensor_silhouette.rows; ++row){
            for (int col = 0; col < sensor_silhouette.cols; ++col){
                if(sensor_silhouette.at<uchar>(row,col)!=255) continue;

				Vector3 p_pixel = point_at_depth_pixel(depth, col, row, camera);
                if((p_pixel-crop_center).squaredNorm() < crop_radius_sq)
                    sensor_silhouette.at<uchar>(row,col) = 255;
                else
                    sensor_silhouette.at<uchar>(row,col) = 0;
            }
        }
    }

    if(_settings.show_hand){
        cv::imshow("show_hand", sensor_silhouette);
    } else {
        cv::destroyWindow("show_hand");
    }
}

bool HandFinder::find_wristband_color(cv::Mat& depth, cv::Mat& color) {

	cv::Mat color_hsv;
	cv::Mat in_z_range;

	cv::cvtColor(color, color_hsv, CV_RGB2HSV);
	cv::inRange(depth, 150, 450 /*mm*/, /*=*/ in_z_range);

	cv::Mat blue;
	cv::Mat yellow;
	cv::Mat green;
	cv::Mat orange;
	//blue
	cv::Scalar blue_min(90, 240, 37);
	cv::Scalar blue_max(107, 255, 255);
	cv::inRange(color_hsv, blue_min, blue_max, /*=*/ blue);
	cv::bitwise_and(blue, in_z_range, blue);
	
	//green
	cv::Scalar green_min(50, 111, 37);
	cv::Scalar green_max(70, 255, 255);
	cv::inRange(color_hsv, green_min, green_max, /*=*/ green);
	cv::bitwise_and(green, in_z_range, green);

	//orange
	cv::Scalar orange_min(2, 111, 37);
	cv::Scalar orange_max(12, 255, 255);
	cv::inRange(color_hsv, orange_min, orange_max, /*=*/ orange);
	cv::bitwise_and(orange, in_z_range, orange);

	//orange
	cv::Scalar yellow_min(14, 111, 37);
	cv::Scalar yellow_max(34, 255, 255);
	cv::inRange(color_hsv, yellow_min, yellow_max, /*=*/ yellow);
	cv::bitwise_and(yellow, in_z_range, yellow);

	int blue_pix = 0;
	int green_pix = 0;
	int orange_pix = 0;
	int yellow_pix = 0;
	for (int row = 0; row < in_z_range.rows; ++row) {
		for (int col = 0; col < in_z_range.cols; ++col) {
			if (blue.at<uchar>(row, col) == 255) 
				blue_pix++;
			if (yellow.at<uchar>(row, col) == 255)
				yellow_pix++;
			if (green.at<uchar>(row, col) == 255)
				green_pix++;
			if (orange.at<uchar>(row, col) == 255)
				orange_pix++;
		}
	}

	cout << "Blue: " << blue_pix << " Green: " << green_pix << " Orange: " << orange_pix << " Yellow: " << yellow_pix << endl;
	
	if (!blue_pix && !green_pix && !orange_pix && !yellow_pix)
		return false;

	if (blue_pix > green_pix) {
		if (yellow_pix > orange_pix) {
			if (blue_pix > yellow_pix) {
				cout << "Blue wristband" << endl;
				_settings.hsv_min = blue_min;
				_settings.hsv_max = blue_max;
			}
			else {
				cout << "Yellow wristband" << endl;
				_settings.hsv_min = yellow_min;
				_settings.hsv_max = yellow_max;
			}
		}
		else {
			if (blue_pix > orange_pix) {
				cout << "Blue wristband" << endl;
				_settings.hsv_min = blue_min;
				_settings.hsv_max = blue_max;
			}
			else {
				cout << "Orange wristband" << endl;
				_settings.hsv_min = orange_min;
				_settings.hsv_max = orange_max;
			}
		}
	}
	else {
		if (yellow_pix > orange_pix) {
			if (green_pix > yellow_pix) {
				cout << "Green wristband" << endl;
				_settings.hsv_min = green_min;
				_settings.hsv_max = green_max;
			}
			else {
				cout << "Yellow wristband" << endl;
				_settings.hsv_min = yellow_min;
				_settings.hsv_max = yellow_max;
			}
		}
		else {
			if (green_pix > orange_pix) {
				cout << "Green wristband" << endl;
				_settings.hsv_min = green_min;
				_settings.hsv_max = green_max;
			}
			else {
				cout << "Orange wristband" << endl;
				_settings.hsv_min = orange_min;
				_settings.hsv_max = orange_max;
			}
		}
	}
	_settings.show_wband = true;
	
	return true;
}