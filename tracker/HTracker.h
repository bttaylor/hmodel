#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <QTimer>
#include <QObject>
#include "util/mylogger.h"
#include "util/tictoc.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Worker.h"
#include "tracker/Data/SolutionStream.h"
#include "tracker/Detection/QianDetection.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/TwSettings.h"
#include "tracker/HModel/Model.h"

#include "tracker/Energy/Fitting/OnlinePerformanceMetrics.h"

#include <ctime>
#include <math.h>
#include <iomanip>
#include <myo/myo.hpp>

#include <FaceTracker/Tracker.h>
//#include "tracker/GLWidget.h"

class HTracker : public QTimer {
	
public:
	//enum Mode { LIVE, BENCHMARK, PLAYBACK,SENSOR_ONLY } 
	Mode mode = LIVE;
	Sensor* sensor;
	DataStream* datastream;
	SolutionStream* solutions;
	//Worker*const worker = NULL;
	Worker* worker = NULL;
	myo::Hub* hub = NULL;

	OnlinePeformanceMetrics online_performance_metrics;

	std::string sequence_path;
	bool real_color;

public:
	float current_fps = 0;
	int first_frame_lag = 0;

	bool tracking_failed = true;
	bool initialization_enabled = true;
	bool tracking_enabled = true;
	bool verbose = false;

	//Brandon
	std::vector<std::vector<float>> thetas;
	bool classified = true;
	std::string word_guess;
	double corrcoefs[3];
	bool record_only;
	bool myoEnable;
	FACETRACKER::Tracker model;
	cv::Mat tri;
	cv::Mat con;
	bool showFaceTrack = false;
	bool manual_mode = false;   //This should only affect Benchmark mode
	bool calibrate = false;
	bool handshape = true;
	int handshape_set = 1;
	int handshape_set_max = 3;
	bool finished = false;
	std::string suffix = "1016";

	int user_num;
	std::string word;
	int word_i = 0;
	std::vector<std::string> words;
	int current_frame = 0;
	std::vector<Vec3d> current_model;
	bool width_lock = false;
	bool length_lock = false;
	bool finger_lock[5];
	//static std::vector<std::vector<float>> Bayes_mu;
	//static std::vector<std::vector<float>> Bayes_sig;


public:
	HTracker(Worker*worker, double FPS, std::string sequence_path, bool real_color, bool myoEnable, bool record_only, int user_name) : worker(worker), hub(hub), model("C:/Developer/FaceTracker-opencv2/model/face2.tracker") {
		setSingleShot(false);
		setInterval((1.0 / FPS)*1000.0);
		this->user_num = user_name;
		this->sequence_path = sequence_path;
		this->real_color = real_color;
		this->record_only = record_only;
		this->myoEnable = myoEnable;
		tw_settings->tw_add_ro(current_fps, "FPS", "group=Tracker");
		tw_settings->tw_add(initialization_enabled, "Detect ON?", "group=Tracker");
		tw_settings->tw_add(tracking_enabled, "ArtICP ON?", "group=Tracker");
		tw_settings->tw_add_ro(tracking_failed, "Tracking Lost?", "group=Tracker");

		//Bayes_mu = std::vector<std::vector<float>>();
		//Bayes_sig = std::vector<std::vector<float>>();

		if (showFaceTrack) {
			cvNamedWindow("Face Tracker", 1);
			worker->set_focus();
		}

		tri = FACETRACKER::IO::LoadTri("C:/Developer/FaceTracker-opencv2/model/face.tri");
		con = FACETRACKER::IO::LoadCon("C:/Developer/FaceTracker-opencv2/model/face.con");
		//model(ftFile);
		current_model = std::vector<Vec3d>(38);
		for (int i = 0; i < 5; i++) {
			finger_lock[i] = false;
		}
		load_words();
		corrcoefs[0] = 0;
		corrcoefs[1] = 0; corrcoefs[2] = 0;
	}
		
	HTracker(Worker*worker, myo::Hub* hub, double FPS, std::string sequence_path, bool real_color) : worker(worker), hub(hub) {
		setSingleShot(false);
		setInterval((1.0 / FPS)*1000.0);
		this->sequence_path = sequence_path;
		this->real_color = real_color;
		tw_settings->tw_add_ro(current_fps, "FPS", "group=Tracker");
		tw_settings->tw_add(initialization_enabled, "Detect ON?", "group=Tracker");
		tw_settings->tw_add(tracking_enabled, "ArtICP ON?", "group=Tracker");
		tw_settings->tw_add_ro(tracking_failed, "Tracking Lost?", "group=Tracker");

		//Bayes_mu = std::vector<std::vector<float>>();
		//Bayes_sig = std::vector<std::vector<float>>();
	}
	void toggle_sensor(bool on) {
		if (on = false) return;
		mode = SENSOR_ONLY;
		if (sensor->spin_wait_for_data(5) == false) LOG(INFO) << "no sensor data";
		solutions->reserve(30 * 60 * 5); // fps * sec * min
		start();
	}
	void toggle_tracking(bool on) {
		if (on == false) return;
		mode = LIVE;
		if (sensor->spin_wait_for_data(5) == false) LOG(INFO) << "no sensor data";
		solutions->reserve(30 * 60 * 5); // fps * sec * min
		start();
	}
	void toggle_benchmark(bool on) {
		if (on == false) return;

		setInterval((1.0 / 60)*1000.0);// 10
		worker->settings->termination_max_iters = 8;

		mode = BENCHMARK;
		start();
	}
	void toggle_playback(bool on) {
		if (on == false) return;
		//setInterval((1.0 / 44)*1000.0);
		setInterval(63);
		mode = PLAYBACK;
		start();
	}
private:
	void timerEvent(QTimerEvent*) {
		process_track();
		//if(!record_only)
			//process_face();
		//compute_initial_transformations();
	}

public:

	int speedup = 1;

	void process_face(){		
		cv::Mat im,gray;
		cv::Mat frame = worker->current_frame.color;
		float scale = 1;
		int64 t1, t0 = cvGetTickCount(); int fnum = 0;
		double fps = 0;
		char sss[256]; std::string text;
		//bool showFaceTrack = true;
		std::vector<int> wSize1(1); wSize1[0] = 7;
		std::vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;
		int nIter = 5; double clamp = 3, fTol = 0.01;
		bool failed = true;
		int fpd = -1;
		bool fcheck = false;

		//IplImage* I = cvQueryFrame(worker->camera); if (!I)continue; frame = I;
		if (scale == 1)im = frame;
		else cv::resize(frame, im, cv::Size(scale*frame.cols, scale*frame.rows));
		cv::flip(im, im, 1); 
		cv::cvtColor(im, gray, CV_BGR2GRAY);

		//track this image
		
		std::vector<int> wSize; 
		if (failed)
			wSize = wSize2; 
		else 
			wSize = wSize1;
		if (model.Track(gray, wSize, fpd, nIter, clamp, fTol, fcheck) == 0){
			int idx = model._clm.GetViewIdx(); failed = false;
			Draw(im, model._shape, con, tri, model._clm._visi[idx]);
		}
		else{
			if (showFaceTrack){
				cv::Mat R(im, cvRect(0, 0, 150, 50)); 
				R = cv::Scalar(0, 0, 255); 
			}
			model.FrameReset(); failed = true;
		}
		
		
		//draw framerate on display image 
		if (fnum >= 9){
			t1 = cvGetTickCount();
			fps = 10.0 / ((double(t1 - t0) / cvGetTickFrequency()) / 1e+6);
			t0 = t1; fnum = 0;
		}
		else fnum += 1;
		if (showFaceTrack) {
			sprintf(sss, "%d frames/sec", (int)round(fps)); text = sss;
			cv::putText(im, text, cv::Point(10, 20),
				CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
			//std::cout << text << " im.w: " << im.cols << " im.r: " << im.rows << std::endl;

			imshow("Face Tracker", im);
		}
		//int c = cvWaitKey(10);
		//if (c == 27)break; else if (char(c) == 'd')model.FrameReset();
	}

	void Draw(cv::Mat &image, cv::Mat &shape, cv::Mat &con, cv::Mat &tri, cv::Mat &visi)
	{
		int i, n = shape.rows / 2; cv::Point p1, p2; cv::Scalar c;

		//draw triangulation
		c = CV_RGB(0, 0, 0);
		for (i = 0; i < tri.rows; i++){
			if (visi.at<int>(tri.at<int>(i, 0), 0) == 0 ||
				visi.at<int>(tri.at<int>(i, 1), 0) == 0 ||
				visi.at<int>(tri.at<int>(i, 2), 0) == 0)continue;
			p1 = cv::Point(shape.at<double>(tri.at<int>(i, 0), 0),
				shape.at<double>(tri.at<int>(i, 0) + n, 0));
			p2 = cv::Point(shape.at<double>(tri.at<int>(i, 1), 0),
				shape.at<double>(tri.at<int>(i, 1) + n, 0));
			cv::line(image, p1, p2, c);
			p1 = cv::Point(shape.at<double>(tri.at<int>(i, 0), 0),
				shape.at<double>(tri.at<int>(i, 0) + n, 0));
			p2 = cv::Point(shape.at<double>(tri.at<int>(i, 2), 0),
				shape.at<double>(tri.at<int>(i, 2) + n, 0));
			cv::line(image, p1, p2, c);
			p1 = cv::Point(shape.at<double>(tri.at<int>(i, 2), 0),
				shape.at<double>(tri.at<int>(i, 2) + n, 0));
			p2 = cv::Point(shape.at<double>(tri.at<int>(i, 1), 0),
				shape.at<double>(tri.at<int>(i, 1) + n, 0));
			cv::line(image, p1, p2, c);
		}
		//draw connections
		c = CV_RGB(0, 0, 255);
		for (i = 0; i < con.cols; i++){
			if (visi.at<int>(con.at<int>(0, i), 0) == 0 ||
				visi.at<int>(con.at<int>(1, i), 0) == 0)continue;
			p1 = cv::Point(shape.at<double>(con.at<int>(0, i), 0),
				shape.at<double>(con.at<int>(0, i) + n, 0));
			p2 = cv::Point(shape.at<double>(con.at<int>(1, i), 0),
				shape.at<double>(con.at<int>(1, i) + n, 0));
			cv::line(image, p1, p2, c, 1);
		}
		//draw points
		for (i = 0; i < n; i++){
			if (visi.at<int>(i, 0) == 0)continue;
			p1 = cv::Point(shape.at<double>(i, 0), shape.at<double>(i + n, 0));
			c = CV_RGB(255, 0, 0); cv::circle(image, p1, 2, c);
		}return;
	}

	void process_track() {
		//compare(); return;
		//worker->updateGL(); return;
		//worker->E_pose.explore_pose_space(1); return;		

		static int frame_offset = 0;
		//static int current_frame = 0;
		//current_frame = 0;

		if (mode == PLAYBACK) {
			playback(); return;
		}

		static std::clock_t start = std::clock();

		float frame_start = std::clock() - start; if (verbose) cout << endl; //cout << current_frame << endl; cout << "start = " << frame_start << endl;

		if (mode == SENSOR_ONLY) {

			bool success = sensor->concurrent_fetch_streams(worker->current_frame, *worker->get_active_handfinder(), worker->get_active_model()->real_color);
			current_frame++;
			frame_offset = datastream->add_frame(worker->current_frame.color.data, worker->current_frame.depth.data, worker->get_active_model()->real_color.data);
			worker->current_frame.id = frame_offset;
			worker->sensor_depth_texture->load(worker->current_frame.depth.data, worker->current_frame.id);
			worker->offscreen_renderer.render_offscreen(true, false);
			worker->updateGL();
			return;
		}

		//TICTOC_BLOCK(fetching_time, "Sensor fetch")
		{
			if (mode == LIVE) {
				if(myoEnable)
					hub->run(1000 / 100);
				//bool success = sensor->fetch_streams(worker->current_frame);		
				//worker->handfinder->binary_classification(worker->current_frame.depth, worker->current_frame.color);

				bool success = sensor->concurrent_fetch_streams(worker->current_frame, *worker->get_active_handfinder(), worker->get_active_model()->real_color);
				
				/*if (current_frame == 1) {
					Vector3 translation = worker->trivial_detector->exec(worker->current_frame, worker->handfinder->sensor_silhouette);
					std::vector<float> thetas = worker->model->get_theta(); thetas[9] = 0; thetas[10] = 0;
					thetas[0] += translation[0]; thetas[1] += translation[1]; thetas[2] += translation[2];
					worker->model->move(thetas);
					worker->model->update_centers();
					worker->model->compute_outline();
					}*/
				current_frame++;
			}
			if (mode == BENCHMARK) {

				if (!manual_mode) {

					//cout << "pre-load_recorded_frame() track" << endl;
					if(!finished)
						load_recorded_frame(); // current_frame);

					current_frame += speedup;
					//current_frame += 4;
					worker->get_active_handfinder()->binary_classification(worker->current_frame.depth, worker->current_frame.color);

					//cv::imshow("sensor_silhouette", workerget_active_handfinder()->->sensor_silhouette); cv::waitKey(3);
					worker->get_active_handfinder()->num_sensor_points = 0; int count = 0;
					for (int row = 0; row < worker->get_active_handfinder()->sensor_silhouette.rows; ++row) {
						for (int col = 0; col < worker->get_active_handfinder()->sensor_silhouette.cols; ++col) {
							if (worker->get_active_handfinder()->sensor_silhouette.at<uchar>(row, col) != 255) continue;
							if (count % 2 == 0) {
								worker->get_active_handfinder()->sensor_indicator[worker->get_active_handfinder()->num_sensor_points] = row * worker->camera->width() + col;
								worker->get_active_handfinder()->num_sensor_points++;
							}
						}
					}

					if (current_frame == 1) {
						Vector3 translation = worker->trivial_detector->exec(worker->current_frame, worker->get_active_handfinder()->sensor_silhouette);
						std::vector<float> thetas = worker->get_active_model()->get_theta(); 
						thetas[0] += translation[0]; thetas[1] += translation[1]; thetas[2] += translation[2];
						//for (int i = 3; i < num_thetas; i++) {
						//	thetas[i] = 0;
						//}
						
						worker->get_active_model()->move(thetas);
						worker->get_active_model()->update_centers();
						worker->get_active_model()->compute_outline();
					}

					if (!worker->current_frame.depth.data || !worker->current_frame.color.data) return;
				}
				else
					//Manual mode
					if (worker->frame_advance) {
						worker->frame_advance = false;
						if(!finished)
							load_recorded_frame(); // current_frame);
						current_frame += speedup;
						//current_frame += 4;
						worker->get_active_handfinder()->binary_classification(worker->current_frame.depth, worker->current_frame.color);

						//cv::imshow("sensor_silhouette", workerget_active_handfinder()->->sensor_silhouette); cv::waitKey(3);
						worker->get_active_handfinder()->num_sensor_points = 0; int count = 0;
						for (int row = 0; row < worker->get_active_handfinder()->sensor_silhouette.rows; ++row) {
							for (int col = 0; col < worker->get_active_handfinder()->sensor_silhouette.cols; ++col) {
								if (worker->get_active_handfinder()->sensor_silhouette.at<uchar>(row, col) != 255) continue;
								if (count % 2 == 0) {
									worker->get_active_handfinder()->sensor_indicator[worker->get_active_handfinder()->num_sensor_points] = row * worker->camera->width() + col;
									worker->get_active_handfinder()->num_sensor_points++;
								}
							}
						}

						if (current_frame == 1) {
							Vector3 translation = worker->trivial_detector->exec(worker->current_frame, worker->get_active_handfinder()->sensor_silhouette);
							std::vector<float> thetas = worker->get_active_model()->get_theta(); thetas[9] = 0; thetas[10] = 0;
							thetas[0] += translation[0]; thetas[1] += translation[1]; thetas[2] += translation[2];
							cout << "Tracker line 373" << endl;
							worker->get_active_model()->move(thetas);
							worker->get_active_model()->update_centers();
							worker->get_active_model()->compute_outline();
						}

						if (!worker->current_frame.depth.data || !worker->current_frame.color.data) return;

						if (this->sensor->handfinder->_wristband_found && !record_only) {
							cout << "405 track_till_converge" << endl;
							tracking_failed = tracking_enabled ? worker->track_till_convergence() : true;
						}
					}
			}
		}

		//TICTOC_BLOCK(uploading_time, "GPU uploading")
		{
			frame_offset = datastream->add_frame(worker->current_frame.color.data, worker->current_frame.depth.data, worker->get_active_model()->real_color.data);
			
			
			if (mode == LIVE) {
				if (worker->spelling) {
					//Brandon added this for segmentation
					double r = correlate_frames(datastream->nth_depth_frame_back(0), datastream->nth_depth_frame_back(1));
					corrcoefs[current_frame % 3] = r;
					r = (corrcoefs[0] + corrcoefs[1] + corrcoefs[2]) / 3;
					//cout << (corrcoefs[0] + corrcoefs[1] + corrcoefs[2]) / 3 << endl;
					if (r > .975 && !classified) {
						int hs_class = worker->classify();
						if (hs_class == 0) {
							cout << endl;
							word_guess = "";
						}
						else {
							string letter = worker->class_names[hs_class];
							cout << letter.at(6);
							word_guess = word_guess + letter.at(6);
						}
						display_prompt(word_guess);
						//cout << "Most likely class: " << worker->class_names[hs_class] << std::endl; 
						classified = true;
					}
					else {
						if (r < .975 && classified)
							classified = false;

					}
				}
			}
			worker->current_frame.id = frame_offset;
			//worker->current_frame.id = ++frame_offset;		

			worker->sensor_depth_texture->load(worker->current_frame.depth.data, worker->current_frame.id);

		}
		float sensor = std::clock() - start; if (verbose) cout << "fetching = " << sensor - frame_start << endl;

		//TICTOC_BLOCK(tracking_time, "Tracking")
		if(!manual_mode){
				if (this->sensor->handfinder->_wristband_found && !record_only) {
					cout << "456 tracktoconverge" << endl;
					//tracking_failed = tracking_enabled ? worker->track_till_convergence() : true;
					if (!tracking_failed && worker->handedness == both_hands) {
						worker->swap_hands();
						worker->get_active_handfinder()->binary_classification(worker->current_frame.depth, worker->current_frame.color);
						worker->get_active_handfinder()->num_sensor_points = 0; int count = 0;
						for (int row = 0; row < worker->get_active_handfinder()->sensor_silhouette.rows; ++row) {
							for (int col = 0; col < worker->get_active_handfinder()->sensor_silhouette.cols; ++col) {
								if (worker->get_active_handfinder()->sensor_silhouette.at<uchar>(row, col) != 255) continue;
								if (count % 2 == 0) {
									worker->get_active_handfinder()->sensor_indicator[worker->get_active_handfinder()->num_sensor_points] = row * worker->camera->width() + col;
									worker->get_active_handfinder()->num_sensor_points++;
								}
							}
						}

						cout << "473 track_till_converge" << endl;
						tracking_failed = tracking_enabled ? worker->track_till_convergence() : true;
						worker->swap_hands();
					}
					/*
					std::vector<float> theta = worker->get_left_model()->get_theta();
					cout << "Left Hand: " << endl;
					for (int i = 0; i < num_thetas; i++) {
						cout << " " << theta[i];
					}
					cout << endl;
					theta = worker->get_right_model()->get_theta();
					cout << "Right Hand: " << endl;
					for (int i = 0; i < num_thetas; i++) {
						cout << " " << theta[i];
					}
					cout << endl;*/
				}
			
		}

		float tracking = std::clock() - start; if (verbose) cout << "tracking = " << tracking - sensor << endl;
		//TICTOC_BLOCK(reinit_time, "Reinitialization")
		{
			if (!record_only) {
				if (initialization_enabled && tracking_failed) {
					static QianDetection detection(worker);
					if (detection.can_reinitialize()) {
						//cout << "Going to run Qian Detector." << endl;
						detection.reinitialize();
					}
				}
			}
		}
		//TICTOC_BLOCK(rendering_time, "Rendering") 
		{
			worker->offscreen_renderer.render_offscreen(true, false);
			worker->updateGL();
			//if (mode == BENCHMARK && real_color) display_color_and_depth_input();		
			if (real_color) display_color_and_depth_input();
		}

		float rendering = std::clock() - start; if (verbose) cout << "rendering = " << rendering - tracking << endl;

		//TICTOC_BLOCK(saving_time, "Saving") 
		{
			solutions->resize(datastream->size());
			std::vector<float> theta = worker->get_active_model()->get_theta();
			solutions->set(frame_offset, theta); // worker->get_active_model()->get_theta());
			Eigen::Map<const Thetas> _theta(theta.data());
			worker->add_tracking_data(_theta);

			if (mode == BENCHMARK && !finished) {
				string tracking_error_filename;
				string solutions_filename;

				std::string sol_root = sequence_path + "sol/P" + std::string(2 - std::to_string(user_num).length(), '0') + std::to_string(user_num) + "/" + word;
				if (GetFileAttributesA(sol_root.c_str()) == INVALID_FILE_ATTRIBUTES)
					CreateDirectory(sol_root.c_str(), NULL);

				std::string set_path = sequence_path + "sol/P" +    std::string(2 - std::to_string(user_num).length(), '0') + std::to_string(user_num) + "/" + word + "/" + suffix;
				std::string img_path = sequence_path + "images/P" + std::string(2 - std::to_string(user_num).length(), '0') + std::to_string(user_num) + "/" + word + "/" + suffix;

				if (handshape) {
					set_path = sequence_path + "sol/P" + std::to_string(user_num) + "/" + word + "/Set" + std::to_string(handshape_set); // +"_" + word;
					img_path = sequence_path + "images/P" + std::to_string(user_num) + "/Set" + std::to_string(handshape_set) + "_" + word; // +"_" + suffix + ".png";
				}
				
				
				if (GetFileAttributesA(set_path.c_str()) == INVALID_FILE_ATTRIBUTES)
					CreateDirectory(set_path.c_str(), NULL);
				
				if (GetFileAttributesA(img_path.c_str()) == INVALID_FILE_ATTRIBUTES)
					CreateDirectory(img_path.c_str(), NULL);
				

				tracking_error_filename = set_path + "/hmodel_tracking_error_" + suffix + ".txt";
				solutions_filename = set_path + "/hmodel_solutions_" + suffix + ".txt";

				ofstream tracking_error_file(tracking_error_filename, ios_base::app);
				if (tracking_error_file.is_open()) {
					tracking_error_file << worker->tracking_error.pull_error << " " << worker->tracking_error.push_error << endl;
					tracking_error_file.close();
				}
				ofstream solutions_file(solutions_filename, ios_base::app);
				if (solutions_file.is_open()) {
					//cout << "   file is open" << endl;
					solutions_file << solutions->frames[frame_offset].transpose() << endl;
					solutions_file.close();
				}

				std::ostringstream stringstream;
				stringstream << std::setw(7) << std::setfill('0') << current_frame;
				std::string renderFile = "/render-" + stringstream.str() + "_" + suffix + ".bmp";

				if (!calibrate)
					if (handshape)
						saveRendering(img_path + "/Set" + std::to_string(handshape_set) + "_" + word + "_" + suffix + ".png");
					else
						saveRendering(img_path + renderFile);
				/*
				if (calibrate) {
					std::vector<float> thetas = worker->get_active_model()->get_theta();
					for (int i = 3; i < num_thetas; i++) {
						thetas[i] = 0;
					}
					worker->get_active_model()->move(thetas);
					worker->get_active_model()->update_centers();
					worker->track_till_convergence();
					
					float scale = 0;
					if (current_frame >= 2 && current_frame < 4) {
						scale = .1;
						//length_lock = true;
					}
					if (current_frame >= 4 && current_frame < 6)
						scale = .05;
					
					if ((current_frame >= 2 && current_frame < 6) ) {
						if (!width_lock)
							adjust_hand(false, scale);
						if (!length_lock)
							adjust_hand(true, scale);
						if (!finger_lock[0] || !finger_lock[1] || !finger_lock[2] || !finger_lock[3] || !finger_lock[4])
							adjust_finger_lengths(scale);
					}

				}*/

				if (handshape) {
					if (worker->stop_process) {
						current_frame = 0;
						worker->stop_process = false;
					}
				}

				/*static ofstream tracking_optimization_file(sequence_path + "hmodel_tracking_optimization.txt");
				if (tracking_optimization_file.is_open()) {
				for (size_t i = 0; i < worker->_settings.termination_max_iters; i++) {
				tracking_optimization_file << worker->tracking_error_optimization[i].pull_error << " " << worker->tracking_error_optimization[i].push_error << endl;
				}
				}*/
			}
			if (worker->save_rastorized_model) {
				cv::Mat rendered_model;
				worker->rastorizer.rastorize_model(rendered_model);

				float pull_error = online_performance_metrics.compute_rastorized_3D_metric(
					rendered_model, worker->current_frame.depth, worker->get_active_handfinder()->sensor_silhouette, worker->camera->inv_projection_matrix());
				float push_error = online_performance_metrics.compute_rastorized_2D_metric(
					rendered_model, worker->get_active_handfinder()->sensor_silhouette, worker->E_fitting.distance_transform.idxs_image());


				static ofstream rastorized_error_file(sequence_path + "hmodel_rastorized_error.txt");
				if (rastorized_error_file.is_open()) {
					rastorized_error_file << pull_error << " " << push_error << endl;
				}
				//worker->model->write_model("...", frame_offset);
			}
		}

		float end = std::clock() - start; //cout << "total = " << end - frame_start << endl; //cout << "end = " << end << endl;
		if (current_frame == 1) first_frame_lag = end - frame_start;
		else if (verbose) cout << "average = " << (std::clock() - (start + first_frame_lag)) / (current_frame - 1) << endl;
		
	}

	void playback() {

		//static int current_frame = 0;
		static int frame_offset = 0;
		static std::vector<float> theta_std = std::vector<float>(num_thetas, 0);

		cout << current_frame << endl;
		static std::clock_t start = std::clock();
		float frame_start = std::clock() - start; if (verbose) cout << endl;

		// TICTOC_BLOCK(tracking_time, "Loading data") 
		{
			load_recorded_frame(); // current_frame);
			worker->current_frame.id = frame_offset;
			worker->sensor_color_texture->load(worker->current_frame.color.data, worker->current_frame.id);
			worker->sensor_depth_texture->load(worker->current_frame.depth.data, worker->current_frame.id);

			if (!worker->current_frame.depth.data || !worker->current_frame.color.data) return;

			worker->get_active_handfinder()->binary_classification(worker->current_frame.depth, worker->current_frame.color);

		}

		// TICTOC_BLOCK(tracking_time, "Loading solutions") 
		{
			if (current_frame == 0) load_recorded_theta(sequence_path + "hmodel_solutions.txt");
			Thetas theta = solutions->frames[current_frame / speedup];
			for (size_t i = 0; i < num_thetas; i++) theta_std[i] = theta[i];
			worker->get_active_model()->move(theta_std);
			worker->get_active_model()->update_centers();
		}

		// TICTOC_BLOCK(tracking_time, "Rendering")
		{
			if (real_color) display_color_and_depth_input();
			worker->offscreen_renderer.render_offscreen(true, false);
			worker->updateGL();
			//glFinish();
		}

		// Write data for fitting
		{
			/*
			std::string filename;
			std::ostringstream stringstream;
			std::string fitting_path = "...";
			static const int num_pose_indices = 7;
			//static int poses_indices[num_pose_indices] = { 149, 232, 359, 425, 682 }; // andrii
			//static int poses_indices[num_pose_indices] = { 137, 174, 261, 374, 796 }; // thomas
			static int poses_indices[num_pose_indices] = { 202, 429, 728, 1067, 926, 1442, 1534 };//{ 58, 106, 166}; // pei-i
			for (size_t i = 0; i < num_pose_indices; i++) {
			if (current_frame == poses_indices[i]) {
			cout << "writing" << endl;
			stringstream << std::setw(1) << i + 1;
			// Write depth
			filename = fitting_path + stringstream.str() + "/depth.png";
			cv::imwrite(filename, worker->current_frame.depth);
			// Write color
			filename = fitting_path + stringstream.str() + "/color.png";
			cv::imwrite(filename, worker->current_frame.full_color);
			// Write mask
			filename = fitting_path + stringstream.str() + "/mask.png";
			cv::imwrite(filename, worker->handfinder->sensor_silhouette);
			// Write model
			worker->model->write_model(fitting_path + stringstream.str() + "/");
			}
			}*/
		}

		frame_offset++;
		current_frame += speedup;

		float end = std::clock() - start;
		if (current_frame == 1) first_frame_lag = end - frame_start;
		else if (verbose) cout << "average = " << (std::clock() - (start + first_frame_lag)) / (current_frame - 1) * speedup << endl;
	}

	void compare() {

		static int frame_offset = 0;
		static int current_frame = 0;
		cout << current_frame << endl;

		// Load depth
		std::ostringstream stringstream;
		stringstream << std::setw(7) << std::setfill('0') << current_frame;
		string filename;
		filename = sequence_path + "Depth/depth-" + stringstream.str() + ".png";
		worker->current_frame.depth = cv::imread(filename, cv::IMREAD_ANYDEPTH);
		filename = sequence_path + "Color/color-" + stringstream.str() + ".png";
		worker->current_frame.color = cv::imread(filename);

		worker->get_active_handfinder()->binary_classification(worker->current_frame.depth, worker->current_frame.color);	

		static cv::Mat sensor_silhouette_flipped;
		cv::flip(worker->get_active_handfinder()->sensor_silhouette, sensor_silhouette_flipped, 0 /*flip rows*/);

		DistanceTransform distance_transform;
		distance_transform.init(worker->camera->width(), worker->camera->height());
		distance_transform.exec(sensor_silhouette_flipped.data, 125);

		// Load model
		//cv::Mat rendered_model = cv::imread(sequence_path + "Tkach_2016/model-" +stringstream.str() + ".png", CV_LOAD_IMAGE_UNCHANGED);
		cv::Mat rendered_model = cv::imread(sequence_path + "Taylor_2016/" + std::to_string(current_frame) + "-Rendered depth---image.png", CV_LOAD_IMAGE_UNCHANGED);
		//cv::Mat rendered_model = cv::imread(sequence_path + "Sharp_2015/" + std::to_string(current_frame) + "-Rendered depth---image.png", CV_LOAD_IMAGE_UNCHANGED);
	
		// Crop wrist
		float wband_size = 10;
		float crop_radius = 150;
		float crop_radius_sq = crop_radius * crop_radius;
		Vector3 crop_center = worker->get_active_handfinder()->wristband_center() + worker->get_active_handfinder()->wristband_direction() * (crop_radius - wband_size);

		for (int row = 0; row < rendered_model.rows; ++row) {
			for (int col = 0; col < rendered_model.cols; ++col) {
				if (rendered_model.at<unsigned short>(row, col) == 5000) continue;
				Integer z = rendered_model.at<unsigned short>(row, col);
				Vector3 p_pixel = worker->camera->depth_to_world(col, row, z);
				if ((p_pixel - crop_center).squaredNorm() > crop_radius_sq) {					
					rendered_model.at<unsigned short>(row, col) = 5000;
				}
			}
		}

		// Show 
		cv::imshow("sensor_silhouette", worker->get_active_handfinder()->sensor_silhouette); cv::waitKey(1);
		cv::Mat normalized_rendered_model;
		cv::normalize(rendered_model, normalized_rendered_model, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		cv::imshow("rendered_model", normalized_rendered_model); cv::waitKey(1);

		// Compute metrics
		float pull_error = online_performance_metrics.compute_rastorized_3D_metric(
			rendered_model, worker->current_frame.depth, worker->get_active_handfinder()->sensor_silhouette, worker->camera->inv_projection_matrix());
		float push_error = online_performance_metrics.compute_rastorized_2D_metric(
			rendered_model, worker->get_active_handfinder()->sensor_silhouette, distance_transform.idxs_image());

		// Write metric
		static ofstream rastorized_error_file(sequence_path + "hmodel_rastorized_error.txt");
		cout << pull_error << " " << push_error << endl;
		if (rastorized_error_file.is_open()) {
			rastorized_error_file << pull_error << " " << push_error << endl;
		}

		// Write cropped model	
		cv::imwrite(sequence_path + "Taylor_Cropped/model-" + stringstream.str() + ".png", rendered_model);

		current_frame++;

	}

	std::vector<float> load_recorded_theta(std::string solution_path) {
		cout << "loading solutions" << endl;
		std::vector<float> theta = std::vector<float>(num_thetas, 0);
		std::ifstream in(solution_path);
		if (!in.is_open()) {
			cout << "cannot open solution file" << endl;
			exit(0);
		}

		///--- Allocate
		int max_num_frames = 5000;
		solutions->frames.resize(max_num_frames);

		///--- Read in the matrix
		int row = 0;
		for (std::string line; std::getline(in, line) && row < max_num_frames; row++) {
			stringstream str(line);
			for (int col = 0; col < num_thetas; ++col) {
				std::string elem;
				str >> elem;
				solutions->frames[row](col) = std::stof(elem);
			}
		}
		in.close();
		return theta;
	}

	void load_recorded_frame() { //size_t current_frame) {

		std::ostringstream stringstream;
		stringstream << std::setw(7) << std::setfill('0') << current_frame;

		if (calibrate) {
			length_lock = false;
			width_lock = false;
			for (int i = 0; i < 5; i++) {
				finger_lock[i] = false;
			}
			std::string set_path;
			if (user_num < 10)
				set_path = sequence_path + "P0" + std::to_string(user_num) + "/Calibrate/Open5";
			else
				set_path = sequence_path + "P" + std::to_string(user_num) + "/Calibrate/Open5";
			std::string checkFile = set_path + "depth-" + stringstream.str() + ".png";

			std::ostringstream stringstream2;
			
			ifstream ftest(checkFile.c_str());
			if (ftest.fail()) {
				cout << "Failed to load: " << checkFile << endl;
				
				worker->get_active_model()->write_model("C:/Projects/MyoFaceVersion/hmodel/data/"); //SHOULD FIX THIS

				worker->current_frame.depth = cv::imread(set_path + "depth-0000005.png", cv::IMREAD_ANYDEPTH);
				worker->current_frame.color = cv::imread(set_path + "color-0000005.png");
				finished = true;
			}
			else {
				ftest.close();
				cout << "Loading: " << set_path + "depth-" + stringstream.str() + ".png" << endl;
				worker->current_frame.depth = cv::imread(set_path + "depth-" + stringstream.str() + ".png", cv::IMREAD_ANYDEPTH);
				worker->current_frame.color = cv::imread(set_path + "color-" + stringstream.str() + ".png");
				
				//worker->get_active_model()->real_color = cv::imread(set_path + "full_color-" + stringstream.str() + ".png");
			}
		}
		else if(handshape){
			std::string set_path;

			if (current_frame == 0) {
				if (word_i == words.size()) {
					handshape_set++;
					word_i = 0;
				}

				word = words[word_i++];
				cout << "Loaded next word: " << word << endl;
				std::vector<float> thetas = worker->get_active_model()->get_theta();
				for (int i = 3; i < num_thetas; i++) {
					thetas[i] = 0;
				}
				cout << "Tracker line 809" << endl;
				worker->get_active_model()->move(thetas);
				worker->get_active_model()->update_centers();
			}

			set_path = sequence_path + "P" + std::string(2 - std::to_string(user_num).length(), '0') + std::to_string(user_num) + "/Set" + std::to_string(handshape_set) + "/" + word + "/";

			std::string checkFile = set_path + "depth-" + stringstream.str() + ".png";
			//cout << "Trying to load: " << checkFile << endl;
			ifstream ftest(checkFile.c_str());
			if (ftest.fail()) {
				cout << "Failed to load: " << checkFile << endl;
				current_frame = 0;
				if (handshape_set <= handshape_set_max)
					load_recorded_frame();
				else
				{
					finished = true;
					//if (user_num <= 28) {
					//	next_participant();
					//}else
					//	finished = true;
				}
			}
			else {
				ftest.close();
				worker->current_frame.depth = cv::imread(set_path + "depth-" + stringstream.str() + ".png", cv::IMREAD_ANYDEPTH);
				worker->current_frame.color = cv::imread(set_path + "color-" + stringstream.str() + ".png");
				worker->get_active_model()->real_color = cv::imread(set_path + "full_color-" + stringstream.str() + ".png");
			}

		}else {

			std::string set_path;

			if (current_frame == 0) {
				if (word_i == words.size()) {
					user_num++;
					word_i = 0;
					cout << "!!!!! End of List!!!!" << endl;
					while (1) {}
					cout << "Moving to user: " << user_num << "   (Hardcoded warning!!)" << endl;

					//
					worker->get_active_model()->user_name = user_num;
					worker->get_active_model()->load_model_from_file();
					cout << "finished new loading" << endl;
					std::vector<float> thetas = worker->get_active_model()->get_theta();
					for (int i = 3; i < num_thetas; i++) {
						thetas[i] = 0;
					}
					cout << "Tracker line 861" << endl;
					worker->get_active_model()->move(thetas);
					worker->get_active_model()->update_centers();

					worker->handfinder_1 = new HandFinder(worker->camera, worker->handedness, "C:/Projects/MyoFaceVersion/hmodel/data/", user_num);
					worker->handfinder = worker->handfinder_1;


					//worker->get_active_model()->init(user_num, "C:/Projects/MyoFaceVersion/hmodel/data/", right_hand); //TODO : hard coded warning!!!
				}
				word = words[word_i++];

				/*int static ver_num = 1;
				set_path = sequence_path + "P" + std::string(2 - std::to_string(user_num).length(), '0') + std::to_string(user_num) + "/" + word + "_" + std::to_string(ver_num) + "_";
				std::string checkFile = set_path + "depth-" + stringstream.str() + ".png";
				ifstream ftest(checkFile.c_str());
				if (ftest.fail()) {
					ver_num++;
					set_path = sequence_path + "P" + std::string(2 - std::to_string(user_num).length(), '0') + std::to_string(user_num) + "/" + word + "_" + std::to_string(ver_num) + "_";
					std::string checkFile = set_path + "depth-" + stringstream.str() + ".png";
				}*/

				cout << "Loaded next word: " << word << endl;
				std::vector<float> thetas = worker->get_active_model()->get_theta();
				for (int i = 3; i < num_thetas; i++) {
					thetas[i] = 0;
				}
				//worker->get_active_model()->move(thetas);
				//worker->get_active_model()->update_centers();
			}

			int ver_num = 1;
			set_path  = sequence_path + "P" + std::string(2 - std::to_string(user_num).length(), '0') + std::to_string(user_num) + "/" + word + "_" + std::to_string(ver_num) + "_";
			//if()
				

			std::string checkFile = set_path + "depth-" + stringstream.str() + ".png";
			//cout << "Trying to load: " << checkFile << endl;
			ifstream ftest(checkFile.c_str());
			if (ftest.fail()) {
				cout << "Failed to load: " << checkFile << endl;
				current_frame = 0;
				load_recorded_frame();
			}
			else {
				ftest.close();
				worker->current_frame.depth = cv::imread(set_path + "depth-" + stringstream.str() + ".png", cv::IMREAD_ANYDEPTH);
				worker->current_frame.color = cv::imread(set_path + "color-" + stringstream.str() + ".png");
				worker->get_active_model()->real_color = cv::imread(set_path + "full_color-" + stringstream.str() + ".png");
			}
		}
	}

	void display_color_and_depth_input() {
		cv::Mat normalized_depth = worker->current_frame.depth.clone();
		cv::inRange(normalized_depth, worker->camera->zNear(), worker->camera->zFar(), normalized_depth);
		cv::normalize(normalized_depth, normalized_depth, 127, 255, cv::NORM_MINMAX, CV_8UC1);
		cv::resize(normalized_depth, normalized_depth, cv::Size(2 * normalized_depth.cols, 2 * normalized_depth.rows), cv::INTER_CUBIC);//resize image
		cv::moveWindow("DEPTH", 592, 855); cv::imshow("DEPTH", normalized_depth);

		cv::namedWindow("RGB");	cv::moveWindow("RGB", 592, 375); cv::imshow("RGB", worker->get_active_model()->real_color);
	}
	/*
	void read_bayes_vectors(std::string data_path, std::string name, std::vector<std::vector<float>> & input) {
		FILE *fp = fopen((data_path + name + ".txt").c_str(), "r");
		int N = 41;

		//fscanf(fp, "%d", &N);
		for (int i = 0; i < N; ++i) {
			std::vector<float> theta = std::vector<float>();
			for (int j = 0; j < 20; ++j){
				float a;
				fscanf(fp, "%f", &a);
				theta.push_back(a);
			}
			input.push_back(theta);
			//std::cout << "one value is: " << theta[3] << std::endl;
		}
		fclose(fp);

	}
	
	void calc_max_likelihood(std::vector<float> theta){
		int N = 41;
		int class_max = 0;
		float max_likelihood = -INFINITE;
		float likelihood = 0;
		for (int i = 0; i < N; ++i){
			for (int j = 0; j < 20; ++j){
				likelihood += -.5*log(2 * M_PI * Bayes_sig[i][j]) - (theta[j] - Bayes_mu[i][j])*(theta[j] - Bayes_mu[i][j]) / (2 * Bayes_sig[i][j]);
			}
			if (likelihood > max_likelihood){
				max_likelihood = likelihood;
				class_max = i;
			}
		}
		std::cout << "Most likely Class: " << class_max << std::endl;

	}*/

	void load_words() {

		words = std::vector<std::string>();
		if (handshape) {
			cout << "Loading ABCs" << endl;
			ifstream fp("C:/Projects/MyoFaceVersion/hmodel/data/prompts/abcList.txt"); // non_abcList.txt");
			std::string word;
			while (getline(fp, word)) {
				words.push_back(word);
			}
			fp.close();
			word_i = 0;
		}
		else {
			ifstream fp("C:/Projects/MyoFaceVersion/hmodel/data/prompts/randomizedLists.txt"); // randomizedLists.txt");
			std::string word;
			while (getline(fp, word)) {
				words.push_back(word);
				//cout << "loaded: " << word << endl;
			}
			fp.close();
			word_i = 0;
		}
		//word = words[word_i++];
	}


	void saveRendering(std::string filename) {

		int width = 640 *2;
		int height = 480 *2;
		unsigned char* imageData = (unsigned char *)malloc((int)(width*height*(3)));
		glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, imageData);

		FILE *filePtr = fopen(filename.c_str(), "wb");
		if (!filePtr)
			return;

		BITMAPFILEHEADER bitmapFileHeader;
		BITMAPINFOHEADER bitmapInfoHeader;

		bitmapFileHeader.bfType = 0x4D42; //"BM"
		bitmapFileHeader.bfSize = width*height * 3;
		bitmapFileHeader.bfReserved1 = 0;
		bitmapFileHeader.bfReserved2 = 0;
		bitmapFileHeader.bfOffBits =
			sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

		bitmapInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
		bitmapInfoHeader.biWidth = width;
		bitmapInfoHeader.biHeight = height;
		bitmapInfoHeader.biPlanes = 1;
		bitmapInfoHeader.biBitCount = 24;
		bitmapInfoHeader.biCompression = BI_RGB;
		bitmapInfoHeader.biSizeImage = 0;
		bitmapInfoHeader.biXPelsPerMeter = 0; // ?
		bitmapInfoHeader.biYPelsPerMeter = 0; // ?
		bitmapInfoHeader.biClrUsed = 0;
		bitmapInfoHeader.biClrImportant = 0;

		fwrite(&bitmapFileHeader, sizeof(BITMAPFILEHEADER), 1, filePtr);
		fwrite(&bitmapInfoHeader, sizeof(BITMAPINFOHEADER), 1, filePtr);
		fwrite(imageData, width*height * 3, 1, filePtr);
		fclose(filePtr);

		free(imageData);
	}




	float average_error() {
		float averagePush = 0;
		float averagePull = 0;
		for (int i = 0; i < 10; ++i) {

			cout << "1097 average error" << endl;
			worker->track_till_convergence();
			averagePush += worker->tracking_error.push_error;
			averagePull += worker->tracking_error.pull_error;
		}
		//cout << "   push error: " << averagePush << " pull error: " << averagePull << std::endl;
		return averagePull + averagePush;
	}

	void reset_hand_model() {
		for (int i = 0; i < num_phalanges; i++) {
			worker->get_active_model()->phalanges[i].init_local(0, 3) = (float)current_model[worker->get_active_model()->phalanges[i].center_id][0];
			worker->get_active_model()->phalanges[i].init_local(1, 3) = (float)current_model[worker->get_active_model()->phalanges[i].center_id][1];
			worker->get_active_model()->phalanges[i].init_local(2, 3) = (float)current_model[worker->get_active_model()->phalanges[i].center_id][2];
			for (int j = 0; j < worker->get_active_model()->phalanges[i].attachments.size(); j++) {
				worker->get_active_model()->phalanges[i].offsets[j] = current_model[worker->get_active_model()->phalanges[i].attachments[j]];
			}
		}
	}

	void save_current_hand_model() {
		for (int i = 0; i < num_phalanges; i++) {
			current_model[worker->get_active_model()->phalanges[i].center_id][0] = worker->get_active_model()->phalanges[i].init_local(0, 3);
			current_model[worker->get_active_model()->phalanges[i].center_id][1] = worker->get_active_model()->phalanges[i].init_local(1, 3);
			current_model[worker->get_active_model()->phalanges[i].center_id][2] = worker->get_active_model()->phalanges[i].init_local(2, 3);
			for (int j = 0; j < worker->get_active_model()->phalanges[i].attachments.size(); j++) {
				current_model[worker->get_active_model()->phalanges[i].attachments[j]] = worker->get_active_model()->phalanges[i].offsets[j];
			}
		}
	}

	void adjust_finger_lengths(float scale) {
		cout << "Checking Finger Lengths" << endl;
		adjust_finger_length(0, scale);
		adjust_finger_length(1, scale);
		adjust_finger_length(2, scale);
		adjust_finger_length(3, scale);
		adjust_finger_length(4, scale);
	}

	void adjust_finger_length(int finger, float scale) {
		//Force straight joints
		if (current_frame < 4) {
			std::vector<float> thetas = worker->get_active_model()->get_theta();
			thetas[11] = 0; thetas[12] = 0; //thumb straight
			thetas[15] = 0; thetas[16] = 0; //index
			thetas[19] = 0; thetas[20] = 0; //middle
			thetas[23] = 0; thetas[24] = 0; //ring
			thetas[27] = 0; thetas[28] = 0; //pinky
			worker->get_active_model()->move(thetas);
			worker->get_active_model()->update_centers();
		}

		int iterations = 0;
		switch (finger) {
		case 0: {
			cout << " Checking thumb : ";
		}break;
		case 1: {
			cout << " Checking pinky : ";
		}break;
		case 2: {
			cout << " Checking ring  : ";
		}break;
		case 3: {
			cout << " Checking middle: ";
		}break;
		case 4: {
			cout << " Checking index : ";
		}break;

		}
		while (!finger_lock[finger] && iterations < 10) {
			save_current_hand_model();
			float init_error = average_error();
			worker->get_active_model()->adjust_finger(finger, 1 + scale);
			worker->get_active_model()->update_centers();
			float longer_error = average_error();
			reset_hand_model();
			worker->get_active_model()->adjust_finger(finger, 1 - scale);
			worker->get_active_model()->update_centers();
			float shorter_error = average_error();
			//cout << "init_err: " << init_error << " long_error: " << longer_error << " short_error: " << shorter_error << endl;
			if (shorter_error < longer_error) {
				if (shorter_error < init_error) {
					//Already shorter. Do nothing
					cout << "   scaling by: " << 1 - scale << " init_err: " << init_error << " long_error: " << longer_error << " short_error: " << shorter_error << endl;
				}
				else {
					//Init < shorter. reset. lock
					reset_hand_model();
					cout << "   keep the current length." << endl;
					finger_lock[finger] = true;
				}
			}
			else {
				//Shorter is worse. reset.
				reset_hand_model();
				if (longer_error < init_error) {
					//longer is better. make longer				
					worker->get_active_model()->adjust_finger(finger, 1 + scale);
					cout << "   scaling by: " << 1 + scale << " init_err: " << init_error << " long_error: " << longer_error << " short_error: " << shorter_error << endl;
				}
				else {
					//init is best. lock
					cout << "   Keep the current length." << endl;
					finger_lock[finger] = true;
				}
			}
			iterations++;
			worker->get_active_model()->update_centers();
		}
	}

	void adjust_hand(bool length, float scale) {
		if(length)
			cout << "Checking Hand Length" << endl;
		else
			cout << "Checking Hand Width" << endl;
		int iterations = 0;
		bool lock;
		if (length)
			lock = length_lock;
		else
			lock = width_lock;
		while (!lock && iterations < 10) {
			save_current_hand_model();
			float init_error = average_error();
			if (length)
				worker->get_active_model()->adjust_hand_length(1 + scale);
			else
				worker->get_active_model()->adjust_hand_width(1 + scale);
			worker->get_active_model()->update_centers();
			float bigger_error = average_error();
			reset_hand_model();
			if (length)
				worker->get_active_model()->adjust_hand_length(1 - scale);
			else
				worker->get_active_model()->adjust_hand_width(1 - scale);
			worker->get_active_model()->update_centers();
			float smaller_error = average_error();
			//if(length)
			//	cout << "init_err: " << init_error << " long_err: " << bigger_error << " short_err: " << smaller_error << endl;
			//else
			//	cout << "init_err: " << init_error << " wide_err: " << bigger_error << " thin_err: " << smaller_error << endl;
			if (smaller_error < bigger_error) {
				if (smaller_error < init_error) {
					cout << "   scaling by " << 1 - scale << " init_err: " << init_error << " big_err: " << bigger_error << " small_err: " << smaller_error << endl;
				}
				else {
					cout << "   keep current scale." << endl;
					reset_hand_model();
					lock = true;
					if (length)
						length_lock = true;
					else
						width_lock = true;
				}
			}
			else {
				reset_hand_model();
				if (bigger_error < init_error) {
					cout << "   scaling by " << 1 + scale << " init_err: " << init_error << " big_err: " << bigger_error << " small_err: " << smaller_error << endl;
					if (length)
						worker->get_active_model()->adjust_hand_length(1 + scale);
					else
						worker->get_active_model()->adjust_hand_width(1 + scale);
				}
				else {
					cout << "   keep current scale." << endl;
					lock = true;
					if (length)
						length_lock = true;
					else
						width_lock = true;
				}
			}
			iterations++;
			worker->get_active_model()->update_centers();
		}
	}

	/*
	void next_participant() {
		int new_user_name = user_num + 1;
		cout << "Trying to start next worker. Participant: " << new_user_name;
		Worker new_worker(worker->camera, worker->test, worker->benchmark, worker->save_rastorized_model, new_user_name, worker->data_path, left_hand, &sensor);

		{
			new_worker.settings->termination_max_iters = 8;

			new_worker.E_fitting.settings->fit2D_enable = true;
			new_worker.E_fitting.settings->fit2D_weight = 0.7;

			new_worker.E_fitting.settings->fit3D_enable = true;

			new_worker.E_limits.jointlimits_enable = true;

			new_worker.E_pose._settings.enable_split_pca = true;
			new_worker.E_pose._settings.weight_proj = 4 * 10e2;

			new_worker.E_collision._settings.collision_enable = true;
			new_worker.E_collision._settings.collision_weight = 1e3;

			new_worker.E_temporal._settings.temporal_coherence1_enable = true;
			new_worker.E_temporal._settings.temporal_coherence2_enable = true;
			new_worker.E_temporal._settings.temporal_coherence1_weight = 0.05;
			new_worker.E_temporal._settings.temporal_coherence2_weight = 0.05;

			new_worker.E_damping._settings.abduction_damping = 1500000;
			new_worker._settings.termination_max_rigid_iters = 1;
		}
		//GLWidget* glwidget = (GLWidget*)worker->glarea;
		//glwidget->worker = &new_worker;
		//glwidget->kinect_renderer.setup(worker->sensor_color_texture->texid(), worker->sensor_depth_texture->texid());
		//worker->cleanup_graphic_resources();
		//new_worker.init_graphic_resources();
		//this->worker = &new_worker;

		//user_num = new_user_name;
		//handshape_set = 1;

	}
	*/

	void display_prompt(std::string prompt) {

		cv::Mat word = cv::Mat::ones(100, 500, CV_8UC3);
		word = cv::Scalar(255, 255, 255);
		cv::putText(word, prompt, cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 2, 0);
		cv::imshow("show_prompt", word); // prompt);
		cv::waitKey(1);
	}

	double correlate_frames(cv::Mat* img1, cv::Mat* img2) {
		if (img1 == NULL || img2 == NULL) {
			cout << "Null pased to correlate" << endl;
			return 0.0;
		}
		int height = datastream->height();
		int width = datastream->width();
		//cv::Mat img1 = cv::Mat(height, width, CV_16UC1, (void*)data1).clone();
		//cv::Mat img2 = cv::Mat(height, width, CV_16UC1, (void*)data2).clone();
		double x = 0;
		double x2 = 0;
		double y = 0;
		double y2 = 0;
		double xy = 0;
		int n = datastream->height() * datastream->width();

		for (int i = 0; i < datastream->height(); i++) {
			for (int j = 0; j < datastream->width(); j++) {		
				if (worker->get_active_handfinder()->sensor_silhouette.at<uchar>(i, j) == 255) {
					x += static_cast<double>(img1->at<unsigned short>(i, j));
					x2 += static_cast<double>(img1->at<unsigned short>(i, j))*static_cast<double>(img1->at<unsigned short>(i, j));
					y += static_cast<double>(img2->at<unsigned short>(i, j));
					y2 += static_cast<double>(img2->at<unsigned short>(i, j))*static_cast<double>(img2->at<unsigned short>(i, j));
					xy += static_cast<double>(img1->at<unsigned short>(i, j))*static_cast<double>(img2->at<unsigned short>(i, j));
				}
			}
		}
		
		double r = (n*xy - x*y) / (sqrt((n - 1) * x2 - x*x) * sqrt((n - 1)*y2 - y*y));
		//cout << r << " " << x << " " << x2 << " " << y << " " << y2 << " " << xy << " " << endl;
		return r;
	}
};


