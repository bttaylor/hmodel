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


class Tracker : public QTimer {
public:
	enum Mode { LIVE, BENCHMARK, PLAYBACK } mode = LIVE;
	Sensor* sensor;
	DataStream* datastream;
	SolutionStream* solutions;
	Worker*const worker = NULL;
	myo::Hub*const hub = NULL;

	OnlinePeformanceMetrics online_performance_metrics;

	std::string data_path;
	bool real_color;

public:
	float current_fps = 0;
	int first_frame_lag = 0;

	bool tracking_failed = true;
	bool initialization_enabled = true;
	bool tracking_enabled = true;
	bool verbose = false;

	//Brandon

	FACETRACKER::Tracker model;
	cv::Mat tri;
	cv::Mat con;
	//static std::vector<std::vector<float>> Bayes_mu;
	//static std::vector<std::vector<float>> Bayes_sig;


public:
	Tracker(Worker*worker, double FPS, std::string data_path, bool real_color) : worker(worker), hub(hub), model("C:/Developer/FaceTracker-opencv2/model/face2.tracker") {
		setSingleShot(false);
		setInterval((1.0 / FPS)*1000.0);
		this->data_path = data_path;
		this->real_color = real_color;
		tw_settings->tw_add_ro(current_fps, "FPS", "group=Tracker");
		tw_settings->tw_add(initialization_enabled, "Detect ON?", "group=Tracker");
		tw_settings->tw_add(tracking_enabled, "ArtICP ON?", "group=Tracker");
		tw_settings->tw_add_ro(tracking_failed, "Tracking Lost?", "group=Tracker");

		//Bayes_mu = std::vector<std::vector<float>>();
		//Bayes_sig = std::vector<std::vector<float>>();
		cvNamedWindow("Face Tracker", 1);

		tri = FACETRACKER::IO::LoadTri("C:/Developer/FaceTracker-opencv2/model/face.tri");
		con = FACETRACKER::IO::LoadCon("C:/Developer/FaceTracker-opencv2/model/face.con");
		//model(ftFile);
	}
		
	Tracker(Worker*worker, myo::Hub* hub, double FPS, std::string data_path, bool real_color) : worker(worker), hub(hub) {
		setSingleShot(false);
		setInterval((1.0 / FPS)*1000.0);
		this->data_path = data_path;
		this->real_color = real_color;
		tw_settings->tw_add_ro(current_fps, "FPS", "group=Tracker");
		tw_settings->tw_add(initialization_enabled, "Detect ON?", "group=Tracker");
		tw_settings->tw_add(tracking_enabled, "ArtICP ON?", "group=Tracker");
		tw_settings->tw_add_ro(tracking_failed, "Tracking Lost?", "group=Tracker");

		//Bayes_mu = std::vector<std::vector<float>>();
		//Bayes_sig = std::vector<std::vector<float>>();
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
		process_face();
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
		bool show = true;
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
			if (show){
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
		//if (show){
			sprintf(sss, "%d frames/sec", (int)round(fps)); text = sss;
			cv::putText(im, text, cv::Point(10, 20),
				CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
			//std::cout << text << " im.w: " << im.cols << " im.r: " << im.rows << std::endl;
		//}
		//show image and check for user input
		imshow("Face Tracker", im);
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
		static int current_frame = 0;

		if (mode == PLAYBACK) {
			playback(); return;
		}

		static std::clock_t start = std::clock();

		float frame_start = std::clock() - start; if (verbose) cout << endl; //cout << current_frame << endl; cout << "start = " << frame_start << endl;

		//TICTOC_BLOCK(fetching_time, "Sensor fetch")
		{
			if (mode == LIVE) {

				//hub->run(1000 / 100);
				//bool success = sensor->fetch_streams(worker->current_frame);		
				//worker->handfinder->binary_classification(worker->current_frame.depth, worker->current_frame.color);

				bool success = sensor->concurrent_fetch_streams(worker->current_frame, *worker->handfinder, worker->model->real_color);

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
				load_recorded_frame(current_frame);
				current_frame += speedup;
				//current_frame += 4;
				worker->handfinder->binary_classification(worker->current_frame.depth, worker->current_frame.color);

				//cv::imshow("sensor_silhouette", worker->handfinder->sensor_silhouette); cv::waitKey(3);
				worker->handfinder->num_sensor_points = 0; int count = 0;
				for (int row = 0; row < worker->handfinder->sensor_silhouette.rows; ++row) {
					for (int col = 0; col < worker->handfinder->sensor_silhouette.cols; ++col) {
						if (worker->handfinder->sensor_silhouette.at<uchar>(row, col) != 255) continue;
						if (count % 2 == 0) {
							worker->handfinder->sensor_indicator[worker->handfinder->num_sensor_points] = row * worker->camera->width() + col;
							worker->handfinder->num_sensor_points++;
						}
					}
				}

				if (current_frame == 1) {
					Vector3 translation = worker->trivial_detector->exec(worker->current_frame, worker->handfinder->sensor_silhouette);
					std::vector<float> thetas = worker->model->get_theta(); thetas[9] = 0; thetas[10] = 0;
					thetas[0] += translation[0]; thetas[1] += translation[1]; thetas[2] += translation[2];
					worker->model->move(thetas);
					worker->model->update_centers();
					worker->model->compute_outline();
				}

				if (!worker->current_frame.depth.data || !worker->current_frame.color.data) return;
			}
		}

		//TICTOC_BLOCK(uploading_time, "GPU uploading")
		{
			frame_offset = datastream->add_frame(worker->current_frame.color.data, worker->current_frame.depth.data, worker->model->real_color.data);
			worker->current_frame.id = frame_offset;
			//worker->current_frame.id = ++frame_offset;		

			worker->sensor_depth_texture->load(worker->current_frame.depth.data, worker->current_frame.id);

		}
		float sensor = std::clock() - start; if (verbose) cout << "fetching = " << sensor - frame_start << endl;

		//TICTOC_BLOCK(tracking_time, "Tracking")
		{
			tracking_failed = tracking_enabled ? worker->track_till_convergence() : true;
		}

		float tracking = std::clock() - start; if (verbose) cout << "tracking = " << tracking - sensor << endl;
		//TICTOC_BLOCK(reinit_time, "Reinitialization")
		{
			if (initialization_enabled && tracking_failed) {
				static QianDetection detection(worker);
				if (detection.can_reinitialize()) {
					detection.reinitialize();
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
			solutions->set(frame_offset, worker->model->get_theta());

			if (mode == BENCHMARK) {
				string tracking_error_filename;
				string solutions_filename;

				tracking_error_filename = data_path + "hmodel_tracking_error.txt";
				solutions_filename = data_path + "hmodel_solutions.txt";

				static ofstream tracking_error_file(tracking_error_filename);
				if (tracking_error_file.is_open()) {
					tracking_error_file << worker->tracking_error.pull_error << " " << worker->tracking_error.push_error << endl;
				}
				static ofstream solutions_file(solutions_filename);
				if (solutions_file.is_open()) {
					solutions_file << solutions->frames[frame_offset].transpose() << endl;
				}
				/*static ofstream tracking_optimization_file(data_path + "hmodel_tracking_optimization.txt");
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
					rendered_model, worker->current_frame.depth, worker->handfinder->sensor_silhouette, worker->camera->inv_projection_matrix());
				float push_error = online_performance_metrics.compute_rastorized_2D_metric(
					rendered_model, worker->handfinder->sensor_silhouette, worker->E_fitting.distance_transform.idxs_image());


				static ofstream rastorized_error_file(data_path + "hmodel_rastorized_error.txt");
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

		static int current_frame = 0;
		static int frame_offset = 0;
		static std::vector<float> theta_std = std::vector<float>(num_thetas, 0);

		cout << current_frame << endl;
		static std::clock_t start = std::clock();
		float frame_start = std::clock() - start; if (verbose) cout << endl;

		// TICTOC_BLOCK(tracking_time, "Loading data") 
		{
			load_recorded_frame(current_frame);
			worker->current_frame.id = frame_offset;
			worker->sensor_color_texture->load(worker->current_frame.color.data, worker->current_frame.id);
			worker->sensor_depth_texture->load(worker->current_frame.depth.data, worker->current_frame.id);

			if (!worker->current_frame.depth.data || !worker->current_frame.color.data) return;

			worker->handfinder->binary_classification(worker->current_frame.depth, worker->current_frame.color);

		}

		// TICTOC_BLOCK(tracking_time, "Loading solutions") 
		{
			if (current_frame == 0) load_recorded_theta(data_path + "hmodel_solutions.txt");
			Thetas theta = solutions->frames[current_frame / speedup];
			for (size_t i = 0; i < num_thetas; i++) theta_std[i] = theta[i];
			worker->model->move(theta_std);
			worker->model->update_centers();
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
		filename = data_path + "Depth/depth-" + stringstream.str() + ".png";
		worker->current_frame.depth = cv::imread(filename, cv::IMREAD_ANYDEPTH);
		filename = data_path + "Color/color-" + stringstream.str() + ".png";
		worker->current_frame.color = cv::imread(filename);

		worker->handfinder->binary_classification(worker->current_frame.depth, worker->current_frame.color);	

		static cv::Mat sensor_silhouette_flipped;
		cv::flip(worker->handfinder->sensor_silhouette, sensor_silhouette_flipped, 0 /*flip rows*/);

		DistanceTransform distance_transform;
		distance_transform.init(worker->camera->width(), worker->camera->height());
		distance_transform.exec(sensor_silhouette_flipped.data, 125);

		// Load model
		//cv::Mat rendered_model = cv::imread(data_path + "Tkach_2016/model-" +stringstream.str() + ".png", CV_LOAD_IMAGE_UNCHANGED);
		cv::Mat rendered_model = cv::imread(data_path + "Taylor_2016/" + std::to_string(current_frame) + "-Rendered depth---image.png", CV_LOAD_IMAGE_UNCHANGED);
		//cv::Mat rendered_model = cv::imread(data_path + "Sharp_2015/" + std::to_string(current_frame) + "-Rendered depth---image.png", CV_LOAD_IMAGE_UNCHANGED);
	
		// Crop wrist
		float wband_size = 10;
		float crop_radius = 150;
		float crop_radius_sq = crop_radius * crop_radius;
		Vector3 crop_center = worker->handfinder->wristband_center() + worker->handfinder->wristband_direction() * (crop_radius - wband_size);

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
		cv::imshow("sensor_silhouette", worker->handfinder->sensor_silhouette); cv::waitKey(1);
		cv::Mat normalized_rendered_model;
		cv::normalize(rendered_model, normalized_rendered_model, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		cv::imshow("rendered_model", normalized_rendered_model); cv::waitKey(1);

		// Compute metrics
		float pull_error = online_performance_metrics.compute_rastorized_3D_metric(
			rendered_model, worker->current_frame.depth, worker->handfinder->sensor_silhouette, worker->camera->inv_projection_matrix());
		float push_error = online_performance_metrics.compute_rastorized_2D_metric(
			rendered_model, worker->handfinder->sensor_silhouette, distance_transform.idxs_image());

		// Write metric
		static ofstream rastorized_error_file(data_path + "hmodel_rastorized_error.txt");
		cout << pull_error << " " << push_error << endl;
		if (rastorized_error_file.is_open()) {
			rastorized_error_file << pull_error << " " << push_error << endl;
		}

		// Write cropped model	
		cv::imwrite(data_path + "Taylor_Cropped/model-" + stringstream.str() + ".png", rendered_model);

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

	void load_recorded_frame(size_t current_frame) {

		std::ostringstream stringstream;
		stringstream << std::setw(7) << std::setfill('0') << current_frame;
		worker->current_frame.depth = cv::imread(data_path + "depth-" + stringstream.str() + ".png", cv::IMREAD_ANYDEPTH);
		worker->current_frame.color = cv::imread(data_path + "color-" + stringstream.str() + ".png");
		worker->model->real_color = cv::imread(data_path + "full_color-" + stringstream.str() + ".png");
	}

	void display_color_and_depth_input() {
		cv::Mat normalized_depth = worker->current_frame.depth.clone();
		cv::inRange(normalized_depth, worker->camera->zNear(), worker->camera->zFar(), normalized_depth);
		cv::normalize(normalized_depth, normalized_depth, 127, 255, cv::NORM_MINMAX, CV_8UC1);
		cv::resize(normalized_depth, normalized_depth, cv::Size(2 * normalized_depth.cols, 2 * normalized_depth.rows), cv::INTER_CUBIC);//resize image
		cv::moveWindow("DEPTH", 592, 855); cv::imshow("DEPTH", normalized_depth);

		cv::namedWindow("RGB");	cv::moveWindow("RGB", 592, 375); cv::imshow("RGB", worker->model->real_color);
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

};


