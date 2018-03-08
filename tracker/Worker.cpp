#include "Worker.h"
#include "util/gl_wrapper.h"
#include "util/tictoc.h"

#include <QElapsedTimer>
#include <QGLWidget>

#include "tracker/ForwardDeclarations.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Energy/Energy.h"
#include "tracker/TwSettings.h"
#include "tracker/HModel/Model.h"

#include <ctime>

void Worker::updateGL() { if (glarea != NULL) glarea->updateGL(); }

Worker::Worker(Camera *camera, bool test, bool benchmark, bool save_rasotrized_model, int user_name, std::string data_path, Handedness handedness, Sensor *sensor) {

	this->camera = camera;
	this->benchmark = benchmark;
	this->test = test;
	this->save_rastorized_model = save_rasotrized_model;
	this->user_name = user_name;
	this->data_path = data_path;
	this->handedness = handedness;
	this->sensor = sensor;

	this->model_1 = new Model();
	if (handedness == right_hand || handedness == both_hands) {
		this->model_1->init(user_name, data_path, right_hand);		
	}
	else {
		this->model_1->init(user_name, data_path, left_hand);
	}
	std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	theta_initial[1] = 70; theta_initial[2] = 400;
	model_1->move(theta_initial);
	model_1->update_centers();
	model_1->compute_outline();
	
	//set active model to model_1
	this->model = this->model_1;

	if (handedness == both_hands) {
		cout << "Initializing 2nd Hand Model." << endl;
		model_2 = new Model();  //Right
		model_2->init(user_name, data_path, left_hand);
		theta_initial[0] = -70; theta_initial[1] = 0;
		model_2->move(theta_initial);
		model_2->update_centers();
		model_2->compute_outline();
	}

	//Brandon
	Bayes_mu = std::vector<std::vector<float>>();
	Bayes_sig = std::vector<std::vector<float>>();
	read_bayes_vectors(data_path, "classifiers/b_mu_params.txt", Bayes_mu);
	read_bayes_vectors(data_path, "classifiers/b_sig_params.txt", Bayes_sig);
	read_class_names(data_path,"classifiers/test_classes.txt");
	errors.reserve(30 * 60 * 5);
	thetas.reserve(30 * 60 * 5);
}

/// @note any initialization that has to be done once GL context is active
void Worker::init_graphic_resources() {
	offscreen_renderer.init(camera, this, data_path, true);
	if (save_rastorized_model) rastorizer.init(camera, this, data_path, false);
	sensor_color_texture = new ColorTexture8UC3(camera->width(), camera->height());
	sensor_depth_texture = new DepthTexture16UC1(camera->width(), camera->height());

	tw_settings->tw_add(settings->termination_max_iters, "#iters", "group=Tracker");
	tw_settings->tw_add(settings->termination_max_rigid_iters, "#iters (rigid)", "group=Tracker");

	///--- Initialize the energies modules
	using namespace energy;
	trivial_detector = new TrivialDetector(camera, &offscreen_renderer);
	handfinder_1 = new HandFinder(camera, handedness, data_path,user_name);
	//Brandon moved here.
	cout << "worker HandFinder just initialized" << endl;
	cout << " address passed: " << &(handfinder_1->settings->show_wband) << endl;
	tw_settings->tw_add(handfinder_1->settings->show_wband, "show_wband", "group=HandFinder");
	tw_settings->tw_add(sensor->handfinder->settings->show_wband, "sensor_wband", "group=HandFinder");
	if (handedness == both_hands) {
		handfinder_2 = new HandFinder(camera, left_hand, data_path,user_name);
	}
	handfinder = handfinder_1;

	E_fitting.init(this);
	E_limits.init(model);
	E_collision.init(model);
	E_pose.init(this);
	E_temporal.init(model);
	E_damping.init(model);
}

void Worker::cleanup_graphic_resources() {
	delete sensor_color_texture;
	delete sensor_depth_texture;
	E_fitting.cleanup();
}

Worker::~Worker() {
	delete trivial_detector;
	delete handfinder;
	delete model;
}

bool Worker::track_till_convergence() {	
	for (int i = 0; i < settings->termination_max_iters; ++i) {
		track(i);
		//tracking_error_optimization[i] = tracking_error;
	}
	
	return monitor.is_failure_frame(tracking_error.pull_error, tracking_error.push_error, E_fitting.settings->fit2D_enable);
}

void Worker::track(int iter) {
	bool eval_error = (iter == settings->termination_max_iters - 1);
	bool rigid_only = (iter < settings->termination_max_rigid_iters);

	std::vector<float> _thetas = model->get_theta();

	///--- Serialize matrices for jacobian computation
	model->serializer.serialize_model();
	//model->compute_rendered_indicator(handfinder->sensor_silhouette, camera);

	///--- Optimization phases	
	LinearSystem system(num_thetas);
	//eval_error = true;
	E_fitting.track(current_frame, system, rigid_only, eval_error, tracking_error.push_error, tracking_error.pull_error, iter); ///<!!! MUST BE FIRST CALL		
	E_collision.track(system);
	E_temporal.track(system, current_frame);
	E_limits.track(system, _thetas);
	E_damping.track(system);
	if (rigid_only) 
		energy::Energy::rigid_only(system);
	else
		E_pose.track(system, _thetas); ///<!!! MUST BE LAST CALL	

	///--- Solve 
	VectorN delta_thetas = energy::Energy::solve(system);	

	///--- Update
	const vector<float> dt(delta_thetas.data(), delta_thetas.data() + num_thetas);
	_thetas = model->get_updated_parameters(_thetas, dt);
	model->move(_thetas);
	model->update_centers();
	model->compute_outline();
	E_temporal.update(current_frame.id, _thetas);
}

void Worker::read_bayes_vectors(std::string data_path, std::string name, std::vector<std::vector<float>> & input) {
	FILE *fp = fopen((data_path + name ).c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);

	for (int i = 0; i < N; ++i) {
		std::vector<float> theta = std::vector<float>();
		for (int j = 0; j < 23; ++j){
			float a;
			fscanf(fp, "%f", &a);
			theta.push_back(a);
		}
		input.push_back(theta);
	}
	fclose(fp);
}

void Worker::read_class_names(std::string data_path, std::string name){
	std::string full_path = data_path + name;
	std::ifstream fp(full_path);

	int N = 25;
	//fscanf(fp, "%d", &N);

	for (int i = 0; i < N; ++i) {
		std::string name;
		fp >> name;
		class_names.push_back(name);
	}
	fp.close();
}

int Worker::classify() {
	std::vector<float> t = model->get_theta();
	Eigen::Matrix3f mat;
	mat = Eigen::AngleAxisf(t[3], Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(t[4], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(t[5], Eigen::Vector3f::UnitZ());
	Eigen::Vector3f y_off = mat * Eigen::Vector3f::UnitY();
	Eigen::Vector3f x_off = mat * Eigen::Vector3f::UnitX();
	Eigen::Vector3f z_off = mat * Eigen::Vector3f::UnitZ();
	t.push_back(acos(y_off[1]));
	t.push_back(acos(x_off[1]));
	t.push_back(acos(-1*z_off[1]));	
	return classify(t);
}

int Worker::classify(std::vector<float> theta) {
	int N = 25;
	int class_max = 0;
	if (get_active_model()->handedness == right_hand) {
		theta[7] *= -1;
		theta[9] *= -1;
		theta[13] *= -1;
		theta[17] *= -1;
		theta[21] *= -1;
		theta[25] *= -1;
	}

	float max_likelihood = -999999;
	float likelihood = 0;
	std::vector<float> likelihoods(25, 0);
	for (int i = 0; i < N; ++i){
		likelihood = 0;
		for (int j = 0; j < 23; ++j){			
			likelihood += -.5*log(2 * M_PI * Bayes_sig[i][j]) - (theta[j+9] - Bayes_mu[i][j])*(theta[j+9] - Bayes_mu[i][j]) / (2 * Bayes_sig[i][j]);
		}
		likelihoods[i] = likelihood;
		if (i == 0)
			max_likelihood = likelihood;
		if (likelihood > max_likelihood){			
			max_likelihood = likelihood;
			class_max = i;
		}
	}
		cout << "possible classes: ";
	for (int i = 0; i < 25; ++i) {
		if (exp(likelihoods[i]) > .5*exp(max_likelihood)) {
			cout << " " << class_names[i];
		}
	}
		cout << endl;

	return class_max;

}

Model* Worker::get_active_model() {
	return model;
}

HandFinder* Worker::get_active_handfinder() {
	return handfinder;
}

void Worker::swap_hands() {
	if (model == model_1) {
		model = model_2;
		handfinder = handfinder_2;
	}
	else {
		model = model_1;
		handfinder = handfinder_1;
	}
	E_fitting.set_model(model, handfinder);
	E_collision.set_model(model);
	E_temporal.set_model(model);
	E_damping.set_model(model);
}

Model* Worker::get_right_model() {
	if (model_1->handedness == right_hand)
		return model_1;
	else
		return NULL;
}

Model* Worker::get_left_model() {
	if (model_1->handedness == left_hand)
		return model_1;
	else 
		return model_2;
}

void Worker::set_focus() {
	glarea->activateWindow();
}

void Worker::add_tracking_data(Thetas theta) {
	thetas.push_back(theta);
	std::vector<float> errs;
	errs.push_back(this->tracking_error.pull_error);
	errs.push_back(this->tracking_error.push_error);
	errors.push_back(errs);
}

void Worker::save_tracking_data(std::string path) {
	cout << "Tracking Data Saving to: " << path << endl;
	std::string tracking_error_filename = path + "hmodel_tracking_error_.txt";
	std::string solutions_filename = path + "hmodel_solutions_.txt";

	ofstream tracking_error_file(tracking_error_filename, ios_base::app);
	ofstream solutions_file(solutions_filename, ios_base::app);

	for (int i = 0; i < errors.size(); ++i) {
		tracking_error_file << errors[i][0] << " " << errors[i][1] << endl;
		solutions_file << thetas[i].transpose() << endl;
	}

	tracking_error_file.close();
	solutions_file.close();

	thetas.clear();
	errors.clear();	
}

void Worker::clear_tracking_data() {
	thetas.clear();
	errors.clear();
}