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

Worker::Worker(Camera *camera, bool test, bool benchmark, bool save_rasotrized_model, int user_name, std::string data_path, Handedness handedness) {

	this->camera = camera;
	this->benchmark = benchmark;
	this->test = test;
	this->save_rastorized_model = save_rasotrized_model;
	this->user_name = user_name;
	this->data_path = data_path;
	this->handedness = handedness;

	this->model = new Model();
	this->activeModel = this->model;
	if (handedness == right_hand || handedness == both_hands) {
		this->model->init(user_name, data_path, right_hand);		
	}
	else {
		this->model->init(user_name, data_path, left_hand);
	}
	std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	theta_initial[1] = 70; theta_initial[2] = 400;
	model->move(theta_initial);
	model->update_centers();
	model->compute_outline();

	if (handedness == both_hands) {
		model2 = new Model();  //Right
		model2->init(user_name, data_path, left_hand);
		theta_initial[0] = -70;
		model2->move(theta_initial);
		model2->update_centers();
		model2->compute_outline();
	}

	//Brandon
	Bayes_mu = std::vector<std::vector<float>>();
	Bayes_sig = std::vector<std::vector<float>>();
	read_bayes_vectors(data_path, "classifiers/mu_params.txt", Bayes_mu);
	read_bayes_vectors(data_path, "classifiers/sig_params.txt", Bayes_sig);
	read_class_names(data_path,"classifiers/classes.txt");

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
	handfinder = new HandFinder(camera, handedness, data_path);
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
	//std::cout << "opening: " << data_path << name << std::endl;
	FILE *fp = fopen((data_path + name ).c_str(), "r");
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
	std::cout << "Reading some vecotr for the classifier" << std::endl;
}

void Worker::read_class_names(std::string data_path, std::string name){
	std::string full_path = data_path + name;
	std::ifstream fp(full_path);
	
	int N = 41;

	for (int i = 0; i < N; ++i) {
		std::string name;
		fp >> name;
		class_names.push_back(name);
	}
	fp.close();
}

int Worker::classify(){
//void calc_max_likelihood(std::vector<float> theta){
	int N = 41;
	int class_max = 0;
	float max_likelihood = -999999;
	float likelihood = 0;
	for (int i = 0; i < N; ++i){
		likelihood = 0;
		for (int j = 0; j < 20; ++j){			
			likelihood += -.5*log(2 * M_PI * Bayes_sig[i][j]) - (model->theta[j+9] - Bayes_mu[i][j])*(model->theta[j+9] - Bayes_mu[i][j]) / (2 * Bayes_sig[i][j]);
		}
		if (i == 0)
			max_likelihood = likelihood;
		if (likelihood > max_likelihood){			
			max_likelihood = likelihood;
			class_max = i;
		}
		//std::cout << "Class " << i << " likelihood: " << likelihood << std::endl;
	}
	//std::cout << "Most likely Class: " << class_names[class_max] << std::endl;

	return class_max;

}

Model* Worker::get_active_model() {
	return activeModel;
}