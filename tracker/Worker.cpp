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
	if (handedness == both_hands) {
		this->model->init(3, data_path, 0);  //Left
	}
	else {
		this->model->init(user_name, data_path, handedness);
	}

	std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	theta_initial[0] = 60; theta_initial[1] = -70; theta_initial[2] = 400;
	model->move(theta_initial);

	model->update_centers();
	model->compute_outline();

<<<<<<< HEAD
	//this->model2 = new Model();
	//this->model2->init(user_name, data_path);
	//std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	//theta_initial[1] = +70; theta_initial[2] = 400;
	//model2->move(theta_initial);

	//model2->update_centers();
	//model2->compute_outline();

	//Brandon
=======
>>>>>>> refs/remotes/origin/master
	Bayes_mu = std::vector<std::vector<float>>();
	Bayes_sig = std::vector<std::vector<float>>();
	std::string path = "C:/Projects/ASLRecog/";
	std::string f1 = "mu_params.txt";
	std::string f2 = "sig_params.txt";
<<<<<<< HEAD
	std::cout << "gonna read something" << std::endl;
	read_bayes_vectors(path, f1, Bayes_mu);
	read_bayes_vectors(path, f2, Bayes_sig);
	read_class_names();
	//read_bayes_vectors("C:/Projects/ASLRecog/", "mu_params.txt", Bayes_mu);
	//read_bayes_vectors("C:/Projects/ASLRecog/", "sig_params.txt", Bayes_sig);

	lock_tracking = true;
=======
	read_bayes_vectors(path, f1, Bayes_mu);
	read_bayes_vectors(path, f2, Bayes_sig);
	read_class_names();

	if (handedness == both_hands || handedness == right_hand) {
		model2 = new Model();  //Right
		model2->init(4, data_path, left_hand);
		theta_initial[0] = -60; 
		model2->move(theta_initial);
		model2->update_centers();
		model2->compute_outline();
	}
>>>>>>> refs/remotes/origin/master

	if (user_name == 0) model->manually_adjust_initial_transformations();
	//if (user_name == 0) model2->manually_adjust_initial_transformations();
}

/// @note any initialization that has to be done once GL context is active
void Worker::init_graphic_resources() {
<<<<<<< HEAD
	//offscreen_renderer.init(camera, model, model2, data_path, true);
	offscreen_renderer.init(camera, model, data_path, true);
	//if (save_rastorized_model) rastorizer.init(camera, model, model2, data_path, false);
=======
	if (handedness == both_hands) {
		offscreen_renderer.init(camera, model, data_path, true,handedness,model2);
	}
	else {
		offscreen_renderer.init(camera, model, data_path, true, handedness, model2);
		//offscreen_renderer.init(camera, model, data_path, true);
	}
>>>>>>> refs/remotes/origin/master
	if (save_rastorized_model) rastorizer.init(camera, model, data_path, false);
	sensor_color_texture = new ColorTexture8UC3(camera->width(), camera->height());
	sensor_depth_texture = new DepthTexture16UC1(camera->width(), camera->height());

	tw_settings->tw_add(settings->termination_max_iters, "#iters", "group=Tracker");
	tw_settings->tw_add(settings->termination_max_rigid_iters, "#iters (rigid)", "group=Tracker");

	///--- Initialize the energies modules
	using namespace energy;
	trivial_detector = new TrivialDetector(camera, &offscreen_renderer);
<<<<<<< HEAD
	handfinder = new HandFinder(camera);
	R_Handfinder = new HandFinder(camera);
=======

	if (handedness == both_hands) {
		handfinder = new HandFinder(camera, left_hand);
		if (!(handfinder == NULL)) cout << "handfinder not NULL" << endl;
		handfinder2 = new HandFinder(camera, right_hand);
		if (!(handfinder2 == NULL)) cout << "handfinder2 not NULL" << endl;
	}else {
		handfinder = new HandFinder(camera, handedness);
		if (handedness == right_hand)
			handfinder2 = new HandFinder(camera, left_hand);
	}
	if (!(handfinder == NULL)) cout << "handfinder not NULL" << endl;
	if (!(handfinder2 == NULL)) cout << "handfinder2 not NULL" << endl;
	
>>>>>>> refs/remotes/origin/master
	E_fitting.init(this);

	E_limits.init(model);
	E_collision.init(model);
	E_pose.init(this);
	E_temporal.init(model);
	E_damping.init(model);
	/*
	E_limits2.init(model2);
	E_collision2.init(model2);
	E_pose2.init(this);
	E_temporal2.init(model2);
	E_damping2.init(model2);*/
}

void Worker::cleanup_graphic_resources() {
	delete sensor_color_texture;
	delete sensor_depth_texture;
	E_fitting.cleanup();
}

Worker::~Worker() {
	delete trivial_detector;
	delete handfinder;
	delete R_Handfinder;
	delete model;
	//delete model2;
}

<<<<<<< HEAD
void Worker::toggle_tracking_lock(){
	lock_tracking = !lock_tracking;
}

/*
bool Worker::track_till_convergence(int model_num) {
	//Brandon did this to disable tracking.
	if (lock_tracking) return false;

	if (model_num == 1){
		for (int i = 0; i < settings->termination_max_iters; ++i) {
			track(i,model,1);
			//tracking_error_optimization[i] = tracking_error;
		}

		return monitor.is_failure_frame(tracking_error.pull_error, tracking_error.push_error, E_fitting.settings->fit2D_enable);
	}
	else{
		for (int i = 0; i < settings->termination_max_iters; ++i) {
			track(i,model2,2);
			//tracking_error_optimization[i] = tracking_error;
		}

		return monitor.is_failure_frame(tracking_error.pull_error, tracking_error.push_error, E_fitting.settings->fit2D_enable);

	}
}*/


bool Worker::track_till_convergence() {
	//Brandon did this to disable tracking.
	if (lock_tracking) {
		std::vector<float> _thetas = model->get_theta();
		for (int i = 6; i < num_thetas; ++i){
			_thetas[i] = 0;
		}
		model->compute_rendered_indicator(this->handfinder->sensor_silhouette, this->camera);
		model->move(_thetas);
		model->update_centers();
		model->compute_outline();
		return true; //false;
=======
void Worker::swap_hands() {
	if (handedness == right_hand)
		handedness = left_hand;
	if (handedness == left_hand)
		handedness = right_hand;

	Model * modTemp = model;
	model = model2;
	model2 = modTemp;

	HandFinder* hfTemp = handfinder;
	handfinder = handfinder2;
	handfinder2 = hfTemp;
	
	E_fitting.swapHands();
	
	E_collision.model = model;
	E_temporal.model = model;
	E_damping.model = model;
}

bool Worker::track_till_convergence(int model) {

	for (int i = 0; i < settings->termination_max_iters; ++i) {
		track2(i);
		//tracking_error_optimization[i] = tracking_error;
	}

	std::vector<float> thetas = model2->get_theta();
	return monitor.is_failure_frame(tracking_error.pull_error, tracking_error.push_error, E_fitting.settings->fit2D_enable);
}

bool Worker::track_till_convergence() {
	//cout << (long)model << " " << (long)E_fitting.model << " " << (long)E_collision.model << " " << (long)E_temporal.model << " " << (long)E_damping.model << endl;

	if (handfinder->sensor_silhouette.empty()) {
		cout << "empty sensor_silhouette in track_till_convergence" << endl;
		return true;
>>>>>>> refs/remotes/origin/master
	}
	for (int i = 0; i < settings->termination_max_iters; ++i) {
		track(i);
		//tracking_error_optimization[i] = tracking_error;
	}

	std::vector<float> thetas = model->get_theta();
	return monitor.is_failure_frame(tracking_error.pull_error, tracking_error.push_error, E_fitting.settings->fit2D_enable);
}

void Worker::track(int iter){
	bool eval_error = (iter == settings->termination_max_iters - 1);
	bool rigid_only = (iter < settings->termination_max_rigid_iters);

	std::vector<float> _thetas = model->get_theta();

	///--- Serialize matrices for jacobian computation
	model->serializer.serialize_model();
	//model->compute_rendered_indicator(handfinder->sensor_silhouette, camera);

	///--- Optimization phases	
	LinearSystem system(num_thetas);
<<<<<<< HEAD
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
	
	
=======

	E_fitting.track(current_frame, system, rigid_only, eval_error, tracking_error.push_error, tracking_error.pull_error, iter); ///<!!! MUST BE FIRST CALL		
	E_collision.track(system);
	//E_temporal.track(system, current_frame);
	E_limits.track(system, _thetas);
	E_damping.track(system);
	if (rigid_only)
		energy::Energy::rigid_only(system);
	else	
		E_pose.track(system, _thetas); ///<!!! MUST BE LAST CALL	

>>>>>>> refs/remotes/origin/master
	///--- Solve 
	VectorN delta_thetas = energy::Energy::solve(system);

	///--- Update
	const vector<float> dt(delta_thetas.data(), delta_thetas.data() + num_thetas);

	_thetas = model->get_updated_parameters(_thetas, dt);
<<<<<<< HEAD
//	if (model_num == 2){
//		_thetas[0] = _thetas[0] + 100;
//	}
=======

>>>>>>> refs/remotes/origin/master
	model->move(_thetas);
	model->update_centers();
	model->compute_outline();
	E_temporal.update(current_frame.id, _thetas);
}

<<<<<<< HEAD
/*
void Worker::track(int iter, Model* model, int model_num) {
	bool eval_error = (iter == settings->termination_max_iters - 1);
	bool rigid_only = (iter < settings->termination_max_rigid_iters);

	std::vector<float> _thetas = model->get_theta();

	///--- Serialize matrices for jacobian computation
	model->serializer.serialize_model();
=======
void Worker::track2(int iter) {
	bool eval_error = (iter == settings->termination_max_iters - 1);
	bool rigid_only = (iter < settings->termination_max_rigid_iters);

	std::vector<float> _thetas = model2->get_theta();
	///--- Serialize matrices for jacobian computation
	model2->serializer.serialize_model();
>>>>>>> refs/remotes/origin/master
	//model->compute_rendered_indicator(handfinder->sensor_silhouette, camera);

	///--- Optimization phases	
	LinearSystem system(num_thetas);
<<<<<<< HEAD
	//eval_error = true;
	if (model_num == 1){
		E_fitting.track(current_frame, system, rigid_only, eval_error, tracking_error.push_error, tracking_error.pull_error, iter); ///<!!! MUST BE FIRST CALL		
		E_collision.track(system);
		E_temporal.track(system, current_frame);
		E_limits.track(system, _thetas);
		E_damping.track(system);
		if (rigid_only)
			energy::Energy::rigid_only(system);
		else
			E_pose.track(system, _thetas); ///<!!! MUST BE LAST CALL	
	}
	else{
		E_fitting2.track(current_frame, system, rigid_only, eval_error, tracking_error.push_error, tracking_error.pull_error, iter); ///<!!! MUST BE FIRST CALL		
		E_collision2.track(system);
		E_temporal2.track(system, current_frame);
		E_limits2.track(system, _thetas);
		E_damping2.track(system);
		if (rigid_only)
			energy::Energy::rigid_only(system);
		else
			E_pose2.track(system, _thetas); ///<!!! MUST BE LAST CALL	
	}
	///--- Solve 
=======
	E_fitting.track(current_frame, system, rigid_only, eval_error, tracking_error.push_error, tracking_error.pull_error, iter, 2); ///<!!! MUST BE FIRST CALL		

																																//std::cout << "2: " << eval_error << tracking_error.push_error << tracking_error.pull_error << std::endl;
																																//std::cout << "system.rhs: ";
																																//for (int i = 0; i < num_thetas; ++i){
																																//	std::cout << system.rhs[i] << " ";
																																//}
																																//std::cout << std::endl;

	E_collision.track(system);
	E_temporal.track(system, current_frame);
	E_limits.track(system, _thetas);
	E_damping.track(system);
	if (rigid_only)
		energy::Energy::rigid_only(system);
	else
		E_pose.track(system, _thetas); ///<!!! MUST BE LAST CALL	

									   ///--- Solve 
>>>>>>> refs/remotes/origin/master
	VectorN delta_thetas = energy::Energy::solve(system);

	///--- Update
	const vector<float> dt(delta_thetas.data(), delta_thetas.data() + num_thetas);
<<<<<<< HEAD
	_thetas = model->get_updated_parameters(_thetas, dt);
	if (model_num == 2){
		_thetas[0] = _thetas[0] + 100;
	}
	model->move(_thetas);
	model->update_centers();
	model->compute_outline();
	E_temporal.update(current_frame.id, _thetas);
}*/

void Worker::read_bayes_vectors(std::string data_path, std::string name, std::vector<std::vector<float>> & input) {
	//std::cout << "opening: " << data_path << name << std::endl;
	FILE *fp = fopen((data_path + name ).c_str(), "r");
	int N = 41;

	//fscanf(fp, "%d", &N);
	for (int i = 0; i < N; ++i) {
		std::vector<float> theta = std::vector<float>();
		for (int j = 0; j < 20; ++j){
=======

	_thetas = model2->get_updated_parameters(_thetas, dt);

	model2->move(_thetas);
	model2->update_centers();
	model2->compute_outline();
	E_temporal.update(current_frame.id, _thetas);
}

void Worker::read_bayes_vectors(std::string data_path, std::string name, std::vector<std::vector<float>> & input) {
	FILE *fp = fopen((data_path + name).c_str(), "r");
	int N = 41;

	for (int i = 0; i < N; ++i) {
		std::vector<float> theta = std::vector<float>();
		for (int j = 0; j < 20; ++j) {
>>>>>>> refs/remotes/origin/master
			float a;
			fscanf(fp, "%f", &a);
			theta.push_back(a);
		}
		input.push_back(theta);
<<<<<<< HEAD
		//std::cout << "one value is: " << theta[3] << std::endl;
	}
	fclose(fp);
	//std::cout << "Reading some vecotr for the classifier" << std::endl;
}

void Worker::read_class_names(){
	std::string data_path = "C:/Projects/ASLRecog/classes.txt";
	std::ifstream fp("C:/Projects/ASLRecog/classes.txt");
	//FILE *fp = fopen((data_path).c_str(), "r");
	int N = 41;

	//fscanf(fp, "%d", &N);
	for (int i = 0; i < N; ++i) {
		std::string name;
		fp >> name;
		//fscanf(fp, "%s", &name);
		class_names.push_back(name);
		//std::cout << "one class is: " << name << std::endl;
	}
	fp.close();
	//	fclose(fp);
	//std::cout << "Reading some vecotr for the classifier" << std::endl;
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

=======
	}
	fclose(fp);
}

void Worker::read_class_names() {
	std::ifstream fp("C:/Projects/ASLRecog/classes.txt");
	int N = 41;

	for (int i = 0; i < N; ++i) {
		std::string name;
		fp >> name;
		class_names.push_back(name);
	}
	fp.close();
}

int Worker::classify() {
	int N = 41;
	int class_max = 0;
	float max_likelihood = -9999999999;
	float likelihood = 0;
	for (int i = 0; i < N; ++i) {
		likelihood = 0;
		for (int j = 0; j < 20; ++j) {
			likelihood += -.5*log(2 * M_PI * Bayes_sig[i][j]) - (model->theta[j + 9] - Bayes_mu[i][j])*(model->theta[j + 9] - Bayes_mu[i][j]) / (2 * Bayes_sig[i][j]);
		}
		if (i == 0)
			max_likelihood = likelihood;
		if (likelihood > max_likelihood) {
			max_likelihood = likelihood;
			class_max = i;
		}
	}
	return class_max;
>>>>>>> refs/remotes/origin/master
}