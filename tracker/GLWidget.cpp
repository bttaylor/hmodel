
#include "util/gl_wrapper.h"
#include "util/OpenGL32Format.h" 
#include "apps/hmodel_atb/AntTweakBarEventFilter.h"

#include "TwSettings.h"
#include "GLWidget.h"
#include "tracker/Data/Camera.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Worker.h"
#include "tracker/Data/SolutionStream.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HandFinder/HandFinder.h"

#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"

#define M_PI 3.14159265358979323846

GLWidget::GLWidget(Worker* worker, DataCollector* collector, DataStream * datastream, SolutionStream * solutions, bool playback, bool real_color, std::string data_path, std::string store_path) :
QGLWidget(OpenGL32Format()),
worker(worker),
datastream(datastream),
collector(collector),
solutions(solutions),
_camera(worker->camera),
store_path(store_path),
convolution_renderer(worker->model, real_color, data_path) {
	this->playback = playback;
	this->data_path = data_path;
	this->resize(640 * 2, 480 * 2);
	this->move(1250, 375);
	convolution_renderer.window_width = this->width();
	convolution_renderer.window_height = this->height();

	std::cout << "Started OpenGL " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;
	this->installEventFilter(new AntTweakBarEventFilter(this)); ///< all actions pass through filter
	recording = false;
	set = 0;
	load_prompts(set++);
	prompt_i = 0;
	for (int i = 0; i < 100; i++){
		prompt_order[i] = i;
	}
	std::random_shuffle(std::begin(prompt_order), std::end(prompt_order));
	scale_adj = 0;
	width_adj = 0;
	thick_adj = 0;
}

GLWidget::~GLWidget() {
	worker->cleanup_graphic_resources();
	tw_settings->tw_cleanup();
}

void GLWidget::initializeGL() {
	//TEST

	Model* temp_model = new Model();
	temp_model->init(10, data_path, 0);
	cout << "Temp Model initialized. centers.size(): " << temp_model->centers.size() << endl;
	std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	theta_initial[0] = 30; theta_initial[1] = 10; theta_initial[2] = 250;
	theta_initial[5] = 1.6;
	temp_model->move(theta_initial);
	temp_model->update_centers();
	temp_model->compute_outline();
	
	if(worker->handedness == both_hands)
		convolution_renderer.model2 = worker->model2;  //THis is the one getting drawn.
		//endtest
	if (worker->model2 != NULL) {
		convolution_renderer.model2 = worker->model2;
	}

	std::cout << "GLWidget::initializeGL()" << std::endl;
	initialize_glew();
	tw_settings->tw_init(this->width(), this->height()); ///< FIRST!!

	glEnable(GL_DEPTH_TEST);

	kinect_renderer.init(_camera);

	///--- Initialize other graphic resources
	this->makeCurrent();
	worker->init_graphic_resources();

	///--- Setup with data from worker
	kinect_renderer.setup(worker->sensor_color_texture->texid(), worker->sensor_depth_texture->texid());

	convolution_renderer.projection = _camera->view_projection_matrix();
	convolution_renderer.init(ConvolutionRenderer::NORMAL);
	cout << "end of initializeGL " << endl;
}

void GLWidget::paintGL() {
	glViewport(0, 0, this->width(), this->height());
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	///--- Rendering
	Eigen::Matrix4f view_projection = _camera->view_projection_matrix() * view;
	if (worker->handfinder->wristband_found()) {
		kinect_renderer.enable_colormap(true);
		kinect_renderer.set_zNear(worker->handfinder->wristband_center()[2] - 150);
		kinect_renderer.set_zFar(worker->handfinder->wristband_center()[2] + 150);
	}
	kinect_renderer.set_uniform("view_projection", view_projection);
	kinect_renderer.render();
	

	glDisable(GL_BLEND);
	convolution_renderer.render();

	//worker->model->render_outline();
	//DebugRenderer::instance().set_uniform("view_projection", view_projection);
	//DebugRenderer::instance().render();

	//tw_settings->tw_draw();
}

void GLWidget::process_mouse_movement(GLfloat cursor_x, GLfloat cursor_y) {
	glm::vec3 image_center_glm = worker->model->centers[worker->model->centers_name_to_id_map["palm_back"]] +
		worker->model->centers[worker->model->centers_name_to_id_map["palm_middle"]];
	image_center = Eigen::Vector3f(image_center_glm[0] / 2, image_center_glm[1] / 2 + 30, image_center_glm[2] / 2);
	float d = (camera_center - image_center).norm();

	float delta_x = cursor_x - cursor_position[0];
	float delta_y = cursor_y - cursor_position[1];

	float theta = initial_euler_angles[0] + cursor_sensitivity * delta_x;
	float phi = initial_euler_angles[1] + cursor_sensitivity * delta_y;

	Eigen::Vector3f x = sin(theta) * sin(phi) * Eigen::Vector3f::UnitX();
	Eigen::Vector3f y = cos(phi) * Eigen::Vector3f::UnitY();
	Eigen::Vector3f z = cos(theta) * sin(phi) * Eigen::Vector3f::UnitZ();

	camera_center = image_center + d * (x + y + z);
	euler_angles = Eigen::Vector2f(theta, phi);

	Vector3 f, u, s;
	f = (image_center - camera_center).normalized();
	u = camera_up.normalized();
	s = u.cross(f).normalized();
	u = f.cross(s);
	view.block(0, 0, 1, 3) = s.transpose();
	view(0, 3) = -s.dot(camera_center);
	view.block(1, 0, 1, 3) = u.transpose();
	view(1, 3) = -u.dot(camera_center);
	view.block(2, 0, 1, 3) = f.transpose();
	view(2, 3) = -f.dot(camera_center);

	// set view matrix 
	convolution_renderer.camera.view = view;
	convolution_renderer.camera.camera_center = camera_center;

	worker->offscreen_renderer.convolution_renderer->camera.view = view;
	worker->offscreen_renderer.convolution_renderer->camera.camera_center = camera_center;
}

void GLWidget::process_mouse_button_pressed(GLfloat cursor_x, GLfloat cursor_y) {
	mouse_button_pressed = true;
	cursor_position = Eigen::Vector2f(cursor_x, cursor_y);
}

void GLWidget::process_mouse_button_released() {
	initial_euler_angles = euler_angles;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
	if (event->buttons() == Qt::LeftButton) {
		process_mouse_movement(event->x(), event->y());
	}
	else {
		if (mouse_button_pressed == true) {
			process_mouse_button_released();
			mouse_button_pressed = false;
		}
	}
}

void GLWidget::mousePressEvent(QMouseEvent *event) {
	process_mouse_button_pressed(event->x(), event->y());
}

void GLWidget::wheelEvent(QWheelEvent * event) {}

void GLWidget::keyPressEvent(QKeyEvent *event) {
	GLWidget* qglviewer = this;
	switch (event->key()) {
	case Qt::Key_Escape: {
		this->close();
	}
		break;
	case Qt::Key_S: {
						cout << "set up path for saving images" << std::endl;
						datastream->save_as_images("C:/Projects/test/");
	}
		break;
	case Qt::Key_Y:
	{
					  std::ofstream outfile;
					  std::string path = store_path + "modelAdjust.txt";					  
					  outfile.open(path, std::ofstream::app);					  
					  outfile << scale_adj << " " << width_adj << " " << thick_adj << " " << std::endl; current_prompt = get_next_prompt();
					  std::cout << "Loading prompt " << prompt_i << ": " << current_prompt << std::endl;
					  cv::Mat word = cv::Mat::ones(100, 500, CV_8UC3);
					  word = cv::Scalar(255, 255, 255);
					  //cv::Mat prompt = cv::imread("C:/Projects/Data/Prompt_Images/1.png");																
					  cv::putText(word, current_prompt, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 2, 0);
					  cv::imshow("show_prompt", word); // prompt);
					  cv::waitKey(1);

					  //current_prompt = "1";
					  //prompt_i++;
					  //std::cout << "Loading prompt: " << "C:/Projects/Data/Prompt_Images/" + current_prompt << std::endl;
					  //cv::Mat prompt = cv::imread("C:/Projects/Data/Prompt_Images/1.png");
					  //cv::imshow("show_prompt", prompt);
					  //cv::waitKey(1);
	}
		break;
	case Qt::Key_Space: {
							if (!recording){
								std::cout << "Recording..." << std::endl;
								if (myoEnable)
									collector->recording = true;
								datastream->clear_stream_buffer();
								recording = true;
							}
							else{
								std::cout << "End Recording" << std::endl;
								recording = false;
								datastream->save_as_images(store_path + current_prompt);
								//datastream->save_as_images("C:/Projects/Data/Brandon/FingerSpell/");
								//std::cout << "post datastream->save_as_images" << std::endl;
								if (myoEnable) {
									collector->recording = false;
									collector->saveMyoData(store_path + current_prompt);
								}
								
								current_prompt = get_next_prompt();
								std::cout << "Loading prompt " << prompt_i << ": " << current_prompt << std::endl;
								cv::Mat word = cv::Mat::ones(100, 500, CV_8UC3);				
								word = cv::Scalar(255, 255, 255);
								//cv::Mat prompt = cv::imread("C:/Projects/Data/Prompt_Images/1.png");																
								cv::putText(word, current_prompt, cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 2, 0);
								cv::imshow("show_prompt", word); // prompt);
								cv::waitKey(1);
							}
	}
		break;
	case Qt::Key_C:{
		//datastream->clear_stream_buffer();
		int cur_class = worker->classify();
		cout << "current class is: " << cur_class << " " << worker->class_names[cur_class] << endl;
	}
		break;
	case Qt::Key_P:{
		worker->model->write_model("C:/",0);
					   //std::ofstream ofs = std::ofstream();
					  // ofs.open(base_path + "model_adj.txt", std::ofstream::out | std::ofstream::app);
					  // ofs << scale_adj << " " << width_adj << " " << thick_adj << std::endl;
					   
					   //worker->model->print_model();	   
	}
		
	break;
	case Qt::Key_1: {
		cout << "uniform scaling up" << endl;
		worker->model->resize_model(1.05, 1.0, 1.0);
		scale_adj++;
	}
	break;
	case Qt::Key_2: {
		cout << "uniform scaling down" << endl;
		worker->model->resize_model(0.95, 1.0, 1.0);
		scale_adj--;
	}
	break;
	case Qt::Key_3: {
		cout << "width scaling up" << endl;
		worker->model->resize_model(1.0, 1.05, 1.0);
		width_adj++;
	}
	break;
	case Qt::Key_4: {
		cout << "width scaling down" << endl;
		worker->model->resize_model(1.0, 0.95, 1.0);
		width_adj--;
	}
	break;
	case Qt::Key_5: {
		cout << "thickness scaling up" << endl;
		worker->model->resize_model(1.0, 1.0, 1.05);
		thick_adj++;
	}
	break;
	case Qt::Key_6: {
		cout << "thickness scaling down" << endl;
		worker->model->resize_model(1.0, 1.0, 0.95);
		thick_adj--;
	}
	break;
	case Qt::Key_9: {
		cout << "Printing Model:" << endl;
		worker->model->print_model();
	}
	break;
	case Qt::Key_Plus: {
		if (worker->joint_number < 0 || worker->joint_number == 28) {
			worker->joint_number = 9;
		}
		else {
			worker->joint_number++;			
		}
		cout << "Setting joint: " << worker->model->phalanges[worker->model->dofs[worker->joint_number].phalange_id].name;
		if (worker->model->dofs[worker->joint_number].axis.z() == 1) {
			cout << " abduction. Set to ";
		}
		else {
			cout << " flexion. Set to ";
		}
		if (worker->joint_min) {
			cout << "min: " << worker->model->dofs[worker->joint_number].min << std::endl;
		}else{
			cout << "max: " << worker->model->dofs[worker->joint_number].max << std::endl;
		}
	}
	break;
	case Qt::Key_Minus: {
		worker->joint_min = !worker->joint_min;
		cout << "Setting joint: " << worker->model->phalanges[worker->model->dofs[worker->joint_number].phalange_id].name;
		if (worker->model->dofs[worker->joint_number].axis.z() == 1) {
			cout << " abduction. Set to ";
		}
		else {
			cout << " flexion. Set to ";
		}
		if (worker->joint_min) {
			cout << "min: " << worker->model->dofs[worker->joint_number].min << std::endl;
		}
		else {
			cout << "max: " << worker->model->dofs[worker->joint_number].max << std::endl;
		}
	}
	break;
	case Qt::Key_M: {
		std::cout << "   push error: " << worker->tracking_error.push_error << " pull error: " << worker->tracking_error.pull_error << std::endl;;
	}
	break;
	case Qt::Key_T: {
		cout << "pre swap    worker->model: " << (long)worker->model << " worker->model2: " << (long)worker->model2 << endl;
		worker->swap_hands();
		cout << "swapped     worker->model: " << (long)worker->model << " worker->model2: " << (long)worker->model2 << endl;
		Model * temp = convolution_renderer.model;
		cout << "pre swap conv_rend->model: " << (long)convolution_renderer.model << " conv_rend->model2: " << (long)convolution_renderer.model2 << endl;
		convolution_renderer.model = convolution_renderer.model2;
		convolution_renderer.model2 = temp;
		cout << "swapped  conv_rend->model: " << (long)convolution_renderer.model << " conv_rend->model2: " << (long)convolution_renderer.model2 << endl;

	}
	break;
	case Qt::Key_J: {
		if (worker->joint_min) {
			worker->model->phalanges[4].init_local(0, 3) += 1;
		}
		else {
			worker->model->phalanges[4].init_local(0, 3) -= 1;
		}
	}
	case Qt::Key_K: {
		if (worker->joint_min) {
			worker->model->phalanges[4].init_local(2, 3) += 1;
		}
		else {
			worker->model->phalanges[4].init_local(2, 3) -= 1;
		}
	}
	break;
	case Qt::Key_B: {
		if (worker->joint_min) {
			for (int i = 0; i < worker->model->phalanges[0].offsets.size(); i++) {
				worker->model->phalanges[0].offsets[i](0) *= 1.05;
			}			
		}
		else {
			for (int i = 0; i < worker->model->phalanges[0].offsets.size(); i++) {
				worker->model->phalanges[0].offsets[i](0) *= .95;
			}
		}
	}
	break;
	case Qt::Key_N: {		
		for (int i = 0; i < 38; i++) {
			fitModelRadii(i);
			worker->offscreen_renderer.render_offscreen(true, false);
			worker->updateGL();
		}
		while (phalange_adj < num_phalanges) {
			fitModelPhalanges();
			worker->offscreen_renderer.render_offscreen(true, false);
			worker->updateGL();
		}
		phalange_adj = 0;
		while (phalange_adj < num_phalanges) {
			fitModelOffsets();
			worker->offscreen_renderer.render_offscreen(true, false);
			worker->updateGL();
		}
		phalange_adj = 0;

	}
	break;

	}
}


void GLWidget::fitModelRadii(int r_i) {
	bool stable = false;
	int iter_max = 10;
	int iter = 0;

	while (!stable && iter < iter_max) {
		//Get baseline error
		float orig_r = worker->model->radii[r_i];
		float averagePush = 0;
		float averagePull = 0;
		for (int i = 0; i < 10; ++i) {
			worker->track_till_convergence();
			averagePush += worker->tracking_error.push_error;
			averagePull += worker->tracking_error.pull_error;
		}
		std::cout << "   push error: " << averagePush << " pull error: " << averagePull << std::endl;;

		//Increase Radii and measure error
		worker->model->radii[r_i] = orig_r * 1.05;
		float Pushrp = 0;
		float Pullrp = 0;
		for (int i = 0; i < 10; ++i) {
			worker->track_till_convergence();
			Pushrp += worker->tracking_error.push_error;
			Pullrp += worker->tracking_error.pull_error;
		}
		std::cout << "   radius of " << r_i << " increase 5% push error: " << Pushrp << " pull error: " << Pullrp << std::endl;;

		//Decrease Radifi and measure error
		worker->model->radii[r_i] = orig_r * .95;
		float Pushrm = 0;
		float Pullrm = 0;
		for (int i = 0; i < 10; ++i) {
			worker->track_till_convergence();
			Pushrm += worker->tracking_error.push_error;
			Pullrm += worker->tracking_error.pull_error;
		}
		std::cout << "   radius of " << r_i << " decrease 5% push error: " << Pushrm << " pull error: " << Pullrm << std::endl;;

		if ((Pushrp + Pullrp) < (averagePull + averagePush)) {
			if ((Pushrm + Pullrm) < (averagePull + averagePush)) {
				std::cout << "Both Ways better. ";
				if ((Pushrp + Pullrp) < (Pushrm + Pullrm)) {
					std::cout << "Increase 5%" << std::endl;
					worker->model->radii[r_i] = orig_r * 1.05;			
				}
				else {
					std::cout << "Decrease 5%" << std::endl;
					worker->model->radii[r_i] = orig_r * 0.95;
				}
			}
			else {
				std::cout << "   Increase 5% " << std::endl;
				worker->model->radii[r_i] = orig_r * 1.05;
			}
		}
		else if ((Pushrm + Pullrm) > (averagePull + averagePush)) {
			worker->model->radii[r_i] = orig_r;
			std::cout << "   No change" << std::endl;
			stable = true;
		}
		else {
			std::cout << "Decrease 5%" << std::endl;
			worker->model->radii[r_i] = orig_r * 0.95;
		}
		iter++;
	}

}

void GLWidget::fitModelOffsets() {
	float averagePush = 0;
	float averagePull = 0;
	for (int i = 0; i < 10; ++i) {
		worker->track_till_convergence();
		averagePush += worker->tracking_error.push_error;
		averagePull += worker->tracking_error.pull_error;
	}
	std::cout << "   push error: " << averagePush << " pull error: " << averagePull << std::endl;;

	for (int j = 0; j < worker->model->phalanges[phalange_adj].offsets.size(); ++j) {
		for (int k = 0; k < 3; k++) {

			worker->model->phalanges[phalange_adj].offsets[j](k) = worker->model->phalanges[phalange_adj].offsets[j](k) + 1;
			float Pushxp1 = 0;
			float Pullxp1 = 0;
			for (int i = 0; i < 10; ++i) {
				worker->track_till_convergence();
				Pushxp1 += worker->tracking_error.push_error;
				Pullxp1 += worker->tracking_error.pull_error;
			}
			std::cout << "   Axis " << k << " +1 push error: " << Pushxp1 << " pull error: " << Pullxp1 << std::endl;;

			worker->model->phalanges[phalange_adj].offsets[j](k) = worker->model->phalanges[phalange_adj].offsets[j](k) - 2;			
			float Pushxm1 = 0;
			float Pullxm1 = 0;
			for (int i = 0; i < 10; ++i) {
				worker->track_till_convergence();
				Pushxm1 += worker->tracking_error.push_error;
				Pullxm1 += worker->tracking_error.pull_error;
			}
			std::cout << "Axis " << k << " -1 push error: " << Pushxm1 << " pull error: " << Pullxm1 << std::endl;

			if ((Pushxp1 + Pullxp1) < (averagePull + averagePush)) {
				if ((Pushxm1 + Pullxm1) < (averagePull + averagePush)) {
					std::cout << "Both Ways better. ";
					if ((Pushxp1 + Pullxp1) < (Pushxm1 + Pullxm1)) {
						std::cout << "Move axis " << k << " + 1" << std::endl;
						worker->model->phalanges[phalange_adj].offsets[j](k) = worker->model->phalanges[phalange_adj].offsets[j](k) + 2;
					}
					else {
						std::cout << "Move axis " << k << " - 1" << std::endl;
					}
				}
				else {
					std::cout << "   Move axis " << k << " + 1" << std::endl;
					worker->model->phalanges[phalange_adj].offsets[j](k) = worker->model->phalanges[phalange_adj].offsets[j](k) + 2;
				}
			}
			else if ((Pushxm1 + Pullxm1) > (averagePull + averagePush)) {
				worker->model->phalanges[phalange_adj].offsets[j](k) = worker->model->phalanges[phalange_adj].offsets[j](k) + 1;
				std::cout << "   No move" << std::endl;
				axis++;
				if (axis == 3) {
					axis = 0;
					phalange_adj++;
					std::cout << "Now adjusting: " << worker->model->phalanges[phalange_adj].name << std::endl;
				}
			}
			else {
				std::cout << "   Move axis " << k << " - 1" << std::endl;
			}
		}
	}
	phalange_adj++;

}

void GLWidget::fitModelPhalanges() {
	float averagePush = 0;
	float averagePull = 0;
	for (int i = 0; i < 10; ++i) {
		worker->track_till_convergence();
		averagePush += worker->tracking_error.push_error;
		averagePull += worker->tracking_error.pull_error;
	}
	std::cout << "   push error: " << averagePush << " pull error: " << averagePull << std::endl;;



	worker->model->phalanges[phalange_adj].init_local(axis, 3) = worker->model->phalanges[phalange_adj].init_local(axis, 3) + 1;
	float Pushxp1 = 0;
	float Pullxp1 = 0;
	for (int i = 0; i < 10; ++i) {
		worker->track_till_convergence();
		Pushxp1 += worker->tracking_error.push_error;
		Pullxp1 += worker->tracking_error.pull_error;
	}
	std::cout << "   Axis " << axis << " +1 push error: " << Pushxp1 << " pull error: " << Pullxp1 << std::endl;;
	
	worker->model->phalanges[phalange_adj].init_local(axis, 3) = worker->model->phalanges[phalange_adj].init_local(axis, 3) - 2;
	float Pushxm1 = 0;
	float Pullxm1 = 0;
	for (int i = 0; i < 10; ++i) {
		worker->track_till_convergence();
		Pushxm1 += worker->tracking_error.push_error;
		Pullxm1 += worker->tracking_error.pull_error;
	}
	std::cout << "   Axis " << axis << " -1 push error: " << Pushxm1 << " pull error: " << Pullxm1 << std::endl;

	if ((Pushxp1 + Pullxp1) < (averagePull + averagePush)) {
		if ((Pushxm1 + Pullxm1) < (averagePull + averagePush)) {
			std::cout << "   Both Ways better. ";
			if ((Pushxm1 + Pullxm1) < (Pushxp1 + Pullxp1)) {
				//keep current setting
				std::cout << "Move axis " << axis << " - 1" << std::endl;
			}
			else {
				std::cout << "Move axis " << axis << " + 1" << std::endl;
				worker->model->phalanges[phalange_adj].init_local(axis, 3) = worker->model->phalanges[phalange_adj].init_local(axis, 3) + 2;
			}
		}
		else {
			std::cout << "   Move axis " << axis << " + 1" << std::endl;
			worker->model->phalanges[phalange_adj].init_local(axis, 3) = worker->model->phalanges[phalange_adj].init_local(axis, 3) + 2;
		}
	}
	else if ((Pushxm1 + Pullxm1) > (averagePull + averagePush)) {
		worker->model->phalanges[phalange_adj].init_local(axis, 3) = worker->model->phalanges[phalange_adj].init_local(axis, 3) + 1;
		std::cout << "   No move" << std::endl;
		axis++;
		if (axis == 3) {
			axis = 0;
			phalange_adj++;
			std::cout << "Now adjusting: " << worker->model->phalanges[phalange_adj].name << std::endl;
		}
	}
	else {
		std::cout << "   Move axis " << axis << " - 1" << std::endl;
	}

}

std::string GLWidget::get_next_prompt(){
	//prompt_i++;
	//std::cout << "Prompt word: Knight" << std::endl;
	//std::strcmp(cur_set, "PalmOutFingersUp")

	std::string prompt = prompts.at(prompt_order[prompt_i++]);
	//std::cout << "prompt is: " << prompt << " prompts has " << prompts.size() << std::endl;
	
	if (prompt_i == 100 && set == 3)
	if (prompt_i == 100){
		prompts.clear();
		load_prompts(set++);
	}

	return prompt;
}

void GLWidget::load_prompts(int set){
	prompts = std::vector<std::string>();
	ifstream fp;
	if (set == 0){
		fp.open("C:/Projects/ASLRecog/hmodel/data/NamesList.txt");
	}
	if (set == 1){
		fp.open("C:/Projects/ASLRecog/hmodel/data/NounsList.txt");
	}
	if (set == 2){
		fp.open("C:/Projects/ASLRecog/hmodel/data/NonEnglishList.txt");
	}
	//FILE *fp = fopen("C:/Projects/ASLRecog/hmodel/data/NamesList.txt", "r");
	std::string word;
	int N = 100;
	for (int i = 0; i < N; ++i) {
		getline(fp, word);
		//fscanf(fp, "%s", &word);
		prompts.push_back(word);
	}
	fp.close();
	//fclose(fp);
}