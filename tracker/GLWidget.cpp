
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
//#include "HTracker.h"

#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"

#define M_PI 3.14159265358979323846

GLWidget::GLWidget(Worker* worker, DataStream * datastream, SolutionStream * solutions, bool playback, bool real_color, std::string data_path, DataCollector* collector, std::string store_path, HTracker* _tracker) :
QGLWidget(OpenGL32Format()),
worker(worker),
datastream(datastream),
solutions(solutions),
_camera(worker->camera),
collector(collector),
convolution_renderer(worker, real_color, data_path) {
	this->playback = playback;
	this->data_path = data_path;
	this->store_path = store_path;
	this->resize(640 * 2, 480 * 2);
	this->move(250, 175);
	convolution_renderer.window_width = this->width();
	convolution_renderer.window_height = this->height();

	std::cout << "Started OpenGL " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;
	this->installEventFilter(new AntTweakBarEventFilter(this)); ///< all actions pass through filter
	set = 0;
	load_prompts(set++);
	recording = false;
	current_model = std::vector<Vec3d>(38);
	for (int i = 0; i < 5; i++) {
		finger_lock[i] = false;
	}
	joint = 9;
	this->tracker = _tracker;
	for (int i = 0; i < 5; i++) {
		finger_lock[i] = false;
	}
}

GLWidget::~GLWidget() {
	worker->cleanup_graphic_resources();
	tw_settings->tw_cleanup();
}

void GLWidget::initializeGL() {
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

	show_hand_render = false;
}

void GLWidget::paintGL() {
	glViewport(0, 0, this->width(), this->height());
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	///--- Rendering
	Eigen::Matrix4f view_projection = _camera->view_projection_matrix() * view;
	if (worker->get_active_handfinder()->wristband_found() && color_known) {
		kinect_renderer.enable_colormap(true);		
		kinect_renderer.set_zNear(worker->get_active_handfinder()->wristband_center()[2] - 250); //200?  was 150
		kinect_renderer.set_zFar(worker->get_active_handfinder()->wristband_center()[2] + 50); //was 150
	}
	else {		
		kinect_renderer.enable_colormap(true);
		kinect_renderer.set_zNear(400 - 250); //200?  was 150
		kinect_renderer.set_zFar(400 + 50); //was 150
	}
	kinect_renderer.set_uniform("view_projection", view_projection);
	kinect_renderer.render();

	glDisable(GL_BLEND);
	if(show_hand_render)
		convolution_renderer.render();

	//worker->model->render_outline();
	//DebugRenderer::instance().set_uniform("view_projection", view_projection);
	//DebugRenderer::instance().render();

	//tw_settings->tw_draw();
}

void GLWidget::process_mouse_movement(GLfloat cursor_x, GLfloat cursor_y) {
	glm::vec3 image_center_glm = worker->get_active_model()->centers[worker->get_active_model()->centers_name_to_id_map["palm_back"]] +
		worker->get_active_model()->centers[worker->get_active_model()->centers_name_to_id_map["palm_middle"]];
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
	case Qt::Key_I: {
		joint++;
		if (joint > 28) {
			joint = 9;
		}
		cout << "Selected Joint: " << joint << endl;
	}
					break;
	case Qt::Key_O: {
		vector<float> theta = worker->get_active_model()->get_theta();
		theta[joint] = worker->get_active_model()->dofs[joint].max;
		cout << " Joint set to max: " << theta[joint] << endl;
		worker->get_active_model()->move(theta);
		worker->get_active_model()->update_centers();
		worker->offscreen_renderer.render_offscreen(true, false);

		updateGL();
	}
					break;
	case Qt::Key_P: {
		vector<float> theta = worker->get_active_model()->get_theta();
		theta[joint] = worker->get_active_model()->dofs[joint].min;
		cout << " Joint set to min: " << theta[joint] << endl;
		worker->get_active_model()->move(theta);
		worker->get_active_model()->update_centers();
		worker->offscreen_renderer.render_offscreen(true, false);

		updateGL();

	}
					break;
	case Qt::Key_A: {
		worker->frame_advance = true;
		width_lock = false;
		length_lock = false;
		for (int i = 0; i < 5; i++) {
			finger_lock[i] = false;
		}
	}
					break;
	case Qt::Key_B: {
		std::vector<float> theta = worker->get_active_model()->get_theta();
		for (int i = 3; i < num_thetas; i++) {
			theta[i] = 0;
		}
		worker->get_active_model()->move(theta);
		worker->get_active_model()->update_centers();
		updateGL();
	}
					break;
	case Qt::Key_D: {
		worker->stop_process = true;
	}
					break;
	case Qt::Key_Z: {
		std::vector<float> theta = worker->get_active_model()->get_theta();
		cout << "wrist abduction was: " << theta[7] << " Limit: " << worker->get_active_model()->dofs[7].min << " - " << worker->get_active_model()->dofs[7].max << endl;
		theta[7] += .1;
		cout << "wrist abduction now: " << theta[7] << endl;
		worker->get_active_model()->move(theta);
		worker->get_active_model()->update_centers();
	}
					break;
	case Qt::Key_C: {
		int hs_class = worker->classify();
		cout << "Most likely class: " << worker->class_names[hs_class] << std::endl;
	}
					break;
	case Qt::Key_X: {
		cout << "Checking Hand Width" << endl;
//		adjust_hand(false, .1);
		cout << "Checking Hand Length" << endl;
//		adjust_hand(true, .1);
		cout << "Checking Finger Lengths" << endl;
//		adjust_finger_lengths(.1);


		//worker->get_active_model()->adjust_finger(1, true);
		worker->get_active_model()->update_centers();
		//cout << "average error longer: " << average_error() << endl;

	}
					break;
	case Qt::Key_V: {
		//adjust_hand(true, .1);
		//worker->get_active_model()->adjust_hand_length(.1);
		//worker->get_active_model()->adjust_finger(1, false);
		worker->get_active_model()->update_centers();
		//cout << "average error shorter: " << average_error() << endl;
		//reset_hand_model();
	}
	break;
	case Qt::Key_E: {
		save_current_hand_model();
		cout << "average error: " << average_error() << endl;
	}
	break;
	case Qt::Key_H: {
		if (show_hand_render)
			show_hand_render = false;
		else
			show_hand_render = true;
	}
	break;
	case Qt::Key_U:{
		calibration_frame = worker->current_frame.id;
		show_hand_render = true;
		cout << "Frame num is: " << calibration_frame << endl;
		dummy();
		width_lock = false;
		length_lock = false;
		finger_lock[0] =false;
		finger_lock[1] = false;
		finger_lock[2] = false;
		finger_lock[3] = false;
		finger_lock[4] = false;
		tracker->mode = LIVE;
	}
	 break;
	case Qt::Key_J: {
		bool found = worker->get_active_handfinder()->find_wristband_color(worker->current_frame.depth, worker->current_frame.color);
		color_known = found;
		//worker->sensor->found_color(found);
		worker->sensor->handfinder->_settings.hsv_min[0] = worker->get_active_handfinder()->settings->hsv_min[0];
		worker->sensor->handfinder->_settings.hsv_min[1] = worker->get_active_handfinder()->settings->hsv_min[1];
		worker->sensor->handfinder->_settings.hsv_min[2] = worker->get_active_handfinder()->settings->hsv_min[2];
		worker->sensor->handfinder->_settings.hsv_max[0] = worker->get_active_handfinder()->settings->hsv_max[0];
		worker->sensor->handfinder->_settings.hsv_max[1] = worker->get_active_handfinder()->settings->hsv_max[1];
		worker->sensor->handfinder->_settings.hsv_max[2] = worker->get_active_handfinder()->settings->hsv_max[2];
		worker->sensor->handfinder->_settings.show_wband = true;
		cout << worker->get_active_handfinder()->settings->hsv_max[0] << " " << worker->get_active_handfinder()->settings->hsv_max[1] << " " << worker->get_active_handfinder()->settings->hsv_max[2] << endl;
		//tracker->toggle_tracking(true);
	}
	break;
	case Qt::Key_S: {
		cout << "set up path for saving images" << std::endl;
		std::string save_path = store_path + "Calibrate";
		
		datastream->save_as_images(save_path);
		if (collector->enabled) {
			collector->recording = false;
			collector->saveMyoData(save_path);
		}
	}
	break;
	case Qt::Key_Q: {
		if (worker->user_name < 10) {

		}
		worker->get_active_model()->write_model(data_path, 0);
		//worker->data_path + "models/brandon/" + ;
		//worker->get_active_model()->write_model("C:/Projects/Data/Participant0/",0);
	}
	break;
	case Qt::Key_Left: {
		if (!recording) {
			current_prompt = get_prev_prompt();
			std::cout << "Redoing prompt " << prompt_i << ": " << current_prompt << std::endl;
			cv::Mat word = cv::Mat::ones(100, 500, CV_8UC3);
			word = cv::Scalar(255, 255, 255);
			cv::putText(word, current_prompt, cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 2, 0);
			cv::imshow("show_prompt", word); // prompt);
			cv::waitKey(1);
		}
		else {
			std::cout << "Saving... ";
			recording = false;
			save_data();

			std::cout << "Redoing prompt " << prompt_i << ": " << current_prompt << std::endl;

			display_prompt(current_prompt);
		}
		this->activateWindow();
	}
	break;
	case Qt::Key_Space: {
		if (prompt_i == 0) {
			current_prompt = get_next_prompt();

			std::cout << "Loading prompt " << prompt_i << ": " << current_prompt << std::endl;
			display_prompt(current_prompt);

			this->activateWindow();
		}
		else {
			if (!recording) {
				std::cout << "Recording... " ;
				if (collector->enabled) {
					collector->clear_buffers();
					collector->recording = true;
				}
				datastream->clear_stream_buffer();
				worker->clear_tracking_data();
				recording = true;
			}
			else {
				std::cout << "Saving... " ;
				recording = false;

				save_data();

				current_prompt = get_next_prompt();
				std::cout << "Loading prompt " << prompt_i << ": " << current_prompt << std::endl;

				display_prompt(current_prompt);
			}
		}
	}
	break;
	case Qt::Key_W: {
		while (set < 4) {
			cout << get_next_prompt() << endl;
		}
	}
	break;
	case Qt::Key_1: {
		cout << "uniform scaling up" << endl;
		worker->get_active_model()->resize_model(1.05, 1.0, 1.0);
	}
	break;
	case Qt::Key_2: {
		cout << "uniform scaling down" << endl;
		worker->get_active_model()->resize_model(0.95, 1.0, 1.0);
	}
	break;
	case Qt::Key_3: {
		cout << "width scaling up" << endl;
		worker->get_active_model()->resize_model(1.0, 1.05, 1.0);
	}
	break;
	case Qt::Key_4: {
		cout << "width scaling down" << endl;
		worker->get_active_model()->resize_model(1.0, 0.95, 1.0);
	}
	break;
	case Qt::Key_5: {
		cout << "thickness scaling up" << endl;
		worker->get_active_model()->resize_model(1.0, 1.0, 1.05);
	}
	break;
	case Qt::Key_6: {
		cout << "thickness scaling down" << endl;
		worker->get_active_model()->resize_model(1.0, 1.0, 0.95);
	}
	break;
	}
}


std::string GLWidget::get_next_prompt() {
	
	if (prompt_i == prompts.size()) {
		load_prompts(set++);
	}
	std::string prompt = prompts.at(prompt_order[prompt_i++]);

	/*
	if (prompt_i == 100 && set == 3)
		if (prompt_i == 100) {
			prompts.clear();
			load_prompts(set++);
		}
	*/
	return prompt;
}

std::string GLWidget::get_prev_prompt() {
	if (prompt_i == 1) {
		//Already at beginning
		return current_prompt;
	}
	prompt_i -= 1;
	std::string prompt = prompts.at(prompt_order[prompt_i - 1]);
	return prompt;
}

void GLWidget::load_prompts(int set) {
	prompts = std::vector<std::string>();
	ifstream fp;
	//for handshape
	cout << "Loading ABC" << endl;
	fp.open(data_path + "prompts/abcList.txt");
	/*
	if (set == 0) {
		cout << "Loading NamesList" << endl;
		fp.open(data_path + "prompts/NamesList.txt");
	}
	if (set == 1) {
		cout << endl << "Loading NounsList" << endl << endl;;
		fp.open(data_path + "prompts/NounsList.txt");
	}
	if (set == 2) {
		cout << endl << "Loading NonEnglishList" << endl << endl;
		fp.open(data_path + "prompts/NonEnglishList.txt");
	}
	if (set == 3) {
		cout << "No more sets to load";
		return;
	}*/
	
	std::string word;
	int N = 27;  //mod
	for (int i = 0; i < N; ++i) {
		getline(fp, word);
		prompts.push_back(word);
	}
	fp.close();

	prompt_i = 0;
	for (int i = 0; i < 27; i++) {  //mod from 100 to 33
		prompt_order[i] = i;
	}
	//std::random_shuffle(std::begin(prompt_order), std::end(prompt_order));
}

void GLWidget::save_data() {
	int extension = 1;
	std::string set_path = store_path + "Set" + std::to_string(set_num);
	if (prompt_i == 1) {
		//First in set
		while (set_num < 9) {
			if (GetFileAttributesA(set_path.c_str()) == INVALID_FILE_ATTRIBUTES) {
				cout << "Set" << std::to_string(set_num) << " did not exist. Creating " << set_path << endl;
				CreateDirectory(set_path.c_str(), NULL);
				break;
			}
			else {
				cout << "Set" << std::to_string(set_num) << " already existed? " << endl;
				set_path = store_path + "Set" + std::to_string(++set_num);
			}
		}
	}
	std::string save_path = set_path + "/" + current_prompt;
	CreateDirectory(save_path.c_str(), NULL);

	std::string checkFile(save_path + "/" + current_prompt + "_" + "imageTimestamps.csv");
	/*ifstream ftest(checkFile.c_str());
	if (!ftest.fail()) { //If file exists
		while (!ftest.fail()) { //While file exists
			extension++;
			ftest.close();
			checkFile = save_path + "_" + std::to_string(extension) + "_" + "imageTimestamps.csv";
			ftest = ifstream(checkFile.c_str());
		}
		ftest.close();
	}*/
	worker->save_tracking_data(save_path + "/" + current_prompt + "_");

	//save_path = save_path + "_" + std::to_string(extension) + "_";
	datastream->save_as_images(save_path + "/" + current_prompt + "_");
	if (collector->enabled) {
		collector->recording = false;
		collector->saveMyoData(save_path);
	}
}

void GLWidget::display_prompt(std::string prompt) {

	cv::Mat word = cv::Mat::ones(100, 500, CV_8UC3);
	word = cv::Scalar(255, 255, 255);
	cv::putText(word, prompt, cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 2, 0);
	cv::imshow("show_prompt", word); // prompt);
	cv::waitKey(1);
}


float GLWidget::average_error() {
	float averagePush = 0;
	float averagePull = 0;
	for (int i = 0; i < 5; ++i) {
		worker->track_till_convergence();
		averagePush += worker->tracking_error.push_error;
		averagePull += worker->tracking_error.pull_error;
	}
	//cout << "   push error: " << averagePush << " pull error: " << averagePull << std::endl;
	return averagePull + averagePush;
}

void GLWidget::reset_hand_model() {
	for (int i = 0; i < num_phalanges; i++) {
		worker->get_active_model()->phalanges[i].init_local(0, 3) = (float)current_model[worker->get_active_model()->phalanges[i].center_id][0];
		worker->get_active_model()->phalanges[i].init_local(1, 3) = (float)current_model[worker->get_active_model()->phalanges[i].center_id][1];
		worker->get_active_model()->phalanges[i].init_local(2, 3) = (float)current_model[worker->get_active_model()->phalanges[i].center_id][2];
		for (int j = 0; j < worker->get_active_model()->phalanges[i].attachments.size(); j++) {
			worker->get_active_model()->phalanges[i].offsets[j] = current_model[worker->get_active_model()->phalanges[i].attachments[j]];
		}
	}
}

void GLWidget::save_current_hand_model() {
	for (int i = 0; i < num_phalanges; i++) {
		current_model[worker->get_active_model()->phalanges[i].center_id][0] = worker->get_active_model()->phalanges[i].init_local(0, 3);
		current_model[worker->get_active_model()->phalanges[i].center_id][1] = worker->get_active_model()->phalanges[i].init_local(1, 3);
		current_model[worker->get_active_model()->phalanges[i].center_id][2] = worker->get_active_model()->phalanges[i].init_local(2, 3);
		for (int j = 0; j < worker->get_active_model()->phalanges[i].attachments.size(); j++) {
			current_model[worker->get_active_model()->phalanges[i].attachments[j]] = worker->get_active_model()->phalanges[i].offsets[j];
		}
	}
}
/*
void GLWidget::adjust_finger_lengths(float scale) {
	adjust_finger_length(0,scale);
	adjust_finger_length(1, scale);
	adjust_finger_length(2, scale);
	adjust_finger_length(3, scale);
	adjust_finger_length(4, scale);
}

void GLWidget::adjust_finger_length(int finger, float scale) {
	//Force straight joints
	std::vector<float> thetas = worker->get_active_model()->get_theta();
	thetas[11] = 0; thetas[12] = 0; //thumb straight
	thetas[15] = 0; thetas[16] = 0; //index
	thetas[19] = 0; thetas[20] = 0; //middle
	thetas[23] = 0; thetas[24] = 0; //ring
	thetas[27] = 0; thetas[28] = 0; //pinky
	worker->get_active_model()->move(thetas);
	worker->get_active_model()->update_centers();

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
		worker->get_active_model()->adjust_finger(finger,1 - scale);
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
	}
}

void GLWidget::adjust_hand(bool length, float scale) {
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
	}
}*/

void GLWidget::calibrate(int count) {
	std::vector<float> thetas = worker->get_active_model()->get_theta();
	for (int i = 3; i < num_thetas; i++) {
		thetas[i] = 0;
	}
	worker->get_active_model()->move(thetas);
	worker->get_active_model()->update_centers();
	worker->track_till_convergence();

	float scale = 0;
	if (count >= 1 && count < 3) {
		scale = .1;
		//length_lock = true;
	}
	if (count >= 3 && count < 5)
		scale = .05;

	if ((count >= 3 && count < 5)) {
		if (!width_lock)
			adjust_hand(false, scale);
		if (!length_lock)
			adjust_hand(true, scale);
		if (!finger_lock[0] || !finger_lock[1] || !finger_lock[2] || !finger_lock[3] || !finger_lock[4])
			adjust_finger_lengths(scale,count);
	}
}


void GLWidget::adjust_finger_lengths(float scale, int count) {
	cout << "Checking Finger Lengths" << endl;
	adjust_finger_length(0, scale, count);
	adjust_finger_length(1, scale, count);
	adjust_finger_length(2, scale, count);
	adjust_finger_length(3, scale, count);
	adjust_finger_length(4, scale, count);
}

void GLWidget::adjust_finger_length(int finger, float scale, int count) {
	//Force straight joints
	if (count < 4) {
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

void GLWidget::adjust_hand(bool length, float scale) {
	if (length)
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


void GLWidget::dummy(){
	tracker->mode = SENSOR_ONLY;
	for (int i = 1; i < 6; i++) {
		int index = calibration_frame - 5 + i;
		worker->get_active_handfinder()->binary_classification(datastream->frames.at(index)->depth, datastream->frames.at(index)->color);

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

		//worker->current_frame.id = frame_offset;
		//worker->current_frame.id = ++frame_offset;		
		worker->sensor_depth_texture->load(datastream->frames.at(index)->depth.data, index);
		//cv::imshow("sensor_silhouette", worker->get_active_handfinder()->sensor_silhouette);
		worker->track_till_convergence();
		worker->offscreen_renderer.render_offscreen(true, false);
		worker->updateGL();
		calibrate(i);
	}
}