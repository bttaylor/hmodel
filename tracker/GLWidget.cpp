
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

GLWidget::GLWidget(Worker* worker, DataStream * datastream, SolutionStream * solutions, bool playback, bool real_color, std::string data_path, DataCollector* collector, std::string store_path) :
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
	this->move(1250, 375);
	convolution_renderer.window_width = this->width();
	convolution_renderer.window_height = this->height();

	std::cout << "Started OpenGL " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;
	this->installEventFilter(new AntTweakBarEventFilter(this)); ///< all actions pass through filter
	set = 0;
	load_prompts(set++);
	recording = false;
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
}

void GLWidget::paintGL() {
	glViewport(0, 0, this->width(), this->height());
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	///--- Rendering
	Eigen::Matrix4f view_projection = _camera->view_projection_matrix() * view;
	if (worker->get_active_handfinder()->wristband_found()) {
		kinect_renderer.enable_colormap(true);
		kinect_renderer.set_zNear(worker->get_active_handfinder()->wristband_center()[2] - 150);
		kinect_renderer.set_zFar(worker->get_active_handfinder()->wristband_center()[2] + 150);
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
	case Qt::Key_C: {
		int hs_class = worker->classify();
		cout << "Most likely class: " << worker->class_names[hs_class] << std::endl;
	}
	case Qt::Key_S: {
		cout << "set up path for saving images" << std::endl;
		//datastream->save_as_images(data_path);
	}
	case Qt::Key_Q: {
		worker->get_active_model()->write_model("C:/Projects/Data/Participant0/",0);
	}
	break;
	case Qt::Key_Space: {
		if (prompt_i == 0) {
			current_prompt = get_next_prompt();
			std::cout << "Loading prompt " << prompt_i << ": " << current_prompt << std::endl;
			cv::Mat word = cv::Mat::ones(100, 500, CV_8UC3);
			word = cv::Scalar(255, 255, 255);
			cv::putText(word, current_prompt, cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 2, 0);
			cv::imshow("show_prompt", word); // prompt);
			cv::waitKey(1);

			this->activateWindow();
		}
		else {
			if (!recording) {
				std::cout << "Recording..." << std::endl;
				if (collector->enabled) {
					collector->clear_buffers();
					collector->recording = true;
				}
				datastream->clear_stream_buffer();
				recording = true;
			}
			else {
				std::cout << "End Recording" << std::endl;
				recording = false;
				std::cout << "Saving images to: " << store_path + current_prompt << endl;
				datastream->save_as_images(store_path + current_prompt);
				if (collector->enabled) {
					collector->recording = false;
					collector->saveMyoData(store_path + current_prompt);
				}

				current_prompt = get_next_prompt();
				std::cout << "Loading prompt " << prompt_i << ": " << current_prompt << std::endl;
				cv::Mat word = cv::Mat::ones(100, 500, CV_8UC3);
				word = cv::Scalar(255, 255, 255);
				cv::putText(word, current_prompt, cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 2, 0);
				cv::imshow("show_prompt", word); // prompt);
				cv::waitKey(1);
			}
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

	std::string prompt = prompts.at(prompt_order[prompt_i++]);
	//std::cout << "prompt is: " << prompt << " prompts has " << prompts.size() << std::endl;

	if (prompt_i == 100 && set == 3)
		if (prompt_i == 100) {
			prompts.clear();
			load_prompts(set++);
		}

	return prompt;
}

void GLWidget::load_prompts(int set) {
	prompts = std::vector<std::string>();
	ifstream fp;
	if (set == 0) {
		fp.open(data_path + "prompts/NamesList.txt");
	}
	if (set == 1) {
		fp.open(data_path + "prompts/NounsList.txt");
	}
	if (set == 2) {
		fp.open(data_path + "prompts/NonEnglishList.txt");
	}
	
	std::string word;
	int N = 100;
	for (int i = 0; i < N; ++i) {
		getline(fp, word);
		prompts.push_back(word);
	}
	fp.close();

	prompt_i = 0;
	for (int i = 0; i < 100; i++) {
		prompt_order[i] = i;
	}
	std::random_shuffle(std::begin(prompt_order), std::end(prompt_order));
}