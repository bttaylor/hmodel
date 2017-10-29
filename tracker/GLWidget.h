#pragma once
#include <QGLWidget>
#include "tracker/ForwardDeclarations.h"
#include "tracker/OpenGL/KinectDataRenderer/KinectDataRenderer.h"
#include "tracker/OpenGL/ConvolutionRenderer/ConvolutionRenderer.h"
#include "apps/hmodel_atb/DataCollector.h"
#include "tracker/HTracker.h"

class GLWidget : public QGLWidget {
public:
	Worker * worker;
	DataStream * const datastream;
	SolutionStream * const solutions;

	Camera*const _camera;
	KinectDataRenderer kinect_renderer;
	ConvolutionRenderer convolution_renderer;

	bool playback;
	bool real_color;

	std::string data_path;

	//Brandon
	HTracker* tracker;
	std::string store_path;
	bool recording;
	DataCollector* collector;
	int prompt_i;
	int prompt_order[100];
	int set;
	std::string current_prompt;
	std::vector<std::string> prompts;
	bool show_hand_render = true;
	int joint;
	bool color_known = false;
	bool width_lock = false;
	bool length_lock = false;
	bool finger_lock[5];
	int calibration_frame;
	int set_num = 1;

	std::vector<Vec3d> current_model;
//	bool width_lock = false;
//	bool length_lock = false;
//	bool finger_lock[5];

public:

	GLWidget(Worker* worker, DataStream * datastream, SolutionStream * solutions, bool playback, bool real_color, std::string data_path, DataCollector* collector, std::string store_path, HTracker* tracker);

	~GLWidget();

	void initializeGL();

	void paintGL();

private:
	Eigen::Vector3f camera_center = Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f image_center = Eigen::Vector3f(0, 0, 400);
	Eigen::Vector3f camera_up = Eigen::Vector3f(0, 1, 0);
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	bool mouse_button_pressed = true;
	Eigen::Vector2f cursor_position = Eigen::Vector2f(640, 480);
	Eigen::Vector2f euler_angles = Eigen::Vector2f(-6.411, -1.8);
	Eigen::Vector2f initial_euler_angles = Eigen::Vector2f(-6.411, -1.8);
	float cursor_sensitivity = 0.003f;

	void process_mouse_movement(GLfloat cursor_x, GLfloat cursor_y);

	void process_mouse_button_pressed(GLfloat cursor_x, GLfloat cursor_y);

	void process_mouse_button_released();
	
	void mouseMoveEvent(QMouseEvent *event);

	void mousePressEvent(QMouseEvent *event);

	void wheelEvent(QWheelEvent * event);

	void keyPressEvent(QKeyEvent *event);

	//Brandon
	std::string get_next_prompt();
	std::string get_prev_prompt();
	void load_prompts(int set);
	void save_data();
	void display_prompt(std::string);
	float average_error();
	void reset_hand_model();
	void save_current_hand_model();
	//void adjust_hand(bool length, float scale);
	//void adjust_finger_lengths(float);
	//void adjust_finger_length(int,float);
	void dummy();
	void adjust_hand(bool length, float scale);
	void adjust_finger_length(int finger, float scale, int count);
	void adjust_finger_lengths(float scale, int count);
	void calibrate(int count);
};
