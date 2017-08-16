#pragma once
#include <QGLWidget>
#include "tracker/ForwardDeclarations.h"
#include "tracker/OpenGL/KinectDataRenderer/KinectDataRenderer.h"
#include "tracker/OpenGL/ConvolutionRenderer/ConvolutionRenderer.h"
#include "apps\hmodel_atb\DataCollector.h"

class GLWidget : public QGLWidget {
public:
	Worker * worker;
	DataStream * const datastream;
	SolutionStream * const solutions;
	DataCollector* collector;

	Camera*const _camera;
	KinectDataRenderer kinect_renderer;
	ConvolutionRenderer convolution_renderer;

	bool playback;
	bool real_color;

	std::string data_path;
	std::string store_path;

	//Brandon
	//Worker * worker2 = NULL;
	bool recording;
	std::string current_prompt;
	std::vector<std::string> prompts;
	void load_prompts(int);
	int prompt_i;
	int prompt_order[100];
	int set;
	int scale_adj;
	int width_adj;
	int thick_adj;
	void fitModelPhalanges();
	void fitModelOffsets();
	void fitModelRadii(int);
	int phalange_adj = 1;
	int axis = 0;
	int offset = 0;
	bool myoEnable;

public:

	GLWidget(Worker* worker, DataCollector* collector, DataStream * datastream, SolutionStream * solutions, bool playback, bool real_color, std::string data_path, std::string store_path);

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

	std::string get_next_prompt();
};
