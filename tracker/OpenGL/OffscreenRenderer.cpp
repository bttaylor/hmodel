#include "OffscreenRenderer.h"

#include "util/tictoc.h"
#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "tracker/Data/Camera.h"
#include "tracker/OpenGL/ConvolutionRenderer/ConvolutionRenderer.h"
#include "tracker/OpenGL/CustomFrameBuffer.h"

void OffscreenRenderer::init(Camera* camera, Model * model, std::string data_path, bool render_block_id) {
	cout << "OffscreenRenderer::init forcing left_hand handedness" << endl;
	init(camera, model, data_path, render_block_id, left_hand, NULL);
}

void OffscreenRenderer::init(Camera* camera, Model * model, std::string data_path, bool render_block_id, Handedness handedness, Model * model2) {
	this->camera = camera;
	this->model = model;

	this->handedness = handedness;
	if (handedness == both_hands) {
		this->model2 = model2;
		if (render_block_id) {
			frame_buffer = new CustomFrameBuffer(camera->width(), camera->height(), render_block_id);
			//frame_buffer2 = new CustomFrameBuffer(camera->width(), camera->height(), render_block_id);
			convolution_renderer = new ConvolutionRenderer(model, ConvolutionRenderer::FRAMEBUFFER, camera->view_projection_matrix(), data_path, model2);
		}
		else { // render_depth
			frame_buffer = new CustomFrameBuffer(camera->width(), camera->height(), render_block_id);
			convolution_renderer = new ConvolutionRenderer(model, ConvolutionRenderer::RASTORIZER, camera->view_projection_matrix(), data_path);
		}
	}
	else {
		//model2 = NULL;
		if (render_block_id) {
			cout << "offscreenrender::init()" << endl;
			frame_buffer = new CustomFrameBuffer(camera->width(), camera->height(), render_block_id);
			convolution_renderer = new ConvolutionRenderer(model, ConvolutionRenderer::FRAMEBUFFER, camera->view_projection_matrix(), data_path);
		}
		else { // render_depth
			frame_buffer = new CustomFrameBuffer(camera->width(), camera->height(), render_block_id);
			convolution_renderer = new ConvolutionRenderer(model, ConvolutionRenderer::RASTORIZER, camera->view_projection_matrix(), data_path);
		}
	}

	//dumb test
	//Model* model2 = new Model();
	//model2->init(10, data_path, 0);
	if(model2 != NULL)
		cout << "Model2 initialized. centers.size(): " << model2->centers.size() << endl;
	//std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	//theta_initial[1] = -10; theta_initial[2] = 400;
	//theta_initial[5] = 1.6;
	//model2->move(theta_initial);
	//model2->update_centers();
	//model2->compute_outline();
	//ConvolutionRenderer* convolution_renderer2 = new ConvolutionRenderer(model2, ConvolutionRenderer::FRAMEBUFFER, camera->view_projection_matrix(), data_path);

	//convolution_renderer->model2 = model2;
	//cout << "conv_renderer->model2->centers.size(): " << convolution_renderer->model2->centers.size() << endl;
	//end dumb test

}

OffscreenRenderer::~OffscreenRenderer() {
	delete frame_buffer;
	delete convolution_renderer;
}

void OffscreenRenderer::render_offscreen(bool last_iter, bool fingers_only, bool reinit) {
	//cout << "OffscreenRenderer model: " << (long)model << endl;
	glViewport(0, 0, camera->width(), camera->height());
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glDisable(GL_BLEND); ///< just in case
	glEnable(GL_DEPTH_TEST);

	//frame_buffer->display_color_attachment();

	frame_buffer->bind(true);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	convolution_renderer->render_offscreen(fingers_only);

	//dumb test
	//std::vector<float> theta = std::vector<float>(num_thetas, 0);
	//std::vector<float> theta = convolution_renderer->model->get_theta();
	//convolution_renderer->model->move(theta);
	//convolution_renderer->model->update_centers();
	//convolution_renderer->render_offscreen(false);
	//convolution_renderer2->render_offscreen(fingers_only);
	//end dumb test
	frame_buffer->unbind();

	if (last_iter) frame_buffer->fetch_color_attachment(model->silhouette_texture);   //changed to model2

	//frame_buffer->display_color_attachment();
	//frame_buffer->display_depth_attachment();

	glFinish();
}

void OffscreenRenderer::rastorize_model(cv::Mat & rastorized_model) {

	glViewport(0, 0, camera->width(), camera->height());
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glDisable(GL_BLEND); ///< just in case
	glEnable(GL_DEPTH_TEST);

	frame_buffer->bind(false);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	convolution_renderer->render_offscreen(false);
	frame_buffer->unbind();

	//frame_buffer->display_depth_attachment();	
	frame_buffer->fetch_depth_attachment(rastorized_model);
	cv::flip(rastorized_model, rastorized_model, 0);
	//frame_buffer->fetch_normals_attachment(rastorized_normals);
	//cv::flip(rastorized_normals, rastorized_normals, 0);

	glFinish();	
}

