#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"

class OffscreenRenderer{
protected:
    Camera* camera = NULL;
public:
	Model * model;	
public:
    CustomFrameBuffer* frame_buffer = NULL; 
	ConvolutionRenderer * convolution_renderer;

	ConvolutionRenderer * convolution_renderer2;
	Model * model2;
	CustomFrameBuffer* frame_buffer2 = NULL;

	void init(Camera *camera, Model * model, std::string data_path, bool render_block_id,Handedness handedness,Model * model2);
    void init(Camera *camera, Model * model, std::string data_path, bool render_block_id);
    void render_offscreen(bool last_iter, bool fingers_only, bool reinit=false);
	void rastorize_model(cv::Mat & rastorized_model);
	~OffscreenRenderer();

	int handedness;
};

