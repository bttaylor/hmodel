#include "Fitting.h"

#include "util/mylogger.h"
#include "util/tictoc.h"

#include "tracker/Worker.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HModel/Model.h"
#include "tracker/TwSettings.h"


#ifdef WITH_CUDA
#include "cudax/CudaHelper.h"
#include "cudax/CublasHelper.h"

#include <cuda_gl_interop.h>
struct MappedResource {
	struct cudaGraphicsResource* resouce = NULL;
	cudaArray* array = NULL;
	GLuint texid = 0;

	void init(GLuint texid) {
		this->texid = texid;
		checkCudaErrors(cudaGraphicsGLRegisterImage(&resouce, texid, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly));
	}
	void cleanup() {
		checkCudaErrors(cudaGraphicsUnregisterResource(resouce));
	}

	cudaArray* bind() {
		checkCudaErrors(cudaGraphicsMapResources(1, &resouce, 0));
		checkCudaErrors(cudaGraphicsSubResourceGetMappedArray(&array, resouce, 0, 0));
		return array;
	}
	void unbind() {
		checkCudaErrors(cudaGraphicsUnmapResources(1, &resouce, 0));
	}
};

MappedResource sensor_depth;
//DistanceTransform distance_transform;

void energy::Fitting::cleanup() {
	kernel_cleanup(); ///< disposes of static resources
	sensor_depth.cleanup();
	distance_transform.cleanup();

	cudax::CublasHelper::cleanup();
	cudax::CudaHelper::cleanup();
}

void energy::Fitting::dumbInit(Worker *worker) {
	this->camera = worker->camera;
	this->sensor_depth_texture = worker->sensor_depth_texture;
	this->handfinder = worker->handfinder;
	this->model = worker->model;

	///--- Init worker for GPU computation of normals
	distance_transform.init(camera->width(), camera->height());

	///--- init resource mapper for cuda
	sensor_depth.init(sensor_depth_texture->texid());

}

void energy::Fitting::swapHands() {
	if (model2 == NULL) {
		cout << "mod2 is nUll" << endl;
	}
	else {
		//cout << "mod2 is NOT nUll" << endl;
		Model * modTemp = model;
		model = model2;
		model2 = modTemp;
	}

	if (handfinder2 == NULL) {
		cout << "haf2 is null" << endl;
	}else{
		//cout << "hf2 is NOT nUll" << endl;
		HandFinder* hfTemp = handfinder;
		handfinder = handfinder2;
		handfinder2 = hfTemp;
	}
	swapped = true;
}

void energy::Fitting::init(Worker *worker) {
	this->camera = worker->camera;
	this->sensor_depth_texture = worker->sensor_depth_texture;
	this->handfinder = worker->handfinder;
	this->model = worker->model;

	//cout << "In E_fitting.init. ";
	//if (!(worker->handfinder == NULL)) cout << "handfinder not NULL" << endl;
	//if (!(worker->handfinder2 == NULL)) cout << "handfinder2 not NULL" << endl;

	if (worker->model2 != NULL) {
		this->model2 = worker->model2;
		this->handfinder2 = worker->handfinder2;
	}

	///--- 3D fitting
	tw_settings->tw_add(settings->fit3D_enable, "E_3D (enable)", "group=Fitting");
	tw_settings->tw_add(settings->fit3D_weight, "E_3D (weight)", "group=Fitting");
	tw_settings->tw_add(settings->fit3D_reweight, "E_3D (l1nrm?)", "group=Fitting");
	tw_settings->tw_add(settings->fit3D_point2plane, "E_3D (p2P?)", "group=Fitting");
	///--- 2D fitting
	tw_settings->tw_add(settings->fit2D_enable, "E_2D (enable)", "group=Fitting");
	tw_settings->tw_add(settings->fit2D_weight, "E_2D (weight)", "group=Fitting");

	cudax::CudaHelper::init();
	cudax::CublasHelper::init();

	///--- Run some tests before we get started
	kernel_memory_tests();

	///--- Init worker for GPU computation of normals
	distance_transform.init(camera->width(), camera->height());

	///--- init resource mapper for cuda
	sensor_depth.init(sensor_depth_texture->texid());

	kernel_init(this->settings, camera->width(), camera->height(), num_thetas, camera->focal_length_x(), camera->focal_length_y(), camera->inv_projection_matrix().data(),
		d, model->centers.size(), model->blocks.size(), model->max_num_outlines, model->num_tangent_fields, model->num_outline_fields, false, worker->test, worker->model->model_type);
	CHECK_ERROR_GL();

}

void energy::Fitting::track(DataFrame& frame, LinearSystem& sys, bool rigid_only, bool eval_error, float & push_error, float & pull_error, int iter, int mod_num) {
	cout << "engergy::Fitting::track w/ mod_num" << endl;
	if (mod_num == 2) {
		Model* temp = model;
		model = model2;
		HandFinder * temph = handfinder;
		handfinder = handfinder2;

		track(frame, sys, rigid_only, eval_error, push_error, pull_error, iter);
		model2 = model;
		model = temp;
		handfinder2 = handfinder;
		handfinder = temph;

	}
	else {
		track(frame, sys, rigid_only, eval_error, push_error, pull_error, iter);
	}
}

void energy::Fitting::track(DataFrame& frame, LinearSystem& sys, bool rigid_only, bool eval_error, float & push_error, float & pull_error, int iter) {
	///--- Make sure sensor has necessary data
	assert(sensor_depth_texture->check_loaded(frame.id));

	// Upload hand model
	kernel_upload_kinematic(model->transformations, model->kinematic_chain);
	kernel_upload_model(d, model->centers.size(), model->blocks.size(), model->outline_finder.outline3D.size(), model->num_tangent_fields, model->num_outline_fields,
		model->host_pointer_centers, model->host_pointer_radii, model->host_pointer_blocks,
		model->host_pointer_tangent_points, model->host_pointer_outline, model->host_pointer_blockid_to_jointid_map);


	cv::Mat& sensor_silhouette = handfinder->sensor_silhouette;
	if (sensor_silhouette.empty()) {
		cout << "empty sensor_silhouette in energy::fitting::track" << endl;
		return;
	}
	//else
		//cv::imshow("butt", sensor_silhouette);
	static int last_uploaded_id = -1; 
	static cv::Mat sensor_silhouette_flipped;

	// Compute distance transform
	if (last_uploaded_id != frame.id || swapped) {  //Added swapped to insure distance transform is recalculated for alternate hand
		swapped = false;	

		kernel_upload_sensor_indicator(handfinder->sensor_indicator, handfinder->num_sensor_points);

		if (settings->fit2D_enable) {
			cv::flip(sensor_silhouette, sensor_silhouette_flipped, 0 /*flip rows*/);
			distance_transform.exec(sensor_silhouette_flipped.data, 125);
			kernel_upload_dtform_idxs(distance_transform.idxs_image_ptr());
			//cv::Mat idx = distance_transform.idxs_image();
			//cv::imshow("idx", idx);
			//cv::Mat dsts = distance_transform.dsts_image();
			//cv::imshow("dfd", dsts);
			//cv::waitKey(1000);
		}
		last_uploaded_id = frame.id;
	}

	// Compute rendered outline
	if (settings->fit2D_enable) {
		model->compute_rendered_indicator(handfinder->sensor_silhouette, camera);
		kernel_upload_rendered_indicator(model->rendered_pixels, model->rendered_points, model->rendered_block_ids, model->num_rendered_points);
	}

	// Run CUDA
	cudax::sensor_depth = sensor_depth.bind();
	kernel_bind();

	bool reweight = settings->fit3D_reweight;
	if (rigid_only && settings->fit3D_reweight && !(settings->fit3D_reweight_rigid)) {
		reweight = false; ///< allows fast rigid motion (mostly visible on PrimeSense @60FPS)
	}

	kernel(sys.lhs.data(), sys.rhs.data(), push_error, pull_error, eval_error, reweight, frame.id, iter,
		handfinder->num_sensor_points, model->num_rendered_points);

	kernel_unbind();
	sensor_depth.unbind();

}

energy::Fitting::~Fitting() {
	kernel_delete();
}

#endif
