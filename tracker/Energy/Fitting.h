#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "tracker/Energy/Fitting/Settings.h"
#include "Fitting/DistanceTransform.h"
struct MappedResource;

namespace energy{
class Fitting {
protected:
    Camera* camera = NULL;
    DepthTexture16UC1* sensor_depth_texture = NULL;
public:
    HandFinder* handfinder = NULL;
	Model * model = NULL;
	//Brnadon Mod
	Model * model2 = NULL;
	HandFinder* handfinder2 = NULL;
	bool swapped = false;

public:
    fitting::Settings _settings;
    fitting::Settings*const settings = &_settings;
	DistanceTransform distance_transform;

public:
#ifdef WITH_CUDA
	void track(DataFrame &frame, LinearSystem &sys, bool rigid_only, bool eval_error, float &push_error, float &pull_error, int iter);
    void track(DataFrame &frame, LinearSystem &sys, bool rigid_only, bool eval_error, float &push_error, float &pull_error, int iter, int mod_num );
    void init(Worker* worker);
	void dumbInit(Worker* worker);
	void swapHands();
    void cleanup();
	~Fitting();
#else
    void track(DataFrame &frame, LinearSystem &sys, bool rigid_only, bool eval_error, float &push_error, float &pull_error){}
    void init(Worker* worker){}
    void cleanup(){}
#endif
};
} /// energy::
