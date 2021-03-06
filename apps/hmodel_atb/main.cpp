#include <iostream>
#include <QApplication>

#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/Camera.h"

#include "tracker/Tracker.h"
#include "tracker/GLWidget.h"

#include <myo/myo.hpp>
#include "DataCollector.h"

//#include <vld.h>

int main(int argc, char* argv[]) {
	bool htrack = false;
	bool test = false; //J' * J on CPU
	bool real_color = false;
	bool save_rastorized_model = false;

	bool benchmark = false;
	bool playback = false;
	int user_name = 10;
	bool myoEnable = false;

	DataCollector collector = DataCollector();
	myo::Hub hub("taylor.com.text");
	if (myoEnable) {		
		myo::Myo* myo = hub.waitForMyo(10000);
		//DataCollector collector = DataCollector();

		if (!myo) {
			std::cout << "Unable to find a Myo!";
		}

		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		hub.addListener(&collector);
	}

	Handedness handedness = right_hand;  //0 == Left; 1 == Right; 2 == Both
	std::string sequence_path = "C:/Projects/Data/TwoHand/"; // "C:/Projects/Data/Fingerspelling/";
	std::string data_path = "C:/Projects/ASLRecog/hmodel/data/"; // "C:/Projects/clean0426/hmodel-master/data/";
	std::string sequence_name = "P01";  //"P01";

	Q_INIT_RESOURCE(shaders);
	QApplication app(argc, argv);

	Camera camera(RealSense, 60);

	//0 = Left == Yellow; 1 == Right == Blue
	SensorRealSense sensor(&camera, real_color, handedness);
	std::cout << "after SensorRealSense sensor()" << std::endl;
	DataStream datastream(&camera);
	SolutionStream solutions;

	//0 = Left == Yellow; 1 == Right == Blue
	Worker worker(&camera, test, benchmark, save_rastorized_model, user_name, data_path, handedness);

	{
		worker.settings->termination_max_iters = 8;

		worker.E_fitting.settings->fit2D_enable = true;
		worker.E_fitting.settings->fit2D_weight = 0.7;

		worker.E_fitting.settings->fit3D_enable = true;

		worker.E_limits.jointlimits_enable = true;

		worker.E_pose._settings.enable_split_pca = true;
		worker.E_pose._settings.weight_proj = 4 * 10e2; 

		worker.E_collision._settings.collision_enable = true;
		worker.E_collision._settings.collision_weight = 1e3;

		worker.E_temporal._settings.temporal_coherence1_enable = true;
		worker.E_temporal._settings.temporal_coherence2_enable = true;
		worker.E_temporal._settings.temporal_coherence1_weight = 0.05;
		worker.E_temporal._settings.temporal_coherence2_weight = 0.05;

		worker.E_damping._settings.abduction_damping = 1500000;
		worker._settings.termination_max_rigid_iters = 1;
	}

	GLWidget glwidget(&worker, &collector, &datastream, &solutions, playback, false /*real_color*/, data_path, sequence_path + sequence_name + "/");
	glwidget.myoEnable = myoEnable;
	worker.bind_glwidget(&glwidget,&glwidget.convolution_renderer);
	glwidget.show();

	Tracker tracker(&worker, &hub, camera.FPS(), sequence_path + sequence_name + "/", real_color);	
	tracker.myoEnable = myoEnable;

	tracker.sensor = &sensor;
	tracker.datastream = &datastream;
	tracker.solutions = &solutions;
	
	///--- Starts the tracking
	tracker.toggle_tracking(!benchmark && !playback);
	tracker.toggle_benchmark(benchmark);
	tracker.toggle_playback(playback);

	return app.exec();
}
