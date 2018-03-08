#include <iostream>
#include <QApplication>
#include <qwindow.h>

#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/Camera.h"

#include "tracker/HTracker.h"
#include "tracker/GLWidget.h"

#include <myo/myo.hpp>
#include "DataCollector.h"
#include "IMUReceiver.h"
//#include <vld.h>

int main(int argc, char* argv[]) {
	bool htrack = false;
	bool test = false; //J' * J on CPU
	bool real_color = false;
	bool record_only = false; //Try this? pass to tracker.
	bool save_rastorized_model = false;

	bool benchmark = false;
	bool playback = false;
	bool myoEnable = true;


	std::cout << "/n Pre IMUReceiver constructor";
	//IMUReceiver rec;
	//rec.HelloUDP();
	//IMUReceiver* rec = new IMUReceiver();
	std::cout << "/n Post IMUReceiver constructor/n";

	DataCollector collector = DataCollector(myoEnable);
	myo::Hub hub("taylor.com.text");
	if (myoEnable){
		myo::Myo* myo = hub.waitForMyo(10000);
		//DataCollector collector = DataCollector();

		if (!myo) {
			std::cout << "Unable to find a Myo!" << endl;
		}
		else {
			std::cout << "   Found a Myo?" << endl;
		}

		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		hub.addListener(&collector);
	}

	Handedness handedness = right_hand;
	std::string sequence_path = "C:/Projects/Data/Fingerspelling/";
	std::string data_path = "C:/Projects/MyoFaceVersion/hmodel/data/";
	int user_name = 61;
	if (user_name == 13)
		handedness = left_hand;

	std::string sequence_name = "P" + std::string(2 - std::to_string(user_name).length(), '0') + std::to_string(user_name);

	Q_INIT_RESOURCE(shaders);
	QApplication app(argc, argv);


	//Camera camera(QVGA, 60);
	Camera camera(RealSense, 60);
	SensorRealSense sensor(&camera, real_color, handedness, data_path, user_name);

	DataStream datastream(&camera);
	SolutionStream solutions;

	Worker worker(&camera, test, benchmark, save_rastorized_model, user_name, data_path, handedness, &sensor);

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


	HTracker tracker(&worker, camera.FPS(), sequence_path, real_color, myoEnable, record_only, user_name);

	GLWidget glwidget(&worker, &datastream, &solutions, playback, false /*real_color*/, data_path, &collector, sequence_path + sequence_name + "/", &tracker);
	worker.bind_glwidget(&glwidget);
	glwidget.show();

	//glwidget.imuReceiver = &rec;
	//rec.setWidget(&glwidget);


	//Full screen
	if (record_only) {
		glwidget.windowHandle()->setScreen(app.screens()[1]);
		glwidget.showFullScreen();
	}

	//Tracker tracker(&worker, camera.FPS(), sequence_path, real_color, myoEnable, record_only, user_name);
	//Tracker tracker(&worker, camera.FPS(), sequence_path + sequence_name + "/aberdeen_1_", real_color, myoEnable, record_only);
	if (myoEnable) {
		tracker.hub = &hub;
		if (!record_only)
			collector.recording = true;
	}
	//Tracker tracker(&worker, &hub, camera.FPS(), sequence_path + sequence_name + "/", real_color);

	tracker.sensor = &sensor;
	tracker.datastream = &datastream;
	tracker.solutions = &solutions;

	///--- Starts the tracking
	tracker.toggle_sensor(true);
	//tracker.toggle_tracking(false);  //This will start w/o tracking. Activate by finding wristband and model fitting
	tracker.toggle_tracking(!benchmark && !playback);
	tracker.toggle_benchmark(benchmark);
	tracker.toggle_playback(playback);

	return app.exec();
}
