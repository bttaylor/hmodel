#include "DataCollector.h"
#include <fstream>


DataCollector::DataCollector(bool enabled)
	:onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose(), emgSamples(), orient_q(), orient_timestamp(0),
	o_buf_i(0), e_buf_i(0), recording(false), a_buf_i(0), g_buf_i(0)
{
	this->enabled = enabled;
	for (int i = 0; i < emgSamples.size(); i++) {
		emgSamples[i] = 0;
	}

	emg_buffer = new std::array<int8_t, 8>[emgBufSize];
	emg_time_buffer = new uint64_t[emgBufSize];
	orient_time_buffer = new uint64_t[orientBufSize];
	orient_buffer = new myo::Quaternion<float>[orientBufSize];
	accel_time_buffer = new uint64_t[orientBufSize];
	gyro_time_buffer = new uint64_t[orientBufSize];
	accel_buffer = new myo::Vector3<float>[orientBufSize];
	gyro_buffer = new myo::Vector3<float>[orientBufSize];
}


DataCollector::~DataCollector()
{
}


// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
void DataCollector::onUnpair(myo::Myo* myo, uint64_t timestamp)
{
	// We've lost a Myo.
	// Let's clean up some leftover state.
	roll_w = 0;
	pitch_w = 0;
	yaw_w = 0;
	onArm = false;
	isUnlocked = false;
	emgSamples.fill(0);
}

void DataCollector::onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel) {
	if (recording) {
		if (a_buf_i == 0) {
			time(&first_IMU);
			//QueryPerformanceCounter(&first_IMU);
		}
		accel_time_buffer[a_buf_i % orientBufSize] = timestamp;
		accel_buffer[a_buf_i % orientBufSize] = accel;
		++a_buf_i;
	}
}

void DataCollector::onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro) {
	if (recording) {
		gyro_time_buffer[g_buf_i % orientBufSize] = timestamp;
		gyro_buffer[g_buf_i % orientBufSize] = gyro;
		++g_buf_i;
		std::cout << "x: " << gyro[0] << " y: " << gyro[1] << " z: " << gyro[2] << std::endl;
	}
}

std::vector<float> DataCollector::integrateRotation(){
	std::vector<float> xyz(3);
	float rotation_x = 0;
	float rotation_y = 0;
	float rotation_z = 0;
	int current_i = g_buf_i;
	uint64_t t_0 = gyro_time_buffer[last_query];
	if (last_query + 1 < current_i){
		//std::cout << "t_difs: ";
		for (int i = 0; i < current_i - last_query - 1; i++){
			uint64_t t_1 = gyro_time_buffer[last_query + 1 + i];
			float t_dif = (float)(t_1 - t_0) / 1000000.0;
			rotation_x += gyro_buffer[last_query + 1 + i][0] * t_dif;
			rotation_y += gyro_buffer[last_query + 1 + i][1] * t_dif;
			rotation_z += gyro_buffer[last_query + 1 + i][2] * t_dif;
			//std::cout << t_dif << " ";
			t_0 = t_1;
		}
		//std::cout << " " << last_query << " " << current_i << std::endl;
	}
	last_query = current_i-1;
	xyz[0] = rotation_x;
	xyz[1] = rotation_y;
	xyz[2] = rotation_z;
	return xyz;
}

// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
// as a unit quaternion.
void DataCollector::onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
{
	if (recording) {
		if (o_buf_i == 0) {
			time(&first_EMG);
			//QueryPerformanceCounter(&first_EMG);
		}
		orient_q = quat;
		orient_timestamp = timestamp;
		orient_time_buffer[o_buf_i % orientBufSize] = orient_timestamp;
		orient_buffer[o_buf_i % orientBufSize] = orient_q;
		++o_buf_i;
	}

	using std::atan2;
	using std::asin;
	using std::sqrt;
	using std::max;
	using std::min;

	// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
	float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
		1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
	float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
	float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
		1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

	// Convert the floating point angles in radians to a scale from 0 to 18.
	roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
	pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
	yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
	//std::cout << "myo onOrientationChanged. pitch: " << pitch_w << " myo: " << (long)myo << std::endl;
}

// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
// making a fist, or not making a fist anymore.
void DataCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
	currentPose = pose;

	if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
		// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
		// Myo becoming locked.
		myo->unlock(myo::Myo::unlockHold);

		// Notify the Myo that the pose has resulted in an action, in this case changing
		// the text on the screen. The Myo will vibrate.
		myo->notifyUserAction();
	}
	else {
		// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
		// are being performed, but lock after inactivity.
		myo->unlock(myo::Myo::unlockTimed);
	}
}


// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
void DataCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
	for (int i = 0; i < 8; i++) {
		emgSamples[i] = emg[i];
	}
	if (recording) {
		emg_time_buffer[e_buf_i % emgBufSize] = timestamp;
		emg_buffer[e_buf_i % emgBufSize] = emgSamples;
		++e_buf_i;
	}
}

// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
// arm. This lets Myo know which arm it's on and which way it's facing.
void DataCollector::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
	myo::WarmupState warmupState)
{
	onArm = true;
	whichArm = arm;
}

// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
// when Myo is moved around on the arm.
void DataCollector::onArmUnsync(myo::Myo* myo, uint64_t timestamp)
{
	onArm = false;
}

// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
void DataCollector::onUnlock(myo::Myo* myo, uint64_t timestamp)
{
	isUnlocked = true;
}

// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
void DataCollector::onLock(myo::Myo* myo, uint64_t timestamp)
{
	isUnlocked = false;
}

// There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
// For this example, the functions overridden above are sufficient.

// We define this function to print the current values that were updated by the on...() functions above.
void DataCollector::print()
{
	// Clear the current line
	std::cout << '\r';

	// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
	std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
		<< '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
		<< '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';

	if (onArm) {
		// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

		// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
		// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
		// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
		std::string poseString = currentPose.toString();

		std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
			<< '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
			<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
	}
	else {
		// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
		std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
	}

	std::cout << std::flush;
}

void DataCollector::onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
	std::cout << "  Entered onPair" << std::endl;
	knownMyos.push_back(myo);

	std::cout << "  Paired with " << identifyMyo(myo) << std::endl;
}

size_t DataCollector::identifyMyo(myo::Myo* myo) {
	// Walk through the list of Myo devices that we've seen pairing events for.
	for (size_t i = 0; i < knownMyos.size(); ++i) {
		// If two Myo pointers compare equal, they refer to the same Myo device.
		if (knownMyos[i] == myo) {
			return i + 1;
		}
	}
	return 0;
}


void DataCollector::saveMyoData(std::string filepath) {

	std::ofstream outfile;

	std::string path = filepath + "myoEMG.csv";
	//write out myo data - EMG has different sample rate than other info

	//myfile.open("myoEMG.csv", std::ofstream::app);
	outfile.open(path, std::ofstream::app);

	//std::cout << "in Myo::saveMyoData trying path: " << path << std::endl;
	//outfile << "blerp blorp";
	//outfile.close();

//	std::cout << "orient_time_buff[0] " << e_buf_i << std::endl;
	//std::cout << "first_EMG " << first_EMG << std::endl;

	outfile << first_EMG << std::endl;
	for (int i = 0; i < e_buf_i; i++) {
		outfile << emg_time_buffer[i] << ",";
		outfile << static_cast<int>(emg_buffer[i][0]) << ",";
		outfile << static_cast<int>(emg_buffer[i][1]) << ",";
		outfile << static_cast<int>(emg_buffer[i][2]) << ",";
		outfile << static_cast<int>(emg_buffer[i][3]) << ",";
		outfile << static_cast<int>(emg_buffer[i][4]) << ",";
		outfile << static_cast<int>(emg_buffer[i][5]) << ",";
		outfile << static_cast<int>(emg_buffer[i][6]) << ",";
		outfile << static_cast<int>(emg_buffer[i][7]) << std::endl;
	}
	outfile.close();

	//write out other myo data
	if ((a_buf_i != o_buf_i) || (a_buf_i != g_buf_i)) {
		std::cout << "Buffer Mismatch Error in the Myo" << std::endl;
		//dumpMyoBuffersIndividually();
	}
	else {
		path = filepath + "myoIMU.csv";
		outfile.open(path, std::ofstream::app);
		outfile << first_IMU << std::endl;
		for (int i = 0; i < o_buf_i; i++) {
			outfile << orient_time_buffer[i] << ",";
			outfile << orient_buffer[i].w() << ",";
			outfile << orient_buffer[i].x() << ",";
			outfile << orient_buffer[i].y() << ",";
			outfile << orient_buffer[i].z() << ",";
			outfile << accel_buffer[i].x() << ",";
			outfile << accel_buffer[i].y() << ",";
			outfile << accel_buffer[i].z() << ",";
			outfile << gyro_buffer[i].x() << ",";
			outfile << gyro_buffer[i].y() << ",";
			outfile << gyro_buffer[i].z() << std::endl;
		}
		outfile.close();
	}

}

void DataCollector::clear_buffers() {
	e_buf_i = 0;
	o_buf_i = 0;
	a_buf_i = 0;
	g_buf_i = 0;
}