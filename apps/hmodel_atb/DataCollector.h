#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
//include "C:\Users\Brandon\Downloads\myo-sdk-win-0.9.0\myo-sdk-win-0.9.0\include\myo\cxx\DeviceListener.hpp"

//#include "stdafx.h"
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <array>

#include <myo/myo.hpp>

class DataCollector : public myo::DeviceListener
{
public:
	bool onArm;
	myo::Arm whichArm;
	bool isUnlocked;
	int roll_w, pitch_w, yaw_w;
	myo::Pose currentPose;
	std::array<int8_t, 8> emgSamples;
	uint64_t orient_timestamp;
	myo::Quaternion<float> orient_q;

	static int const		emgBufSize = 200 * 30;
	static int const		orientBufSize = 50 * 30;
	uint64_t* orient_time_buffer;
	uint64_t* emg_time_buffer;
	myo::Quaternion<float>* orient_buffer;
	std::array<int8_t, 8>* emg_buffer;
	time_t					first_EMG;
	time_t					first_IMU;
	//LARGE_INTEGER			first_EMG;
	//LARGE_INTEGER			first_IMU;
	int o_buf_i;
	int e_buf_i;
	bool recording;
	int a_buf_i;
	int g_buf_i;
	myo::Vector3<float>* accel_buffer;
	uint64_t* accel_time_buffer;
	myo::Vector3<float>* gyro_buffer;
	uint64_t* gyro_time_buffer;

	DataCollector();
	~DataCollector();
	
	void onUnpair(myo::Myo* myo, uint64_t timestamp);
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat);
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState);
	void onArmUnsync(myo::Myo* myo, uint64_t timestamp);
	void onUnlock(myo::Myo* myo, uint64_t timestamp);
	void onLock(myo::Myo* myo, uint64_t timestamp);
	void print();
	void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel);
	void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro);
	void saveMyoData(std::string filepath);
};

