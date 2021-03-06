// ViveMocap.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"

#define STOREPATH "animdata"
#define RAD_2_DEG (180.0/3.14159)
#define TIMESTR_BUFSIZE 1024
#define MAX_TIME 1800

using namespace vr;

int record_hz;

typedef struct {
	double x;
	double y;
	double z;
	double qw;
	double qx;
	double qy;
	double qz;
	double time;

} six_dof_moment;

double get_time() {
	return double(clock()) / CLOCKS_PER_SEC;
}

six_dof_moment pose_to_six_dof_moment(TrackedDevicePose_t pose, double time) {
	six_dof_moment moment;
	HmdMatrix34_t mat = pose.mDeviceToAbsoluteTracking;

	moment.x = mat.m[0][3];
	moment.y = mat.m[1][3];
	moment.z = mat.m[2][3];

	double tr = mat.m[0][0] + mat.m[1][1] + mat.m[2][2];

	if (tr > 0) {
		double S = sqrt(tr + 1.0) * 2;
		moment.qw = 0.25 * S;
		moment.qx = (mat.m[2][1] - mat.m[1][2]) / S;
		moment.qy = (mat.m[0][2] - mat.m[2][0]) / S;
		moment.qz = (mat.m[1][0] - mat.m[0][1]) / S;
	} else if ((mat.m[0][0] > mat.m[1][1]) && (mat.m[0][0] > mat.m[2][2])) {
		double S = sqrt(1.0 + mat.m[0][0] - mat.m[1][1] - mat.m[2][2]) * 2;
		moment.qw = (mat.m[2][1] - mat.m[1][2]) / S;
		moment.qx = 0.25 * S;
		moment.qy = (mat.m[0][1] + mat.m[1][0]) / S;
		moment.qz = (mat.m[0][2] + mat.m[2][0]) / S;
	} else if (mat.m[1][1] > mat.m[2][2]) {
		double S = sqrt(1.0 + mat.m[1][1] - mat.m[0][0] - mat.m[2][2]) * 2;
		moment.qw = (mat.m[0][2] - mat.m[2][0]) / S;
		moment.qx = (mat.m[0][1] + mat.m[1][0]) / S;
		moment.qy = 0.25 * S;
		moment.qz = (mat.m[1][2] + mat.m[2][1]) / S;
	} else {
		double S = sqrt(1.0 + mat.m[2][2] - mat.m[0][0] - mat.m[1][1]) * 2;
		moment.qw = (mat.m[1][0] - mat.m[0][1]) / S;
		moment.qx = (mat.m[0][2] + mat.m[2][0]) / S;
		moment.qy = (mat.m[1][2] + mat.m[2][1]) / S;
		moment.qz = 0.25 * S;
	}

	moment.time = time;

	return moment;
}

std::string serialize_moment(six_dof_moment moment) {
	std::ostringstream s;
	s << moment.x << " " << moment.y << " " << moment.z << " ";
	s << moment.qw << " " << moment.qx << " " << moment.qy << " " << moment.qz << " ";
	s << moment.time;

	return s.str();
}

void event_loop(IVRSystem* VR) {

	TrackedDeviceIndex_t activeDevice = k_unTrackedDeviceIndexInvalid;
	double initialTime = 0;
	double nextPoll = 0;
	std::vector<six_dof_moment> animation;

	while (true) {
		VREvent_t ev;
		bool close_anim = false;

		if (VR->PollNextEvent(&ev, sizeof(VREvent_t))) {
			if (VR->GetTrackedDeviceClass(ev.trackedDeviceIndex) == TrackedDeviceClass_Controller) {
				if (ev.data.controller.button == EVRButtonId::k_EButton_SteamVR_Trigger) {
					if (ev.eventType == VREvent_ButtonPress && activeDevice == k_unTrackedDeviceIndexInvalid) {
						std::cout << "Recording!" << std::endl;
						activeDevice = ev.trackedDeviceIndex;
						initialTime = get_time();
						nextPoll = 0;
					} else if (ev.eventType == VREvent_ButtonUnpress && ev.trackedDeviceIndex == activeDevice) {
						close_anim = true;
					}
				}
			}
		}

		if (activeDevice != k_unTrackedDeviceIndexInvalid) {
			double curtime = get_time() - initialTime;
			if (curtime >= nextPoll) {
				nextPoll += 1.0 / record_hz;
				TrackedDevicePose_t devicePose[k_unMaxTrackedDeviceCount];
				VR->GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding, 0, devicePose, k_unMaxTrackedDeviceCount);
				six_dof_moment m = pose_to_six_dof_moment(devicePose[activeDevice], curtime);
				std::cout << " " << serialize_moment(m) << std::endl;
				animation.push_back(m);
				if (curtime > MAX_TIME) {
					close_anim = true;
					std::cout << "Maximum recording time of " << MAX_TIME << " seconds exceeded" << std::endl;
				}
			}
		}

		if (close_anim) {

			std::cout << "Saving recorded data..." << std::endl;

			// get time as formatted string
			time_t rawtime;
			time(&rawtime);
			struct tm time_s;
			localtime_s(&time_s, &rawtime);
			char timestr[TIMESTR_BUFSIZE];
			strftime(timestr, TIMESTR_BUFSIZE, "%a %b %d %H-%M-%S %G", &time_s);

			// create filename
			std::stringstream filepath_ss;
			filepath_ss << STOREPATH << "/" << timestr << ".txt";
			std::string filepath = filepath_ss.str();

			// write anim data to file
			std::ofstream datafile;
			datafile.open(filepath);
			if (datafile.is_open()) {
				for (auto it = animation.begin(); it != animation.end(); ++it)
					datafile << serialize_moment(*it) << std::endl;
				datafile.close();
				std::cout << "Data saved to " << filepath << std::endl;
			} else {
				std::cout << "Unable to open file! Please try again later." << std::endl;
			}

			// clean up
			animation.clear();
			activeDevice = k_unTrackedDeviceIndexInvalid;

			close_anim = false;
		}
	}
}

void initialize() {
	do {
		std::cout << "Please type the polling hz you would like to record at: ";
		std::cin >> record_hz;
	} while (record_hz <= 0);
	std::cout << "Now recording at " << record_hz << " hz.\n";
	std::cout << "Hold trigger on any connected vive wand to begin recording.\n";
}

int main() {

	if (!CreateDirectoryA(STOREPATH, NULL) && GetLastError() != ERROR_ALREADY_EXISTS) {
		std::cout << "Unable to create directory " << STOREPATH << "/ and cannot continue, aborting." << std::endl;
		return 1;
	}

	HmdError vrError;
	IVRSystem* VR = VR_Init(&vrError, EVRApplicationType::VRApplication_Background);

	if (vrError == VRInitError_None) {
		initialize();
		event_loop(VR);
		VR_Shutdown();
	} else if (vrError == VRInitError_Init_NoServerForBackgroundApp) {
		std::cout << "SteamVR not running; please start SteamVR before continuing." << std::endl;
	} else {
		std::cout << "Error code " << vrError << " encountered while starting." << std::endl;
	}

	return 0;
}