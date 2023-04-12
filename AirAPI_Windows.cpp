#include "pch.h"
#include "AirAPI_Windows.h"
#include "deps/hidapi-win/include/hidapi.h"
#include "deps/Fusion/Fusion/Fusion.h"
#include <iostream>
#include <mutex>
#include <array>
#include <cstdint>
#include <vector>
//Air USB VID and PID
#define AIR_VID 0x3318
#define AIR_PID 0x0424

//Is Tracking
bool g_isTracking = false;

//Is Listening
bool g_isListening = false;

// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)

// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)

// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)

#define MAG_SCALAR (1.0f / 32768.0f) * 1600.0f
#define MAG_OFFSET -1600.0f

static int rows, cols;
static FusionEuler euler;
static FusionVector earth;
static FusionQuaternion qt;
static int visual_marker = 0;

HANDLE trackThread;
HANDLE listenThread;
static HANDLE hFile;
static FILE* fpt;

hid_device* device;

hid_device* device4;

#define SAMPLE_RATE (1000) // replace this with actual sample rate

std::mutex mtx;
std::mutex it4;

typedef struct {
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
	uint16_t mag[3];
} air_sample;


static int
parse_report(const unsigned char* buffer_in, int size, air_sample* out_sample)
{
	if (size != 64) {
		printf("Invalid packet size");
		return -1;
	}
	// clock in nanoseconds
	buffer_in += 4;
	out_sample->tick = ((uint64_t) * (buffer_in++));
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 8);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 16);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 24);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 32);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 40);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 48);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 56);

	uint32_t t0v, t1v, t2v, t3v, t0a, t1a, t2a, t3a;
	// gyroscope measurements	
	buffer_in += 6;
	if (*(buffer_in + 2) & 0x80) {
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[0] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[0] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[0] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[0] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[1] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[1] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[1] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[1] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[2] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[2] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[2] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[2] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	// accelerometer data
	buffer_in += 6;
	if (*(buffer_in + 2) & 0x80) {
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[0] = t0a | t1a | t2a | t3a;
		// out_sample->accel[0] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[0] = t0a | t1a | t2a | t3a;
		// out_sample->accel[0] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[1] = t0a | t1a | t2a | t3a;
		// out_sample->accel[1] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[1] = t0a | t1a | t2a | t3a;
		// out_sample->accel[1] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[2] = t0a | t1a | t2a | t3a;
		// out_sample->accel[2] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[2] = t0a | t1a | t2a | t3a;
		// out_sample->accel[2] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	// mag data
	buffer_in += 6;

	out_sample->mag[0] = *(buffer_in++);
	out_sample->mag[0] |= (*(buffer_in++) << 8);
	out_sample->mag[1] = *(buffer_in++);
	out_sample->mag[1] |= (*(buffer_in++) << 8);
	out_sample->mag[2] = *(buffer_in++);
	out_sample->mag[2] |= (*(buffer_in++) << 8);

	return 0;
}


static void
process_ang_vel(const int32_t in_ang_vel[3], float out_vec[])
{

	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_ang_vel[0]) * GYRO_SCALAR;
	out_vec[1] = (float)(in_ang_vel[1]) * GYRO_SCALAR;
	out_vec[2] = (float)(in_ang_vel[2]) * GYRO_SCALAR;
}

static void
process_accel(const int32_t in_accel[3], float out_vec[])
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_accel[0]) * ACCEL_SCALAR;
	out_vec[1] = (float)(in_accel[1]) * ACCEL_SCALAR;
	out_vec[2] = (float)(in_accel[2]) * ACCEL_SCALAR;

}

static void
process_mag(const uint16_t in_mag[3], float out_vec[])
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_mag[0]) * MAG_SCALAR + MAG_OFFSET;
	out_vec[1] = (float)(in_mag[1]) * MAG_SCALAR + MAG_OFFSET;
	out_vec[2] = (float)(in_mag[2]) * MAG_SCALAR + MAG_OFFSET;
}


static void
open_csv_logfile()
{
	char filename[40];
	struct tm* timenow;

	time_t now = time(NULL);
	timenow = gmtime(&now);

	strftime(filename, sizeof(filename), "./logs/data_%Y-%m-%d_%H-%M-%S.csv", timenow);

	fpt = fopen(filename, "w+");

	if (fpt == NULL) {
		// get out code
		printf("File open failed. Exiting...");
		exit(1);
	}
	else
	{
		printf("CSV Logfile opened: %s\n", filename);
	}


	fprintf(fpt, "ts_nanoseconds, gyro1_dps, gyro2_dps, gyro3_dps, accel1_g, accel2_g, accel3_g, mag1_uT, mag2_uT, mag3_uT, euler1_deg, euler2_deg, euler3_deg, quat1, quat2, quat3, quat4, accel_err_deg, accel_ignored, accel_rej_timer,mag_err_deg,mag_ignored,mag_rej_timer, initialising, accel_rej_warn, accel_rej_timeout, mag_rej_warn, mag_rej_timeout, visual_marker\n");
}


static hid_device*
open_device()
{
	struct hid_device_info* devs = hid_enumerate(AIR_VID, AIR_PID);
	struct hid_device_info* cur_dev = devs;
	hid_device* device = NULL;

	while (devs) {
		if (cur_dev->interface_number == 3) {
			device = hid_open_path(cur_dev->path);
			std::cout << "Interface 3 bound" << std::endl;
			break;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);

	open_csv_logfile();

	return device;
}

static hid_device*
open_device4()
{
	struct hid_device_info* devs = hid_enumerate(AIR_VID, AIR_PID);
	struct hid_device_info* cur_dev = devs;
	hid_device* device = NULL;

	while (devs) {
		if (cur_dev->interface_number == 4) {
			device = hid_open_path(cur_dev->path);
			std::cout << "Interface 4 bound" << std::endl;
			break;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return device;
}

struct ThreadParams {
	hid_device* device;
};

DWORD WINAPI track(LPVOID lpParam) {

	//Thread to handle tracking
	unsigned char buffer[64] = {};
	uint64_t last_sample_tick = 0;
	air_sample sample = {};
	ThreadParams* params = static_cast<ThreadParams*>(lpParam);
	//static FusionVector ang_vel = {}, accel_vec = {};
	static float ang_vel[3] = {};
	static float accel_vec[3] = {};
	static float mag_vec[3] = {};


	// Define calibration (replace with actual calibration data if available)
	const FusionMatrix gyroscopeMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector gyroscopeSensitivity = { 1.0f, 1.0f, 1.0f };
	const FusionVector gyroscopeOffset = { 0.0f, 0.0f, 0.0f };
	const FusionMatrix accelerometerMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector accelerometerSensitivity = { 1.0f, 1.0f, 1.0f };
	const FusionVector accelerometerOffset = { 0.0f, 0.0f, 0.0f };
	const FusionMatrix softIronMatrix = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector hardIronOffset = { 0.0f, 0.0f, 0.0f };


	// Initialise algorithms
	FusionOffset offset;
	FusionAhrs ahrs;

	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);


	// Set AHRS algorithm settings
	const FusionAhrsSettings settings = {
			.gain = 0.5f,
			.accelerationRejection = 10.0f,
			.magneticRejection = 20.0f,
			.rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);


	while (g_isTracking) {


		try {
			// code that might throw an exception
			int res = hid_read(device, buffer, sizeof(buffer));
			if (res < 0) {
				break;
			}
		}
		catch (const std::exception& e) {
			// handle the exception
			std::cerr << e.what();
		}

		if (buffer[0] != 0x01 || buffer[1] != 0x02) {
			continue;
		}


		//parse
		parse_report(buffer, sizeof(buffer), &sample);

		//process sample
		process_ang_vel(sample.ang_vel, ang_vel);
		process_accel(sample.accel, accel_vec);
		process_mag(sample.mag, mag_vec);

		// Acquire latest sensor data
		const uint64_t timestamp = sample.tick; // replace this with actual gyroscope timestamp
		FusionVector gyroscope = { ang_vel[0], ang_vel[1], ang_vel[2] }; // replace this with actual gyroscope data in degrees/s
		FusionVector accelerometer = { accel_vec[0], accel_vec[1], accel_vec[2] }; // replace this with actual accelerometer data in g

		// Apply calibration
		gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

		// Update gyroscope offset correction algorithm
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		// Calculate delta time (in seconds) to account for gyroscope sample clock error
		static uint64_t previousTimestamp;
		const float deltaTime = (float)(timestamp - previousTimestamp) / (float)1e9;
		previousTimestamp = timestamp;

		if (deltaTime > 0.01) {
			FusionAhrsReset(&ahrs);
			continue;
		}

		// Update gyroscope AHRS algorithm
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

		int visual_target_num = 0;

		//lock mutex and update values
		mtx.lock();
		qt = FusionAhrsGetQuaternion(&ahrs);
		euler = FusionQuaternionToEuler(qt);
		earth = FusionAhrsGetEarthAcceleration(&ahrs);
		visual_target_num = visual_marker;
		mtx.unlock();

		FusionAhrsInternalStates internal_states = FusionAhrsGetInternalStates(&ahrs);
		FusionAhrsFlags ahrs_flags = FusionAhrsGetFlags(&ahrs);

		

		fprintf(fpt, "%I64u, %f, %f, %f, %f, %f, %f, %f, %f, %f,", timestamp, ang_vel[0], ang_vel[1], ang_vel[2], accel_vec[0], accel_vec[1], accel_vec[2], mag_vec[0], mag_vec[1], mag_vec[2]);
		fprintf(fpt, " %f, %f, %f, %f, %f, %f, %f,", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, qt.element.w, qt.element.x, qt.element.y, qt.element.z);
		fprintf(fpt, " %f, %d, %f, %f, %d, %f,", internal_states.accelerationError, internal_states.accelerometerIgnored, internal_states.accelerationRejectionTimer,
			internal_states.magneticError, internal_states.magnetometerIgnored, internal_states.magneticRejectionTimer);
		fprintf(fpt, " %d, %d, %d, %d, %d,", ahrs_flags.initialising, ahrs_flags.accelerationRejectionWarning, ahrs_flags.accelerationRejectionTimeout,
			ahrs_flags.magneticRejectionWarning, ahrs_flags.magneticRejectionTimeout);
		fprintf(fpt, " %d\n", visual_target_num);

	}
	return 0;
}
int brightness = 0;
DWORD WINAPI interface4Handler(LPVOID lpParam) {
	
	//get initial brightness from device
	std::array<uint8_t, 17> initBrightness = { 0x00, 0xfd, 0x1e, 0xb9, 0xf0, 0x68, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };
	hid_write(device4, initBrightness.data(), initBrightness.size());
	

	while (g_isListening) {
		std::array<uint8_t, 65> recv = {};
		int res = hid_read(device4, recv.data(), recv.size());
		if (res > 0) {
			switch (recv[22]) {

			case 0x03: //Brightness down press
				it4.lock();
				brightness = recv[30];
				it4.unlock();
				break;

			case 0x02: //Brightness up press
				it4.lock();
				brightness = recv[30];
				it4.unlock();
				break;
			default:
				//std::cout << "Unknown Packet! " << (int)recv[22] << std::endl;
				break;
			}

			switch (recv[15]) {

			case 0x03: //Brightness from cmd
				it4.lock();
				brightness = recv[23];
				it4.unlock();
				break;

			default:
				//todo
				break;
			}
		}
	}
	return 1;
}


int StartConnection()
{


	if (g_isTracking) {
		std::cout << "Already Tracking" << std::endl;
		return 1;
	}
	else {
		std::cout << "Opening Device" << std::endl;
		// open device
		device = open_device();
		device4 = open_device4();
		if (!device || !device4) {
			std::cout << "Unable to open device" << std::endl;
			return 1;
		}


		std::cout << "Sending Payload" << std::endl;
		// open the floodgates
		uint8_t magic_payload[] = { 0x00, 0xaa, 0xc5, 0xd1, 0x21, 0x42, 0x04, 0x00, 0x19, 0x01 };


		int res = hid_write(device, magic_payload, sizeof(magic_payload));
		if (res < 0) {
			std::cout << "Unable to write to device" << std::endl;
			return 1;
		}
		ThreadParams trackParams = { device };
		g_isTracking = true;
		std::cout << "Tracking Starting Thread" << std::endl;


		//Start Tracking Thread
		trackThread = CreateThread(NULL, 0, track, &trackParams, 0, NULL);
		if (trackThread == NULL) {
			std::cout << "Failed to create thread" << std::endl;
			return 1;
		}

		ThreadParams listenParams = { };
		g_isListening = true;
		//Start Interface 4 listener
		listenThread = CreateThread(NULL, 0, interface4Handler, &listenParams, 0, NULL);
		if (listenThread == NULL) {
			std::cout << "Failed to create thread" << std::endl;
			return 1;
		}
		std::cout << "Listenr Thread Started" << std::endl;

		visual_marker = 0;

		return 1;
	}
}


int StopConnection()
{
	if (g_isTracking) {
		g_isTracking = false;
		g_isListening = false;
		// Wait for the track thread to finish
		WaitForSingleObject(trackThread, INFINITE);
		TerminateThread(trackThread, 0);
		CloseHandle(trackThread);

		// Wait for the listen thread to finish
		WaitForSingleObject(listenThread, INFINITE);
		TerminateThread(listenThread, 0);
		CloseHandle(listenThread);

		fclose(fpt);

		std::cout << "CSV logfile closed" << std::endl;

		return 1;
	}
	else {
		return -1;
	}
}

float* q = new float[4];

float* GetQuaternion()
{
	mtx.lock();
	q[0] = qt.array[0];
	q[1] = qt.array[1];
	q[2] = qt.array[2];
	q[3] = qt.array[3];
	mtx.unlock();
	return q;
}

float* e = new float[3];

float* GetEuler()
{

	mtx.lock();
	e[0] = euler.angle.pitch;
	e[1] = euler.angle.roll;
	e[2] = euler.angle.yaw;
	mtx.unlock();
	return e;
}

void SetVisualMarker(int marker)
{
	mtx.lock();
	visual_marker = marker;
	mtx.unlock();

	return;
}


int GetBrightness()
{
	int curBrightness;
	it4.lock();
	curBrightness = brightness;
	it4.unlock();

	return curBrightness;
}
