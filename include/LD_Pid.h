#ifndef LD_PID_H
#define LD_PID_H

#include <chrono>

struct PIDParams{
    float P;
    float I;
    float D;
    bool constant_Time;
    float max_Output;
    float min_Output;

};

class PIDLoop {
private:
	float k_P;
	float k_I;
	float k_D;
	float error_Last;
	//let's try it a different way...
	float proportional;
	float integral;
	float derivative;
	bool first_Loop;
	float max_Output;
	float min_Output;
    std::chrono::time_point<std::chrono::steady_clock> Timer1;
	std::chrono::time_point<std::chrono::steady_clock> Timer2;
	//std::chrono::duration<float, std::milli> TimeInterval;
	bool constant_PID_Time;
	float time_Interval;

public:
	PIDLoop();
	PIDLoop(PIDParams my_Options);
	~PIDLoop();
	int InitPID(PIDParams my_Options);
	float UpdatePID(float error);
	int ChangePID(float myK_p, float myK_i, float myK_d);
    int ResetPID();
};

struct XY_PID{
    PIDLoop x;
    PIDLoop y;
};

#endif // LD_PID_H
