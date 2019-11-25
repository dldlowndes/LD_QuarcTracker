#include "LD_Pid.h"
#include <chrono>
#include <iostream>

PIDLoop::PIDLoop() {}

PIDLoop::PIDLoop(PIDParams my_Options) {
    InitPID(my_Options);
}

PIDLoop::~PIDLoop() {}

int PIDLoop::InitPID(PIDParams my_Options) {
	k_P = my_Options.P;
	k_I = my_Options.I;
	k_D = my_Options.D;
	std::cout << "Init PID with P:" << k_P << ", I:" << k_I << ", D:" << k_D << std::endl;
	error_Last = 0;
	integral = 0;
	derivative = 0;
	first_Loop = true;
	max_Output = my_Options.max_Output;
	min_Output = my_Options.min_Output;
	constant_PID_Time = my_Options.constant_Time;
	Timer1 = std::chrono::steady_clock::now();
	Timer2 = std::chrono::steady_clock::now();
	time_Interval = 40; // Default value if constant_PID_Time is true. Set to something sensible!
	return 0;
}

float PIDLoop::UpdatePID(float error){
    // This time_Interval thing is horrible! Full of dodgy cludges at the moment! :/
    if (!constant_PID_Time){
        Timer2 = Timer1;
        Timer1 = std::chrono::steady_clock::now();
        time_Interval = std::chrono::duration_cast<std::chrono::milliseconds>(Timer1 - Timer2).count() / 10;
    }

    error /= 1000;

    // P part.
	proportional = k_P * error;


    if (!first_Loop) {
    // I and D terms not valid in first loop. Only do P for first step.
        // I part. Clip to max range of mirror (just in case)
        integral = integral + (k_I * (error*time_Interval/2));
        if (integral > (max_Output / 2)) {
            integral = max_Output / 2;
        }
        else if (integral < (min_Output / 2 )) {
            integral = min_Output / 2;
        }

        // D part.
        derivative = k_D * ((error - error_Last) / (time_Interval * 1000));
	}
	else {
        first_Loop = false;
	}

    float output_PID = (proportional) + (integral) + (derivative);

	// Clip entire PID output to max range of mirror
	if (output_PID > max_Output) {
		output_PID = max_Output;
	}
	else if (output_PID < min_Output) {
		output_PID = min_Output;
	}

	// We need this to calculate D term next time.
	error_Last = error;

	//std::cout << "P,I,D: " << (proportional) << ", " << (integral) << ", " << (derivative) << "\n";
	return output_PID;
}

int PIDLoop::ChangePID(float myK_p, float myK_i, float myK_d){
    k_P = myK_p;
    k_I = myK_i;
    k_D = myK_d;
    //std::cout << "Changed PID parameters" << "\n";
    return 0;
}

int PIDLoop::ResetPID(){
	error_Last = 0;
	integral = 0;
	first_Loop = true;
    return 0;
}
