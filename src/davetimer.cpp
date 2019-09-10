#include "davetimer.h"

DaveTimer::DaveTimer()
{
    //ctor
}

DaveTimer::~DaveTimer()
{
    //dtor
}

void DaveTimer::Start_Timer(){
    timer_Start = std::chrono::steady_clock::now();
    timer_Started = true;
    return;
}


void DaveTimer::Stop_Timer(){
    timer_Stop = std::chrono::steady_clock::now();
    timer_Stopped = true;
    return;
}


int DaveTimer::Get_Last_Time_Difference(){
    if (timer_Started){
        if (!timer_Stopped){
            Stop_Timer();
        }
        timer_Difference = std::chrono::duration_cast<std::chrono::milliseconds>(timer_Stop - timer_Start);
        return timer_Difference.count();
    }
    else{
        return 0;
    }

}
