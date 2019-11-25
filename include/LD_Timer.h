#ifndef DAVETIMER_H
#define DAVETIMER_H

#include <chrono>

class DaveTimer
{
    public:
        DaveTimer();
        ~DaveTimer();

        void Start_Timer();
        void Stop_Timer();
        int Get_Last_Time_Difference();

    protected:

    private:
        bool timer_Started;
        bool timer_Stopped;

        std::chrono::time_point<std::chrono::steady_clock> timer_Start;
        std::chrono::time_point<std::chrono::steady_clock> timer_Stop;
        std::chrono::duration<long int, std::milli> timer_Difference;
};

#endif // DAVETIMER_H
