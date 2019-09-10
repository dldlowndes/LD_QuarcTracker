#include "DaveUtil.h"

#include <iostream>

#ifdef _WIN32
    #include <Windows.h>
#elif __linux
    #include <unistd.h>
#endif // _WIN32

void MySleep(int SleepMs)
{
#ifdef _WIN32
	Sleep(SleepMs);
#elif __linux
	usleep(SleepMs * 1000);
#endif // _WIN32
}
