#include "davetracker.h"
#include <iostream>

#define HAVE_OPENCV

int main(){
    DaveTracker2::APTOptions my_Options =
        DaveTracker2::Load_Ini_Files("config/GeneralSettings.ini");

    DaveTracker2::Tracker my_Tracker(my_Options);

    std::cout << "Ready, press enter to start" << std::endl;
    std::cin.get();
    my_Tracker.Fine_Tracker(100);

    return 0;
}
