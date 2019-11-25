#include "LD_QuarcTracker.h"
#include <iostream>

int main(){
    LD_QuarcTracker::APTOptions my_Options =
        LD_QuarcTracker::Load_Ini_Files("config/GeneralSettings.ini");

    LD_QuarcTracker::Tracker my_Tracker(my_Options);

    std::cout << "Ready, press enter to start" << std::endl;
    std::cin.get();
    my_Tracker.Fine_Tracker(100);

    return 0;
}
