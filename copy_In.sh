#!/bin/sh

# Project specific stuff.
cp ../LD_Camera/include/LD_Camera.h include/
cp ../LD_MemsMirror/include/LD_MemsMirror.h include/
cp ../LD_Camera/include/LD_Camera.h include/
cp ../QuarcTracker_Linked/include/LD_TrackerCamera.h include/
cp ../QuarcTracker_Linked/include/LD_QuarcTracker.h include/

cp ../LD_Camera/src/LD_Camera.cpp src/
cp ../LD_MemsMirror/src/LD_MemsMirror.cpp src/
cp ../LD_Camera/src/LD_Camera.cpp src/
cp ../QuarcTracker_Linked/src/LD_TrackerCamera.cpp src/
cp ../QuarcTracker_Linked/src/LD_QuarcTracker.cpp src/
cp ../QuarcTracker_Linked/main.cpp ./

cp ../QuarcTracker_Linked/config/GeneralSettings.ini config/
cp ../QuarcTracker_Linked/image_to_png.py ./
cp ../QuarcTracker_Linked/.gitignore ./

# General purpose.
cp ../LD_Util/cpp/include/LD_Pid.h include/
cp ../LD_Util/cpp/include/LD_Timer.h include/
cp ../LD_Util/cpp/include/LD_Util.h include/

cp ../LD_Util/cpp/src/LD_Pid.cpp src/
cp ../LD_Util/cpp/src/LD_Timer.cpp src/
cp ../LD_Util/cpp/src/LD_Util.cpp src/


# Stuff I didn't write.
cp ../LD_Util/INI_Reader/include/ini.h include/
cp ../LD_Util/INI_Reader/include/INIReader.h include/
cp ../LD_Util/rs232/include/rs232.h include/

cp ../LD_Util/INI_Reader/src/ini.cpp src/
cp ../LD_Util/INI_Reader/src/INIReader.cpp src/
cp ../LD_Util/rs232/src/rs232.cpp src/
