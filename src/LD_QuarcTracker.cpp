#include "LD_QuarcTracker.h"
#include "INIReader.h"
#include "rs232.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace LD_QuarcTracker{
    APTOptions Load_Ini_Files(std::string tracker_Ini_Filename){
        // Load the ini files for dumping out to structs.
        INIReader tracker_Ini(tracker_Ini_Filename);

        // Contains parameter structs for each element in the tracker.
        APTOptions my_Options;

        // The AOI middle is a more natural way of depicting an AOI but it will
        // be converted later into how the camera prefers is (which is top left
        // co-ordinate of the AOI)
        // This also makes it easy to define the set point of the tracker as
        // the middle of the AOI.
        LD_Camera::Pixel_Values aoi_Middle = {
            (int)tracker_Ini.GetInteger("Camera Settings", "AOI_Position_X", 640),
            (int)tracker_Ini.GetInteger("Camera Settings", "AOI_Position_Y", 512)
        };
        LD_Camera::Pixel_Values aoi_Size = {
            (int)tracker_Ini.GetInteger("Camera Settings", "AOI_Size_X", 256),
            (int)tracker_Ini.GetInteger("Camera Settings", "AOI_Size_Y", 256)
        };

        // AOI size can be used as-is since size is invariant of the way the
        // position is described.
        aoi_Size = {
            my_Options.camera_Options.aoi_Dimensions.aoi_Size.s32Width = aoi_Size.x,
            my_Options.camera_Options.aoi_Dimensions.aoi_Size.s32Height = aoi_Size.y
        };

        // AOI position is the top left of the AOI, not the middle.
        my_Options.camera_Options.aoi_Dimensions.aoi_Position = {
            aoi_Middle.x - (aoi_Size.x / 2),
            aoi_Middle.y - (aoi_Size.y / 2)
        };

        // Camera options. Separate values for the full frame and AOI as usual
        // since they might want to be different.
        my_Options.camera_Options.bits_Per_Pixel =
            tracker_Ini.GetInteger("Camera Settings", "Bits_Per_Pixel", 8);
        my_Options.camera_Options.exposure_Full.pixel_Clock =
            tracker_Ini.GetInteger("Camera Settings", "Pixel_Clock", 20);;
        my_Options.camera_Options.exposure_Full.frame_Rate =
            tracker_Ini.GetReal("Camera Settings", "FrameRate_Full", 10);
        my_Options.camera_Options.exposure_Full.exposure =
            tracker_Ini.GetReal("Camera Settings", "Exposure", 1);
        my_Options.camera_Options.exposure_AOI.pixel_Clock =
            tracker_Ini.GetInteger("Camera Settings", "Pixel_Clock", 20);;
        my_Options.camera_Options.exposure_AOI.frame_Rate =
            tracker_Ini.GetReal("Camera Settings", "FrameRate_AOI", 60);
        my_Options.camera_Options.exposure_AOI.exposure =
            tracker_Ini.GetReal("Camera Settings", "Exposure", 1);

        // Not much to set for the mirror, most of the settings are hard coded
        // in the driver since they are hardware dependent.
        my_Options.mirror_Options.comport_Name =
            tracker_Ini.Get("Mirror Settings", "Com_Port_Name", "ttyACM0");
        // For safety, can restrict the mirror not to fully hit its limits.
        my_Options.mirror_Options.limit =
            tracker_Ini.GetReal("Mirror Settings", "Limit", 0.25);

        // Full frame set point is just the AOI middle (that's the point)
        my_Options.tracker_Options.full_Setpoint = aoi_Middle;
        // AOI set point is the middle of the AOI (that's also the point!)
        my_Options.tracker_Options.aoi_Setpoint.x = aoi_Size.x / 2;
        my_Options.tracker_Options.aoi_Setpoint.y = aoi_Size.y / 2;
        // Can toggle the tracker on/off to show a comparison between the
        // on/off state directly.
        my_Options.tracker_Options.tracker_Period =
            tracker_Ini.GetInteger("Tracker Options", "Tracker_Period", 0);
        // An option to never enable the  AOI if preferrred.
        my_Options.tracker_Options.do_AOI =
            tracker_Ini.GetBoolean("Tracker Options", "do_AOI", false);
        // How far inside the AOI region should the spot be before triggering
        // the AOI? (to avoid repeatedly triggering AOI on/off when the spot
        // is right at the threshold. AOI is turned off if the spot is lost
        // on the AOI so there is hysteresis if this is <1
        my_Options.tracker_Options.aoi_Threshold =
            tracker_Ini.GetReal("Tracker Options", "aoi_Threshold", 0.5);

        // PID parameters for closed loop control. separate for full frame
        // and AOI.
        my_Options.tracker_Options.full_PID_Options.P =
            tracker_Ini.GetReal("Tracker Options", "pid_P", 0.1);
        my_Options.tracker_Options.full_PID_Options.I =
            tracker_Ini.GetReal("Tracker Options", "pid_I", 0);
        my_Options.tracker_Options.full_PID_Options.D =
            tracker_Ini.GetReal("Tracker Options", "pid_D", 0);
        my_Options.tracker_Options.full_PID_Options.constant_Time = true;
        my_Options.tracker_Options.full_PID_Options.min_Output =
            -my_Options.mirror_Options.limit;
        my_Options.tracker_Options.full_PID_Options.max_Output =
            my_Options.mirror_Options.limit;

        my_Options.tracker_Options.aoi_PID_Options.P =
            tracker_Ini.GetReal("Tracker Options", "pid_P_aoi", 0.1);
        my_Options.tracker_Options.aoi_PID_Options.I =
            tracker_Ini.GetReal("Tracker Options", "pid_I_aoi", 0);
        my_Options.tracker_Options.aoi_PID_Options.D =
            tracker_Ini.GetReal("Tracker Options", "pid_D_aoi", 0);
        // AOI PID options might all be set to 0 which means "just use the full
        // frame options".
        if((my_Options.tracker_Options.aoi_PID_Options.P == 0) &
           (my_Options.tracker_Options.aoi_PID_Options.I == 0) &
           (my_Options.tracker_Options.aoi_PID_Options.D == 0)){
           std::cout << "Making AOI PID and full PID parameters equal" << std::endl;
            my_Options.tracker_Options.aoi_PID_Options =
                my_Options.tracker_Options.full_PID_Options;
        }
        my_Options.tracker_Options.aoi_PID_Options.constant_Time = true;
        my_Options.tracker_Options.aoi_PID_Options.min_Output =
            -my_Options.mirror_Options.limit;
        my_Options.tracker_Options.aoi_PID_Options.max_Output =
            my_Options.mirror_Options.limit;

        // For the spot finder, only consider pixels that are above
        // peak_Thresh * maximum pixel. Reduces noise but also makes the
        // spot finding easier since dim areas far from the spot are totally
        // disregarded.
        my_Options.tracker_Options.full_Spot_Finder.peak_Thresh =
            tracker_Ini.GetReal("Spot Detection", "peak_Thresh_Full", 0.5);
        // Ditto for when "fitting" a gaussian to the spot profiles.
        my_Options.tracker_Options.full_Spot_Finder.gaussian_Thresh =
            tracker_Ini.GetReal("Spot Detection", "gaussian_Thresh_Full", 0.5);
        // Assuming there is a single peak in the image, there should only be
        // a small number of pixels above peak_Thresh. If there are too many
        // it's probably because the peak is very low or nonexistent.
        my_Options.tracker_Options.full_Spot_Finder.n_Peak_Pixels =
            tracker_Ini.GetReal("Spot Detection", "nPeakPixels", 500);
        // Dump out spot profiles for diagnostics?
        my_Options.tracker_Options.full_Spot_Finder.save_Data =
            tracker_Ini.GetBoolean("Spot Detection", "gaussian_SaveData", false);

        // Spot finder for AOI will have less systematic background because
        // the frame is smaller but the entire spot is still (hopefully) in
        // the AOI. The thresholds could probably be set a bit more gently.
        my_Options.tracker_Options.aoi_Spot_Finder.peak_Thresh =
            tracker_Ini.GetReal("Spot Detection", "peak_Thresh_AOI", 0.5);
        my_Options.tracker_Options.aoi_Spot_Finder.gaussian_Thresh =
            tracker_Ini.GetReal("Spot Detection", "gaussian_Thresh_AOI", 0.5);
        my_Options.tracker_Options.aoi_Spot_Finder.n_Peak_Pixels =
            tracker_Ini.GetReal("Spot Detection", "nPeakPixels", 500);
        my_Options.tracker_Options.aoi_Spot_Finder.save_Data =
            tracker_Ini.GetBoolean("Spot Detection", "gaussian_SaveData", false);

        // If HAVE_OPENCV is defined, the display mode determines how much
        // detail is plotted with the camera feed by opencv.
        my_Options.tracker_Options.display_Mode =
            tracker_Ini.GetInteger("Tracker Options", "Display_Mode", 2);

        return my_Options;
    }

    Tracker::Tracker(){

    }

    Tracker::Tracker(APTOptions my_Options){
        Init(my_Options);
    }

    Tracker::~Tracker(){

    }

    int Tracker::Init(APTOptions my_Options){
        switch (LD_Camera::FindCameras()){
            case 0:
                std::cout << "Error, no cameras found" << std::endl;
                return 1;
            case 1:
                my_Options.camera_Options.Camera_ID = 0;
                break;
            default:
                std::cout << "Choose camera for tracker: " << std::endl;
                std::cin >> my_Options.camera_Options.Camera_ID;
                break;
        }

        my_Spot_Finder = my_Options.tracker_Options.full_Spot_Finder;
        my_Spot_Finder_AOI = my_Options.tracker_Options.aoi_Spot_Finder;

        // Camera ID in case there are multiple cameras attached. 0 is default.
        //my_Options.camera_Options.Camera_ID = 0;
        my_Camera.Init(my_Options.camera_Options);
        my_Camera.Set_SpotFinder_Options(my_Spot_Finder);

        // Grab the AOI dimensions from the camera to set some parameters
        // of the tracker that need this
        use_AOI = my_Options.tracker_Options.do_AOI;
        LD_Camera::AOI temp_AOI = my_Options.camera_Options.aoi_Dimensions;

        // aoi_Boundary is the boundary of where the AOI should trigger ON
        // if this is set at the actual edge of the AOI and during the AOI
        // enable, the spot wanders back off the AOI. CHAOS ENSUES. This makes
        // sure the spot will remain by waiting until the spot is closer to the
        // set point.
        aoi_Boundary_X = (temp_AOI.aoi_Size.s32Width / 2) * my_Options.tracker_Options.aoi_Threshold;
        aoi_Boundary_Y = (temp_AOI.aoi_Size.s32Height / 2) * my_Options.tracker_Options.aoi_Threshold;

        my_Mirror.Init(my_Options.mirror_Options);

        // Set points the closed loop control will endeavour to bring the
        // spot.
        full_Setpoint = my_Options.tracker_Options.full_Setpoint;
        aoi_Setpoint = my_Options.tracker_Options.aoi_Setpoint;
        // Start in full frame mode.
        Set_Setpoint(full_Setpoint);


        display_Mode = my_Options.tracker_Options.display_Mode;
        tracker_Period = my_Options.tracker_Options.tracker_Period;

        pid_X.InitPID(my_Options.tracker_Options.full_PID_Options);
        pid_Y.InitPID(my_Options.tracker_Options.full_PID_Options);

        return 0;
    }

    LD_Camera::Pixel_Values Tracker::Get_Setpoint(){
        return active_Setpoint;
    }

    int Tracker::Set_Setpoint(LD_Camera::Pixel_Values new_Setpoint){
        active_Setpoint = new_Setpoint;
        return 0;
    }

    int Tracker::Fine_Tracker(uint64_t steps_To_Run){

        uint64_t step = 0;
        keep_Running = true;
        while(keep_Running){
            timer_Loop.Start_Timer();
            //std::cout << "Start step " << step << std::endl;
            // Take camera image, react to it (aoi, mirror etc)
            Fine_Track_Step();

            #ifdef HAVE_OPENCV
            // If openCV is available (and wanted), display the camera image in
            // a window with some decorations (re: display_Mode) and watch for
            // keypresses in the window used for control.
            kb_Hit = my_Camera.Show_Image(display_Mode);

            // Check if there was a keyboard press during the display and if
            // so, whether to do anything about it.
            Keyboard_Handler(kb_Hit);
            #endif // HAVE_OPENCV

            step++;
            if (step == steps_To_Run){
                std::cout << "Requested number of tracker steps completed" << "\n";
                keep_Running = false;
            }

            if(tracker_Period > 0){
                // Toggle the tracker on/off to show the effect of the tracker
                // compared to if it wasn't active.
                if ((step / tracker_Period)%2 == 0){
                    if (tracker_On == false){
                        std::cout << "Tracker On" << "\n";
                        tracker_On = true;
                    }
                }
                else{
                    if (tracker_On == true){
                        std::cout << "Tracker Off" << "\n";
                        tracker_On = false;
                    }
                }
            }
            timer_Loop.Stop_Timer();
            //std::cout << "Timer: " << timer_Loop.Get_Last_Time_Difference() << "\n";

            // Push this loop's data into a std::list to output later.
            Fill_DataList(step);

        }

        // Loop's over, now save the std::list of the loop data to csv.
        Save_Datafile("tracker_Data.csv");

        return 0;
    }

    int Tracker::Get_Error(){
        // Spot position distance from set point.
        if(spot_Found){
            spot_Error.x = (spot_Coords.x - active_Setpoint.x);
            spot_Error.y = (spot_Coords.y - active_Setpoint.y);
        }
        else{
            spot_Error = {-1, -1};
        }
        return 0;
    }

    int Tracker::Fine_Track_Step(){

        timer_Camera.Start_Timer();
        my_Camera.Take_Picture();
        timer_Camera.Stop_Timer();
        // Maybe this should return a struct rather than returning a bool
        // and then the spot coords by reference?
        spot_Found = my_Camera.Spot_Finder(spot_Coords);

        // spot_Coords is nonsense if the spot wasn't found.
        if (spot_Found){
            no_Spot_Counter = 0;
            Get_Error();
            //std::cout << "Spot error: " << spot_Error.x << ", " << spot_Error.y << std::endl;

            // The tracker could be off for serveral reasons. The PID will go
            // crazy if the mirror can't move so just don't update it.
            // And obviously don't move the mirror either.
            if (tracker_On){
                timer_Mirror.Start_Timer();
                move_X = pid_X.UpdatePID(spot_Error.x);
                mirror_X += move_X;
                move_Y = pid_Y.UpdatePID(spot_Error.y);
                mirror_Y += move_Y;
                //std::cout << "Move by: " << move_X << ", " << move_Y << std::endl;
                //std::cout << "Mirror pos: " << mirror_X << ", " << mirror_Y << std::endl;

                my_Mirror.Move(mirror_X, mirror_Y);
                timer_Mirror.Stop_Timer();
            }

            if (use_AOI && !my_Camera.aoi_Set){
                // If AOI is required, check if the spot fulfils the criteria
                // for enabling the AOI.
                spot_Inside_AOI = (std::abs(spot_Error.x) < aoi_Boundary_X) &&
                                  (std::abs(spot_Error.y) < aoi_Boundary_Y);
                // And if so, do it.
                // TODO: Does anything else need updating here?
                if (spot_Inside_AOI){
                    my_Camera.Enable_AOI();
                    Set_Setpoint(aoi_Setpoint);
                }
            }
        }
        else{
            std::cout << "Spot not found" << "\n";
            no_Spot_Counter++;

            my_Camera.Save_Picture("Error.bin", true);
            if (my_Camera.aoi_Set){
                // If the AOI is on, the spot may have just fallen off it, try
                // disabling the AOI and seeing if the spot is on the full
                // frame.
                my_Camera.Disable_AOI();
                Set_Setpoint(full_Setpoint);
                // TODO: Enact a search pattern (lissajous figure?) if the spot
                // isn't on the full frame either. At least reset the mirror to
                // the origin.
            }
            if (no_Spot_Counter > 10){
                std::cout << "Resetting mirror" << std::endl;
                mirror_X = 0;
                mirror_Y = 0;
                my_Mirror.Move(mirror_X, mirror_Y);
                no_Spot_Counter = 0;
            }

        }

        return 0;
    }

    int Tracker::Live_Camera(){
        #ifdef HAVE_OPENCV
        // Just show the raw image from the camera and don't do anything else.
        while(true){
            my_Camera.Take_Picture();
            //my_Camera.Save_Picture("test.bin", true);
            my_Camera.Show_Image(0);
        }
        #endif //HAVE_OPENCV
        return 0;
    }

    int Tracker::Keyboard_Handler(int kb_Hit){
            switch (kb_Hit){
            case 255: // no key
                // default, don't check any of the other keys.
                break;
            case 113: // q
                std::cout << "\'q\' pressed while tracking. quitting" << "\n";
                keep_Running = false;
                break;
            case 116: // t
                if (tracker_On){
                    std::cout << "Turning tracker off. \'t\' to turn back on" << "\n";
                }
                else{
                    std::cout << "Turning tracker on. \'t\' to turn off" << "\n";
                }
                tracker_On = !tracker_On;
                break;
            case 81: // left
            case 82: // up
            case 83: // right
            case 84: // down
                if (tracker_On){
                    std::cout << "Mirror under closed loop control, ignoring arrow keys" << "\n";
                    std::cout << "press \'t\' to take control of the mirror" << "\n";
                }
                else{
                    Keyboard_Mirror(kb_Hit);
                }
                break;
        }
        return 0;
    }

    int Tracker::Keyboard_Mirror(int kb_Hit){
        // Move the mirror based on a valid arrow key press.
        switch (kb_Hit){
            case 81:
                std::cout << "Left" << "\n";
                mirror_X -= mirror_Increment;
                break;
            case 82:
                std::cout << "Up" << "\n";
                mirror_Y += mirror_Increment;
                break;
            case 83:
                std::cout << "Right" << "\n";
                mirror_X += mirror_Increment;
                break;
            case 84:
                std::cout << "Down" << "\n";
                mirror_Y -= mirror_Increment;
                break;
        }

        return my_Mirror.Move(mirror_X, mirror_Y);
    }

    int Tracker::Fill_DataList(uint64_t step_Number){
        // All information I can think of that's worth outputting per cycle
        // of the tracker.
        tracker_Data.push_back({
            step_Number,
            spot_Coords.x,
            spot_Coords.y,
            spot_Error.x,
            spot_Error.y,
            mirror_X,
            mirror_Y,
            tracker_On,
            spot_Found,
            my_Camera.aoi_Set,
            timer_Camera.Get_Last_Time_Difference(),
            timer_Mirror.Get_Last_Time_Difference(),
            timer_Loop.Get_Last_Time_Difference()
        });
        return 0;
    }

    int Tracker::Save_Datafile(std::string filename){
            std::cout << "Saving data file" << "\n";
            std::ofstream tracker_Data_File(filename);
            tracker_Data_File << "Step, Spot X, Spot Y, Error X, Error Y, Mirror X, Mirror Y, Tracker On?, Spot Found?, AOI on?, t_Camera, t_Mirror, t_Loop\n";
            for(auto data_Step : tracker_Data){
                tracker_Data_File << data_Step.step_Number << ", " <<
                    data_Step.spot_X << ",  " <<
                    data_Step.spot_Y << ", "  <<
                    data_Step.error_X << ", " <<
                    data_Step.error_Y << ", " <<
                    data_Step.mirror_X << ", " <<
                    data_Step.mirror_Y << ", " <<
                    data_Step.is_Tracking << ", " <<
                    data_Step.spot_Found << ", " <<
                    data_Step.is_AOI << ", " <<
                    data_Step.time_Camera << ", " <<
                    data_Step.time_Mirror << ", " <<
                    data_Step.time_Loop
                    << "\n";
        }
        tracker_Data_File.close();
        return 0;
    }

} // namespace DaveTracker2


