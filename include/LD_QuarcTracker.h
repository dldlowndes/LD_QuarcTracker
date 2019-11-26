#ifndef LD_QUARCTRACKER_H
#define LD_QUARCTRACKER_H

#include "LD_Camera.h"
#include "LD_TrackerCamera.h"
#include "LD_MemsMirror.h"
#include "LD_Util.h"
#include "LD_Pid.h"
#include "LD_Timer.h"

#include <list>

namespace LD_QuarcTracker{

    struct TrackerOptions{
        // Recommended, speeds up tracker a lot.
        bool do_AOI;

        // Proportion into the AOI region the spot should be before triggering
        // the AOI to be engaged (to stop a spot near the boundary repeatedly
        // toggling the AOI on/off.
        float aoi_Threshold;

        // Turn tracker on/off with this period to characterise performance.
        // Ignored if 0;
        int tracker_Period;

        LD_QuarcTracker::SpotFinderOptions full_Spot_Finder;
        LD_QuarcTracker::SpotFinderOptions aoi_Spot_Finder;

        LD_Camera::Pixel_Values full_Setpoint;
        LD_Camera::Pixel_Values aoi_Setpoint;

        // PID parameters for the mirror control.
        PIDParams full_PID_Options;
        PIDParams aoi_PID_Options;

        int display_Mode;
    };

    struct APTOutput{
        uint64_t step_Number;
        float spot_X;
        float spot_Y;
        float error_X;
        float error_Y;
        float mirror_X;
        float mirror_Y;
        bool is_Tracking;
        bool spot_Found;
        bool is_AOI;
        int time_Camera;
        int time_Mirror;
        int time_Loop;
    };

    struct APTOptions{
        TrackerOptions tracker_Options;
        LD_Camera::CameraOptions camera_Options;
        LD_MemsMirror::MirrorOptions mirror_Options;
    };

    APTOptions Load_Ini_Files(std::string tracker_Ini_Filename);

    class Tracker{
        public:
            Tracker();
            Tracker(APTOptions my_Options);
            ~Tracker();
            int Init(APTOptions my_Options);

            // Take one image, move mirror once
            int Fine_Track_Step();

            // Run the fine tracker in a loop. If image displa is on, also
            // monitor the keyboard events from the opencv window to interpret.
            int Fine_Tracker(uint64_t steps_To_Run = 0);

            // Just show the camera feed and nothing else.
            int Live_Camera();

            // Get/Set the set point for the tracker. this is the co-ordinates
            // on the full frame image the tracker will try to get the spot to
            // and then the centre of the AOI once the spot is close enough.
            LD_Camera::Pixel_Values Get_Setpoint();
            int Set_Setpoint(LD_Camera::Pixel_Values new_Setpoint);

        private:
            // Get the spot error based on the spot finder co-ordinates and the
            // set point co-ordinates.
            int Get_Error();

            // The opencv window registers keypresses while it is open (and in
            // focus!). 'q'=quit, 't'=toggle tracker, arrows=move mirror if
            // tracker is toggled off.
            int Keyboard_Handler(int kb_Hit);
            int Keyboard_Mirror(int kb_Hit);

            // Every step, add a line of tracker output data to a list.
            int Fill_DataList(uint64_t step_Number);
            // Save the tracker output data to a file. Do this after the loop
            // is finished to avoid slowing down the loop by worrying about
            // disk I/O mid loop. (possible target for threading?)
            int Save_Datafile(std::string filename);

            TrackerCamera my_Camera;
            // Separate setpoints for the full frame and AOI. The AOI setpoint
            // will just be the middle of the AOI (that's the point of setting
            // the AOI) but the full set point could theoretically be anywhere
            // in the frame if required. For example to correct for a small
            // misalignment in the APT optics.
            LD_Camera::Pixel_Values full_Setpoint;
            LD_Camera::Pixel_Values aoi_Setpoint;
            // This is set when the AOI is toggled to the relevant value.
            LD_Camera::Pixel_Values active_Setpoint;
            // If a spot is found by the spot finder, this is the co-ordinates.
            LD_Camera::Subpixel_Values spot_Coords;
            // Distance from the found spot to the set point.
            LD_Camera::Subpixel_Values spot_Error;

            // The Mirrorcle MEMS mirror performing the fine tracking.
            LD_MemsMirror::Mirror my_Mirror;

            // The PID loop mediating the closed loop control may need
            // different settings for the full frame and the AOI since
            // the loop bandwidth is dramatically different.
            PIDParams full_PID_Options;
            PIDParams aoi_PID_Options;

            // Separate PID for x and y (although should this be compressed
            // into a PID_2D class?)
            PIDLoop pid_X;
            PIDLoop pid_Y;

            // Track the position of the mirror.
            float mirror_X = 0;
            float mirror_Y = 0;
            float move_X = 0;
            float move_Y = 0;
            // Amount to move mirror by when controlling with arrow keys.
            float mirror_Increment = 0.01;

            // Whether to enable the camera AOI when the spot is close to the
            // set point. (much faster!)
            bool use_AOI;
            // Did the spot detector find a spot last time it ran?
            bool spot_Found = false;
            int no_Spot_Counter = 0;
            // Is the spot inside the AOI? Updated every loop
            bool spot_Inside_AOI = false;
            // Is the tracker on. If not, the camera and spot finder still run
            // but the PIDs are not updated and the mirror is not moved.
            bool tracker_On = true;
            // Condition for the tracker while loop to keep running.
            bool keep_Running = true;
            // Turn the tracker off/on regularly to monitor the efficacy of
            // the tracking (by giving you something to compare it to.
            // (Doesn't do anything if =0)
            int tracker_Period;
            // display modes:
            //  0: just show the image.
            //  1: draw a circle around the detected spot
            //  2: also draw a circle at the set point
            //  3: also draw a rectagle at the AOI (if not set)
            int display_Mode;
            // OpenCV window (if enabled) logs keypresses, we can use these for
            // a poor man's UI for now.
            int kb_Hit;

            // Threshold distance from the set point that should (if enabled)
            // activate the camera AOI.
            int aoi_Boundary_X;
            int aoi_Boundary_Y;

            // Separate options for the spot finder for AOI and full frame
            // since exposure etc could be different and certainly there is
            // less systematic noise on the AOI because the field is smaller.
            SpotFinderOptions my_Spot_Finder;
            SpotFinderOptions my_Spot_Finder_AOI;

            // Save output of each step in a list rather than worrying about
            // file I/O all the time. Dump it to file on quit.
            std::list<APTOutput> tracker_Data;

            // Timing various loop steps.
            LD_Timer timer_Loop;
            LD_Timer timer_Camera;
            LD_Timer timer_Mirror;

    };
} // namespace LD_QuarcTracker

#endif // LD_QUARCTRACKER_H
