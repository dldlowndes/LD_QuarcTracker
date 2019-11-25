#ifndef TRACKERCAMERA_H
#define TRACKERCAMERA_H

#include "LD_Camera.h"

#define HAVE_OPENCV
#ifdef HAVE_OPENCV
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#endif // HAVE_OPENCV

namespace LD_QuarcTracker{

    struct SpotFinderOptions{
        double peak_Thresh;
        double gaussian_Thresh;
        int n_Peak_Pixels;
        bool save_Data;
    };

    class TrackerCamera : public LD_Camera::Camera{
        // Extends the LD_Camera basic camera wrapper for the uEye driver.
        // Adds a spot finder which assumes a single gaussian(ish) spot
        // somewhere in the frame.
        public:
            int Set_SpotFinder_Options(SpotFinderOptions options);
            // The actual spot finder.
            bool Spot_Finder(LD_Camera::Subpixel_Values& spot_Coords);
            #ifdef HAVE_OPENCV
            // Displays camera feed with various overlays depending on the
            // setting in display_Mode.
            int Show_Image(int display_Mode);
            #endif // HAVE_OPENCV

        private:
            // Project the image as histograms on the X and Y axis.
            int Project_Image_XY();
            // Save X&Y projections of camera image from spot finder.
            int Save_Projections();
            // Get weighted average X value of a distribution
            float Weighted_Average(std::vector<uint64_t> &distribution);

            #ifdef HAVE_OPENCV
            // Make an opencv mat from the image buffer vector so opencv can
            // understand it. Probaly a bit inefficient but the display should
            // be disabled in production when performance actually matters.
            int Make_Opencv_Mat(LD_Camera::AOI my_AOI);
            // Open a window and display the current myMat. (which may or may
            // not have been drawn on)
            int Display_Mat();
            // Draw features on myMat, if enabled, used to draw spots at the
            // set point and tracked spot position, and a rectangle around the
            // AOI.
            int Draw_Spot(LD_Camera::Subpixel_Values spot_Position, cv::Scalar colour);
            int Draw_AOI(LD_Camera::AOI my_AOI);

            cv::Mat myMat;
            #endif // HAVE_OPENCV

            // The image data as a 1D vector (thus you will need to know the
            // dimensions of the image to make sense of this.
            std::vector<uint16_t> image_Buffer;
            // For the spot finder, container for the projection of the spot
            // in X and Y.
            std::vector<uint64_t> row_Totals;
            std::vector<uint64_t> col_Totals;

            SpotFinderOptions my_Options;
            LD_Camera::Subpixel_Values my_Spot_Coords;

    };
} // namespace DaveTracker2

#endif //TRACKERCAMERA_H
