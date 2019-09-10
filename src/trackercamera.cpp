#include "trackercamera.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace DaveTracker2{
    int TrackerCamera::Set_SpotFinder_Options(SpotFinderOptions options){
        my_Options = options;
        return 0;
    }

    bool TrackerCamera::Spot_Finder(DaveCamera2::Subpixel_Values& spot_Coords){
        // Project the image on the X and Y axes, this should look like two
        // gaussians (ish) if there is a single gaussian spot in the image
        // and n_Peak_Pixels will be relatively small (ie few bright pixels)
        int n_Peak_Pixels = Project_Image_XY();
        //std::cout << n_Peak_Pixels << " pixels above 50% brightness" << std::endl;

        // Figure out whether it looks like there is a peak in the data. The
        // expectation is that there won't be a very high proportion of the
        // pixels higher than the threshold considered for noise (since the
        // spot is a spot and not a massive blob.
        uint16_t min_Peak_Pixels = 5;
        if ((n_Peak_Pixels > my_Options.n_Peak_Pixels) |
            (n_Peak_Pixels < min_Peak_Pixels)){
            std::cout << "Error " << n_Peak_Pixels << " pixels above threshold, that's too many" << "\n";
            // Try and make it obvious this is an invalid value (better than
            // not updating the value if the spot isn't found)
            spot_Coords = {-1, -1};
            //Save_Projections();
            return false;
        }

        // Sometimes it's useful to be able to properly analyse this data
        // offline but likely this will just take time so mostly don't bother.
        if (my_Options.save_Data){
            Save_Projections();
        }

        // Sum of the rows is the projection in Y.
        spot_Coords.y = Weighted_Average(row_Totals);

        // Sum of the cols is the projection in X.
        spot_Coords.x = Weighted_Average(col_Totals);

        my_Spot_Coords = spot_Coords;
        // Spot was found.
        return true;
    }

    int TrackerCamera::Project_Image_XY(){
        // Find the size of the current frame (it's referred to as an AOI but
        // it could be the full frame, AOI is just a useful container for the
        // information. This allows the spot finder to interpret the image
        // buffer.
        DaveCamera2::AOI my_AOI;

        // Look at the correct image data from the camera.
        if(aoi_Set){
            my_AOI = Get_AOI_Info();
            image_Buffer = aoi_Image_Data;
        }
        else{
            my_AOI = Get_Sensor_Size();
            image_Buffer = full_Image_Data;
        }

        // Make sure the containers are the right size and set to zeros.
        if (row_Totals.size() != (unsigned int)my_AOI.aoi_Size.s32Height){
            row_Totals.resize(my_AOI.aoi_Size.s32Height);
        }
        std::fill(row_Totals.begin(), row_Totals.end(), 0);
        if (col_Totals.size() != (unsigned int)my_AOI.aoi_Size.s32Width){
            col_Totals.resize(my_AOI.aoi_Size.s32Width);
        }
        std::fill(col_Totals.begin(), col_Totals.end(), 0);

        // Get max value for the spot tracker. The spot tracker ignores pixels
        // which are lower than some threshold to remove noise and any
        // systematic illumination which adds up to being substantial when the
        // rows/cols are summed.
        uint16_t max_Pixel = *std::max_element(image_Buffer.begin(), image_Buffer.end());
        uint16_t peak_Thresh = max_Pixel * my_Options.peak_Thresh;
        uint16_t abs_Thresh = 50;

        // Sum the pixels in each row and column. This is a projection of
        // the image into 1D. For a laser spot this will result in a (roughly)
        // gaussian - especially when the low valued pixels are ignored.
        int row;
        int col;
        int numCols = my_AOI.aoi_Size.s32Width;
        int x = 0;
        int n_Peak_Pixels = 0;
        for(auto pixel : image_Buffer){
            // Try not to count noise.
            if((pixel > peak_Thresh) & (pixel > abs_Thresh)){
                // I'm quite proud of this short cut.
                row = std::floor(x / numCols);
                col = x % numCols;

                row_Totals[row] += pixel;
                col_Totals[col] += pixel;
                n_Peak_Pixels++;
            }
            // Even using a ranged for, still need to keep track of element
            // number :(
            x++;
        }

        return n_Peak_Pixels;
    }

    float TrackerCamera::Weighted_Average(std::vector<uint64_t> &distribution){
        uint64_t distribution_Max = *std::max_element(distribution.begin(), distribution.end());
        uint64_t gaussian_Thresh = distribution_Max * my_Options.gaussian_Thresh;

        int i = 0;
        uint64_t running_Total = 0;
        uint64_t running_Weights = 0;
        for(auto pixel : distribution){
            if (pixel > gaussian_Thresh){
                // Only consider significantly big enough values to make sure
                // low level noise is not skewing the result
                running_Total += (i * pixel);
                running_Weights += pixel;
            }
            i++;
        }
        // The centre of the distribution to subpixel accuracy.
        return ((float)running_Total) / ((float)running_Weights);
    }

    int TrackerCamera::Save_Projections(){
        std::ofstream rows_Out("rows_Out.csv");
        for(auto row : row_Totals){
            rows_Out << row << "\n";
        }
        rows_Out.close();

        std::ofstream cols_Out("cols_Out.csv");
        for(auto col : col_Totals){
            cols_Out << col << "\n";
        }
        cols_Out.close();
        std::cout << "Spots saved" << "\n";

        return 0;
    }

    #ifdef HAVE_OPENCV
    int TrackerCamera::Make_Opencv_Mat(DaveCamera2::AOI my_AOI){
        // Make a mat that looks a lot like the vector we had in the first
        // place. (but 2d!). Easiest to make it B&W then convert to colour.
        myMat = cv::Mat(my_AOI.aoi_Size.s32Height,
                        my_AOI.aoi_Size.s32Width,
                        CV_16UC1,
                        image_Buffer.data()).clone();
        // Display doesn't like 16 bit images, convert to 8 bit.
        myMat.convertTo(myMat, CV_8UC1);
        // Add some colour channels so we can draw on the image in colour.
        cv::cvtColor(myMat, myMat, cv::COLOR_GRAY2BGR);
        return 0;
    }

    int TrackerCamera::Display_Mat(){
        // Display the image and return the key that was pressed while the
        // image was displayed.
        cv::imshow("hello", myMat);
        return cv::waitKey(1);
    }

    int TrackerCamera::Draw_Spot(DaveCamera2::Subpixel_Values spot_Position, cv::Scalar colour){
        // Draw a spot on the currently stored mat containing the camera image.
        cv::Point spot_Centre((int)spot_Position.x, (int)spot_Position.y);
        cv::circle(myMat, spot_Centre, 15, colour, 2, 8, 0);
        return 0;
    }

    int TrackerCamera::Draw_AOI(DaveCamera2::AOI my_AOI){
        // Draw a rectangle corresponding to the AOI position in the full
        // frame.
        cv::Point top_Left(my_AOI.aoi_Position.s32X,
                           my_AOI.aoi_Position.s32Y);
        cv::Point bottom_Right (my_AOI.aoi_Position.s32X + my_AOI.aoi_Size.s32Width,
                                my_AOI.aoi_Position.s32Y + my_AOI.aoi_Size.s32Height);
        cv::rectangle(myMat, top_Left, bottom_Right, {255,0,0} , 2, 8, 0);
        return 0;
    }

    int TrackerCamera::Show_Image(int display_Mode){
        // display modes:
        // 0: just show the image.
        // 1: draw a circle around the detected spot
        // 2: also draw a circle at the set point
        // 3: also draw a rectagle at the AOI (if not set)

        // TODO: This is actually a call to the camera API, check the
        // performance implications of this.
        DaveCamera2::AOI current_AOI = Get_Active_Frame_Info();

        if(aoi_Set){
            image_Buffer = aoi_Image_Data;
        }
        else{
            image_Buffer = full_Image_Data;
        }

        // Need an opencv mat to pass to cv::imshow
        Make_Opencv_Mat(current_AOI);

        switch (display_Mode){
            // Note that this falls through all the way down. (hence the
            // reverse order of the cases)
            case 3:
                if(aoi_Set == false){
                    Draw_AOI(Get_AOI_Info());
                }
                [[fallthrough]];
            case 2:
                if (aoi_Set){
                    Draw_Spot({
                        ((float)current_AOI.aoi_Size.s32Width/2),
                        ((float)current_AOI.aoi_Size.s32Height/2)
                        }, {0, 255, 0});
                }
                else{
                    Draw_Spot({
                        current_AOI.aoi_Position.s32X + ((float)current_AOI.aoi_Size.s32Width/2),
                        current_AOI.aoi_Position.s32Y + ((float)current_AOI.aoi_Size.s32Height/2)
                        }, {0,255,0});
                }
                [[fallthrough]];
            case 1:
                Draw_Spot(my_Spot_Coords, {0,0,255});
                [[fallthrough]];
            case 0:
                // Just display the image with nothing else.
                break;

        }

        return Display_Mat();
    }
    #endif // HAVE_OPENCV
}
