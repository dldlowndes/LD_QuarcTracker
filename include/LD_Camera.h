#ifndef LDCAMERA_H
#define LDCAMERA_H


#include <string>
#include <vector>

// Silence the warnings we can't do anything about.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <uEye.h>
#pragma GCC diagnostic pop

namespace LD_Camera{
    int FindCameras();

    struct Pixel_Values{
        int x;
        int y;
    };

    struct Subpixel_Values{
        float x;
        float y;
    };

    struct Exposure{
        int pixel_Clock;
        double frame_Rate;
        double exposure;
    };

    struct AOI{
        IS_POINT_2D aoi_Position;
        IS_SIZE_2D aoi_Size;
    };

    struct CameraOptions{
        int Camera_ID;
        Exposure exposure_Full;
        Exposure exposure_AOI;
        int bits_Per_Pixel;
        AOI aoi_Dimensions;
    };

    struct Camera_Memory{
        int id;
        char* buffer;
    };

    class Camera
    {
        public:
            // Don't init the camera if you're not ready to.
            Camera();

            // Just calls Init(my_Options) directly.
            Camera(CameraOptions my_Options);
            ~Camera();

            // Connects to camera and sets up memory and parameters
            int Init(CameraOptions my_Options);

            // Return an AOI object which is defines the whole sensor.
            AOI Get_Sensor_Size();

            // Return the currently set pixel clock, frame rate and exposure,
            // hereby PCFRE
            Exposure Get_Exposure();

            // Set new values for PCFRE, persists for either full frame or AOI,
            // whichever is currently active
            int Set_Exposure(Exposure new_Exposure);

            // Set currently active exposure (either full or AOI) to the
            // one originally set when camera was initted.
            int Reset_Exposure();

            // Return the size of the AOI which would be set by Enable_AOI
            AOI Get_AOI_Info();

            // Update the AOI dimensions (but do not enable).
            int Set_AOI_Info(AOI new_AOI);

            // Update the AOI dimensions and exposure settings at the
            // same time.
            int Set_AOI_Info(AOI new_AOI, Exposure new_Exposure);

            // Turn on the AOI. Set the exposure to the previous exposure
            // when the AOI was set (or the default). To change AOI size
            // while AOI is set, just Set_AOI_Info then call Enable_AOI
            // again.
            int Enable_AOI();

            // Revert back to full frame and full frame exposure.
            int Disable_AOI();

            // Return an AOI object of the current frame (even if it is the
            // full frame)
            AOI Get_Active_Frame_Info();

            bool aoi_Set = false;

            // Fill the image buffer with the data from the sensor.
            int Take_Picture();

            // Return a linear vector of the pixel values. Calls Take_Picture
            // so anything that relies on the image being in memory still works
            const std::vector<uint16_t>& Get_Picture();

            // Save the most recent result from Take_Picture to file. Either as
            // a csv (including line breaks where relevant) or as a binary file
            // with 2 bytes per pixel.
            int Save_Picture(std::string filename, bool binary = true);

        protected:
            // Nice C++ STL containers for the image data.
            std::vector<uint16_t> full_Image_Data;
            std::vector<uint16_t> aoi_Image_Data;

        private:
            // Move from the image buffer into the vector container.
            int Copy_Memory(Camera_Memory &my_Buffer, std::vector<uint16_t> &my_Dest_Vector);

            // Save the vector as a CSV, insert a line break every line_Length.
            int To_CSV(std::vector<uint16_t> &image_Vector, std::string file_Name, int line_Length);

            // Plop out the vector data raw into a file (more space/time efficent)
            int Dump_Binary(std::vector<uint16_t> &image_Vector, std::string file_Name);

            // Called by Enable_AOI/Disable_AOI. Deals with activating
            // relevant image memories. Also checks that the new_AOI
            // conforms to
            int Change_AOI(AOI &new_AOI);

            // Handle to camera.
            HCAM m_hG;

            // From uEye.h, a bunch of stuff about the sensor that the camera returns
            SENSORINFO sInfo;

            // Somewhere for the uEye API to return to.
            int m_Ret;

            // Internally pixels are always stored as uint16_t for convenience,
            // but some things will need to know the exact number.
            int m_nBitsPerPixel;

            // Contains arrays for the camera API to dump image data into.
            Camera_Memory frame_Memory;
            Camera_Memory aoi_Memory;

            // Default objects are the ones set in options passed to Init()
            Exposure default_Exposure;
            Exposure default_AOI_Exposure;

            // Remember what the exposure was last time the full frame and AOI
            // were set respectively.
            Exposure full_Frame_Exposure;
            Exposure aoi_Exposure;

            // What is the exposure *right now* it should be one of
            // full_Frame_Exposure or aoi_Exposure.
            Exposure current_Exposure;

            // An AOI object which defines the full size of the frame.
            AOI full_Frame;
            // The AOI defined in the options passed to Init()
            AOI default_AOI;
            // The AOI which is actually set when Enable_AOI is called.
            // Is also what is got/set by Get/Set_AOI.
            AOI my_AOI;

            // Arbitary AOIs are not allowed, Set_AOI_Info(new_AOI) checks
            // that the desired AOI conforms to these constraints.
            IS_POINT_2D aoi_Position_Increment;
            IS_SIZE_2D aoi_Size_Increment;

            bool is_Connected = false;
            bool is_Initted = false;
    };
} // namespace LD_Camera

int Test_AOI(LD_Camera::CameraOptions my_Options);
int Test_Multi(LD_Camera::CameraOptions my_Options);

#endif // LDCAMERA_H
