#include <fstream>
#include <iostream>


#include "LD_Camera.h"

namespace LD_Camera{
    int FindCameras(){
        int num_Cameras = 0;
        is_GetNumberOfCameras(&num_Cameras);
        std::cout << num_Cameras << " cameras found" << "\n";

        // From the API manual docs for function is_GetCameraList()
        UEYE_CAMERA_LIST* pucl;
        pucl = (UEYE_CAMERA_LIST*) new  BYTE [sizeof(DWORD) + (num_Cameras * sizeof(UEYE_CAMERA_INFO))];
        pucl->dwCount = num_Cameras;
        if(is_GetCameraList(pucl) == IS_SUCCESS){
            for(int iCamera = 0; iCamera < (int)pucl->dwCount; iCamera++){
                std::cout << "Camera: " << iCamera <<
                            ", ID: " << pucl->uci[iCamera].dwCameraID <<
                            ", Model: " << pucl->uci[iCamera].FullModelName <<
                            ", Serial: " << pucl->uci[iCamera].SerNo << std::endl;
            }
        }
        return num_Cameras;
    }

    Camera::Camera()
    {
        //ctor
    }

    Camera::Camera(CameraOptions my_Options)
    {
        Init(my_Options);
    }

    Camera::~Camera()
    {
        //dtor
    }

    int Camera::Init(CameraOptions my_Options){
        std::cout << "Opening camera ID: " << my_Options.Camera_ID << "\n";
        m_hG = (HCAM)my_Options.Camera_ID;
        m_Ret = is_InitCamera(&m_hG, NULL); // NULL for window handle since we are using DIB mode.

        std::cout << "Connect to camera says: " << m_Ret << "\n";
        if (m_Ret == IS_SUCCESS){
            is_Connected = true;
        }
        else{
            std::cout << "########## FAIL. Incorrect camera ID? ##########" << "\n";
            return 1;
        }

        // retrieve original image size. If this fails assume 1280x1024.
        m_Ret = is_GetSensorInfo(m_hG, &sInfo);
        if (m_Ret != IS_SUCCESS){
            sInfo.nMaxWidth = 1280;
            sInfo.nMaxHeight = 1024;
        }

        // Define the full frame as a massive AOI.
        full_Frame.aoi_Position = {0,0};
        full_Frame.aoi_Size = {(int)sInfo.nMaxWidth, (int)sInfo.nMaxHeight};

        // Set the default AOI from the options file.
        default_AOI.aoi_Size = my_Options.aoi_Dimensions.aoi_Size;
        default_AOI.aoi_Position = my_Options.aoi_Dimensions.aoi_Position;
        my_AOI = default_AOI;

        // Camera does not allow arbitary AOIs, get the allowed increments.
        is_AOI(m_hG, IS_AOI_IMAGE_GET_POS_INC, (void*)&aoi_Position_Increment, sizeof(aoi_Position_Increment));
        is_AOI(m_hG, IS_AOI_IMAGE_GET_SIZE_INC, (void*)&aoi_Size_Increment, sizeof(aoi_Size_Increment));

        // Set camera modes:
        // DIB is the sort of raw low level access we want.
        is_SetDisplayMode(m_hG, IS_SET_DM_DIB);
        m_nBitsPerPixel = my_Options.bits_Per_Pixel;
        // TODO: Check whether camera is USB3 (10 bit is USB3 only).
        if (m_nBitsPerPixel > 8){
            std::cout << "Setting colour mode to MONO16" << "\n";
            m_Ret = is_SetColorMode(m_hG, IS_CM_MONO16);
        }
        else{
            std::cout << "Setting colour mode to MONO8" << "\n";
            m_Ret = is_SetColorMode(m_hG, IS_CM_MONO8);
        }
        std::cout << "Set colour mode says: " << m_Ret << "\n";

        // Rather than free running. Quite a bit faster!
        is_SetExternalTrigger(m_hG, IS_SET_TRIGGER_SOFTWARE);

        // Allocate image memories.
        is_AllocImageMem(m_hG, sInfo.nMaxWidth, sInfo.nMaxHeight, m_nBitsPerPixel,
                            &frame_Memory.buffer, &frame_Memory.id);
        is_AllocImageMem(m_hG, my_AOI.aoi_Size.s32Width, my_AOI.aoi_Size.s32Height, m_nBitsPerPixel,
                            &aoi_Memory.buffer, &aoi_Memory.id);
        // set memory active
        is_SetImageMem(m_hG, frame_Memory.buffer, frame_Memory.id);

        // Make some vectors to copy the horrible image memories to.
        // TODO: experiment if this can be done directly by passing
        // vector.data instead of &Camera_Memory.buffer...
        full_Image_Data.resize(full_Frame.aoi_Size.s32Width * full_Frame.aoi_Size.s32Height, 0);
        aoi_Image_Data.resize(my_AOI.aoi_Size.s32Width * my_AOI.aoi_Size.s32Height, 0);

        // Gain settings perform strangely, let's just leave it at the max for now.
        int max_Gain = is_SetHWGainFactor(m_hG, IS_INQUIRE_MASTER_GAIN_FACTOR, 100);
        std::cout << "Max gain: " << max_Gain << "\n";
        is_SetHWGainFactor(m_hG, IS_SET_MASTER_GAIN_FACTOR, max_Gain);
        int gain_Actual = is_SetHWGainFactor(m_hG, IS_SET_MASTER_GAIN_FACTOR, 100);
        std::cout << "Gain set to: " << gain_Actual << "\n";

        // Set exposure settings to default.
        default_Exposure = my_Options.exposure_Full;
        full_Frame_Exposure = default_Exposure;
        default_AOI_Exposure = my_Options.exposure_AOI;
        aoi_Exposure = default_AOI_Exposure;
        Set_Exposure(default_Exposure);

        is_Initted = true;
        std::cout << "Camera ready" << "\n";

        return 0;
    }

    AOI Camera::Get_Sensor_Size(){
        return full_Frame;
    }

    Exposure Camera::Get_Exposure(){
        return current_Exposure;
    }

    int Camera::Set_Exposure(Exposure new_Exposure){
        // Set pixel clock.
        std::cout << "Requesting " << new_Exposure.pixel_Clock << "MHz pixel clock" << "\n";
        int pix_Ret = is_PixelClock(m_hG, IS_PIXELCLOCK_CMD_SET, (void*)&new_Exposure.pixel_Clock, sizeof(new_Exposure.pixel_Clock));
        std::cout << "\tPixel clock says: " << pix_Ret << "\n";
        if (pix_Ret == IS_SUCCESS){
            current_Exposure.pixel_Clock = new_Exposure.pixel_Clock;
        }

        // Frame rate has specific values it can be so is_SetFrameRate returns the actual value it set to.
        double new_FPS;
        std::cout << "Requesting " << new_Exposure.frame_Rate << "FPS" << "\n";
        int frame_Ret = is_SetFrameRate(m_hG, new_Exposure.frame_Rate, &new_FPS);
        std::cout << "\tFrame set says: " << frame_Ret << " set to " << new_FPS << "\n";
        if (frame_Ret == IS_SUCCESS){
            current_Exposure.frame_Rate = new_FPS;
        }

        // Set exposure
        std::cout << "Requesting " << new_Exposure.exposure << "ms exposure" << "\n";
        int exp_Ret = is_Exposure(m_hG, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&new_Exposure.exposure, sizeof(new_Exposure.exposure));
        std::cout << "\tExposure says: " << exp_Ret << "\n";
        if (exp_Ret == IS_SUCCESS){
            current_Exposure.exposure = new_Exposure.exposure;
        }

        if (aoi_Set){
            aoi_Exposure = current_Exposure;
        }
        else{
            full_Frame_Exposure = current_Exposure;
        }

        return pix_Ret & frame_Ret & exp_Ret;
    }

    int Camera::Reset_Exposure(){
        if (aoi_Set){
            return Set_Exposure(default_AOI_Exposure);
        }
        else{
            return Set_Exposure(default_Exposure);
        }
    }

    int Camera::Take_Picture(){
        //std::cout << "Take picture" << "\n";
        if (is_Connected && is_Initted){
            // Capture image into memory buffer.
            m_Ret = is_FreezeVideo(m_hG, IS_WAIT);
            if (m_Ret != IS_SUCCESS) {
                std::cout << "Fail. Camera says: " << m_Ret << "\n";
                return 1;
            }

            if(aoi_Set){
                Copy_Memory(aoi_Memory, aoi_Image_Data);
            }
            else{
                Copy_Memory(frame_Memory, full_Image_Data);
            }
        }
        else{
            std::cout << "Camera not connected or not initialized" << "\n";
            return 1;
        }
        return 0;
    }

    const std::vector<uint16_t>& Camera::Get_Picture(){
        Take_Picture();

        if(aoi_Set){
            return aoi_Image_Data;
        }
        else{
            return full_Image_Data;
        }
    }

    int Camera::Copy_Memory(Camera_Memory &my_Buffer, std::vector<uint16_t> &my_Dest_Vector){
        //std::cout << "Copy memory" << "\n";
        if (m_nBitsPerPixel > 8){
            uint16_t first_Byte;
            uint16_t second_Byte;
            uint16_t shift = 16 - m_nBitsPerPixel;
            for(unsigned  int i=0; i < my_Dest_Vector.size() * 2; i+=2){
                first_Byte = (uint8_t)my_Buffer.buffer[i];
                second_Byte = (uint8_t)my_Buffer.buffer[i+1];

                my_Dest_Vector[i/2] = ((first_Byte << 0) + (second_Byte << 8)) >> shift;
            }
        }
        else{
            for(unsigned int i=0; i < my_Dest_Vector.size(); i++){
                my_Dest_Vector[i] = (uint8_t)my_Buffer.buffer[i];
            }
        }
        return 0;
    }

    int Camera::Save_Picture(std::string filename, bool binary){
        if(aoi_Set == false){
            if (binary){
                return Dump_Binary(full_Image_Data, filename);
            }
            else{
                return To_CSV(full_Image_Data, filename, full_Frame.aoi_Size.s32Width);
            }
        }
        else{
            if (binary){
                return Dump_Binary(aoi_Image_Data, filename);
            }
            else{
                return To_CSV(aoi_Image_Data, filename, my_AOI.aoi_Size.s32Width);
            }
        }
    }

    int Camera::To_CSV(std::vector<uint16_t> &image_Vector, std::string file_Name, int line_Length){
        std::ofstream file_Out(file_Name, std::ofstream::out);

        std::cout << "Saving " << image_Vector.size() << " pixels" << "\n";

        for(unsigned int i=0; i < image_Vector.size(); i++){
            file_Out << image_Vector[i];
            if(((i+1) % line_Length == 0) && (i != 0)){
                file_Out << "\n";
            }
            else{
                file_Out << ",";
            }
        }

        file_Out.close();
        return 0;
    }

    int Camera::Dump_Binary(std::vector<uint16_t> &image_Vector, std::string file_Name){
        std::ofstream file_Out(file_Name, std::ofstream::binary);

        std::cout << "Saving " << image_Vector.size() << " pixels" << "\n";

        for(auto pixel : image_Vector){
            file_Out << (uint8_t)((pixel >> 8) & 0xFF);
            file_Out << (uint8_t)(pixel & 0xFF);
        }

        file_Out.close();
        return 0;
    }

    AOI Camera::Get_Active_Frame_Info(){
        AOI this_AOI;
        m_Ret = is_AOI(m_hG, IS_AOI_IMAGE_GET_POS, (void*)&this_AOI.aoi_Position, sizeof(this_AOI.aoi_Position));
        m_Ret = is_AOI(m_hG, IS_AOI_IMAGE_GET_SIZE, (void*)&this_AOI.aoi_Size, sizeof(this_AOI.aoi_Size));

        //std::cout << "Current frame at: " << this_AOI.aoi_Position.s32X << ", " << this_AOI.aoi_Position.s32Y << "\n";
        //std::cout << "and size is:" << this_AOI.aoi_Size.s32Width << ", " << this_AOI.aoi_Size.s32Height << "\n";

        return this_AOI;
    }

    AOI Camera::Get_AOI_Info(){
        return my_AOI;
    }

    int Camera::Set_AOI_Info(AOI new_AOI){
        // Check the requested AOI conforms to the constraints of size
        // and position increments.
        new_AOI.aoi_Position.s32X = new_AOI.aoi_Position.s32X - (new_AOI.aoi_Position.s32X % aoi_Position_Increment.s32X);
        new_AOI.aoi_Position.s32Y = new_AOI.aoi_Position.s32Y - (new_AOI.aoi_Position.s32Y % aoi_Position_Increment.s32Y);
        new_AOI.aoi_Size.s32Height = new_AOI.aoi_Size.s32Height - (new_AOI.aoi_Size.s32Height % aoi_Size_Increment.s32Height);
        new_AOI.aoi_Size.s32Width = new_AOI.aoi_Size.s32Width - (new_AOI.aoi_Size.s32Width % aoi_Size_Increment.s32Width);

        my_AOI = new_AOI;

        // Reallocate memory for this image buffer
        // TODO: only do this if my_AOI.aoi_Size != new_AOI.aoi_Size
        is_AllocImageMem(m_hG, my_AOI.aoi_Size.s32Width, my_AOI.aoi_Size.s32Height, m_nBitsPerPixel,
                    &aoi_Memory.buffer, &aoi_Memory.id);
        // and reallocate the vector container too.
        aoi_Image_Data.resize(my_AOI.aoi_Size.s32Width * my_AOI.aoi_Size.s32Height, 0);
        return 0;
    }

    int Camera::Set_AOI_Info(AOI new_AOI, Exposure new_Exposure){
        Set_AOI_Info(new_AOI);
        aoi_Exposure = new_Exposure;
        return 0;
    }

    int Camera::Change_AOI(AOI &new_AOI){
        // Setting using an IS_RECT object (I hope) ensures that errors from
        // resize then move or move then resize sometimes not working because
        // the first command overflows the AOI off the sensor.
        IS_RECT temp_New_AOI;
        temp_New_AOI.s32Height = new_AOI.aoi_Size.s32Height;
        temp_New_AOI.s32Width = new_AOI.aoi_Size.s32Width;
        temp_New_AOI.s32X = new_AOI.aoi_Position.s32X;
        temp_New_AOI.s32Y = new_AOI.aoi_Position.s32Y;

        // Set the AOI.
        m_Ret = is_AOI(m_hG, IS_AOI_IMAGE_SET_AOI, (void*)&temp_New_AOI, sizeof(temp_New_AOI));
        std::cout << "Set AOI says: " << m_Ret << "\n";

        // Activate relevant image memory based on whether this was called by
        // Enable_AOI or Disable_AOI.
        if (aoi_Set){
            is_SetImageMem(m_hG, aoi_Memory.buffer, aoi_Memory.id);
        }
        else{
            is_SetImageMem(m_hG, frame_Memory.buffer, frame_Memory.id);
        }
        return 0;
    }

    int Camera::Enable_AOI(){
        std::cout << "Putting AOI at: " << my_AOI.aoi_Position.s32X << ", " << my_AOI.aoi_Position.s32Y << std::endl;
        aoi_Set = true;
        Change_AOI(my_AOI);
        Set_Exposure(aoi_Exposure);
        return 0;
    }

    int Camera::Disable_AOI(){
        aoi_Set = false;
        Set_Exposure(full_Frame_Exposure);
        Change_AOI(full_Frame);
        return 0;
    }

} // namespace LD_Camera

int Test_AOI(LD_Camera::CameraOptions my_Options){
    LD_Camera::Camera my_Camera(my_Options);

    my_Camera.Enable_AOI();
    my_Camera.Take_Picture();
    my_Camera.Save_Picture("images/aoi.dat", true);

    my_Camera.Disable_AOI();
    my_Camera.Reset_Exposure();
    my_Camera.Take_Picture();
    my_Camera.Save_Picture("images/full.dat", true);
    return 0;
}

int Test_Multi(LD_Camera::CameraOptions my_Options){
    LD_Camera::Camera my_Camera(my_Options);

    for(int i=0; i<50; i++){
        my_Camera.Take_Picture();
        my_Camera.Save_Picture("images/frame"+std::to_string(i)+".dat", true);
    }
    return 0;
}
