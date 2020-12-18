/*
 * @Author: your name
 * @Date: 2020-12-15 20:37:54
 * @LastEditTime: 2020-12-18 21:12:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /RealsenseTest/Realsense.hpp
 */
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace rs2;

#define RED 1
#define BLUE 2
#define NORMAL 3

class Realsense
{
   private:
      pipeline pipe;
      config pipe_config;
      
   public:
      int Width = 1920;
      int Height = 1080;
      void init()
      {
         pipe_config.enable_stream(RS2_STREAM_COLOR,Width,Height,RS2_FORMAT_BGR8,30);
         pipeline_profile profile = pipe.start(pipe_config);
      }


      Mat getimg()
      {
         frameset frameset = pipe.wait_for_frames();
         frame color_frame = frameset.get_color_frame();

         const int color_w=color_frame.as<video_frame>().get_width();
         const int color_h=color_frame.as<video_frame>().get_height();

         Mat color_image(Size(color_w,color_h),
                        CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);
         return color_image;
      }
      void setSize(int a, int b)
      {  
         Width = a;
         Height = b;
      }
      void setSensor(int mode)
      {
         sensor RGB_Sensor = pipe.get_active_profile().get_device().query_sensors()[1];
         RGB_Sensor.set_option(rs2_option::RS2_OPTION_SATURATION, 100);
         switch(mode)
         {
         case(1):

            RGB_Sensor.set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, 2800);
            break;
         
         case(2):
         
            RGB_Sensor.set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, 6500);
            break;
         
         case(3):
            
            RGB_Sensor.set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, 4500);
            RGB_Sensor.set_option(rs2_option::RS2_OPTION_SATURATION, 60);
         
         default:
            RGB_Sensor.set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, 4500);
            RGB_Sensor.set_option(rs2_option::RS2_OPTION_SATURATION, 60);
         }

      }
};
