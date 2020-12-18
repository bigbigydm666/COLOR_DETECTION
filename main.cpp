/*
 * @Author: your name
 * @Date: 2020-12-15 17:01:23
 * @LastEditTime: 2020-12-18 21:12:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /RSDRIVER/main.cpp
 */
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include "Realsense.hpp"
using namespace cv;
using namespace std;
using namespace rs2;





int main()
{
   vector<double> Poly3 = {-1.34330310272629e-12, 1.43114537288108e-07, -0.00559057589537007, 121.214140140806};
   vector<double> Poly2 = {0.00145491853262326, -0.864713874688413, 168.111430776990} ;

   namedWindow("control", 1);
   int hMax = 255, hMin = 136, sMax = 255, sMin = 0, vMax = 255, vMin = 0;
   int canny1 = 0, canny2 = 127;
   createTrackbar("hMax", "control", &hMax, 255);
   createTrackbar("hMin", "control", &hMin, 255);
   createTrackbar("sMax", "control", &sMax, 255);
   createTrackbar("sMin", "control", &sMin, 255);
   createTrackbar("vMax", "control", &vMax, 255);
   createTrackbar("vMin", "control", &vMin, 255);
   createTrackbar("canny1", "control", &canny1, 255);
   createTrackbar("canny2", "control", &canny2, 255);
   Realsense d435;
   d435.setSize(1920, 1080);
   d435.init();
   d435.setSensor(RED);
   Mat src;
   Mat temp = imread("1.png", IMREAD_GRAYSCALE);
   vector<Mat> temp_contours;
   temp_contours.push_back(temp);
   if(temp.empty())
   {
      cerr<<"could not load image"<<endl;
      exit(1);
   }
      vector<Vec3b> colors;  // 随机生成几种颜色
		

      for (int i = 0; i < 20; i++)
	   {
	      int b = theRNG().uniform(0, 255);
	      int g = theRNG().uniform(0, 255);
	      int r = theRNG().uniform(0, 255);
	      colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	   }

   while(true)
   {
      src = d435.getimg();

      medianBlur(src, src, 3);
      GaussianBlur(src, src, Size(5, 5), 0);
      Mat img = src.clone();
      
      //得到roi
      Rect ROIRECT = Rect(d435.Width/2 - 150, d435.Height/2 - 150, 300, 300);
      Mat roisrc = src(ROIRECT);
      rectangle(src, ROIRECT, Scalar(255, 255, 0));
      Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9));
      //在roi中进行特征提取
      




   
      
      
      vector<Mat> roiSplit;
      split(roisrc, roiSplit);
      Mat redFeature2 = roiSplit[2] + 10 - roiSplit[0];

      Mat GMask = 50 - roiSplit[1];
      Mat BMask =roiSplit[0] - 70; 
      Mat RMask = roiSplit[2] - 190;
      multiply(redFeature2, GMask, redFeature2);
      multiply(redFeature2, BMask, redFeature2);
      multiply(redFeature2, RMask, redFeature2);

      threshold(redFeature2, redFeature2, 1, 255, THRESH_BINARY);
      morphologyEx(redFeature2, redFeature2, MORPH_CLOSE, kernel);
      morphologyEx(redFeature2, redFeature2, MORPH_OPEN, kernel);

      //提取轮廓
      
      double maxVal;
      double minVal;
      minMaxLoc(redFeature2, &minVal, &maxVal);
      //redFeature2 -= 0.5* maxVal;
      //redFeature2 *= 255*2/0.5; 
      
      Mat roihsv;
      cvtColor(roisrc, roihsv, COLOR_BGR2HSV);
      Mat roisrc2 = roisrc.clone();
      Mat temp1, temp2;
      inRange(roihsv, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), temp1);
      //inRange(roihsv, Scalar(0, 112, 75), Scalar(0, 255, 220), temp2);
      Mat redFeature = temp1;
      morphologyEx(redFeature, redFeature, MORPH_CLOSE, kernel);
      vector<vector<Point>> contours;
      findContours(redFeature, contours, 0, 1);
      Rect minRec;
      double pro = 1;
      double min_pro = 999;
      int min_pro_loc = -1;
      if(!contours.empty())
      {
         drawContours(roisrc, contours, -1, Scalar(128, 128, 240), 2);

      
         for(int i = 0; i < contours.size(); i++)
         {
            if(contourArea(contours[i]) < 5000)   continue;
            minRec = boundingRect(contours[i]);
            if(minRec.width >= minRec.height)   continue;
            pro = matchShapes(contours[i], temp_contours[0], CONTOURS_MATCH_I1, 0);
            if(pro < min_pro)
            {
               min_pro = pro;
               min_pro_loc = i;
            }
            //cout<<pro<<' '<<min_pro<<endl;
         }
      }
      
      
      
      
      
      
      
      
      
      
      if(min_pro_loc>= 0)
      {  
         minRec = boundingRect(contours[min_pro_loc]);
         rectangle(roisrc, minRec, Scalar(0, 255, 0), 2);
         ostringstream oss;
         oss <<"Size: " << contourArea(contours[min_pro_loc]) << "Height: " << minRec.height;
         string text = oss.str();
         putText(src, text, Point(50,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 23, 0), 4, 8);
         double Size = double(contourArea(contours[min_pro_loc]));
         double H = double(minRec.height);
         double DistanceBySize = Poly3[0] * pow(Size, 3) + Poly3[1] * pow(Size, 2) + Poly3[2] * Size + Poly3[3];
         double DistanceByHeight = Poly2[0] * pow(H, 2) + Poly2[1] * H + Poly2[2];
         ostringstream oss1;
         oss1 <<"Distance by Size:" << DistanceBySize;
         string text1 = oss1.str();
         putText(src, text1, Point(50,150), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 4, 8);
         ostringstream oss2;
         oss2 <<"Distance by Height:" << DistanceByHeight;
         string text2 = oss2.str();
         putText(src, text2, Point(50,250), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 4, 8);
      }
      
      
      
      
      
      
      
      
      int b = src.data[src.channels()*(src.cols*540 + 960) + 0];    
      int g = src.data[src.channels()*(src.cols*540 + 960) + 1];
      int r = src.data[src.channels()*(src.cols*540 + 960) + 2];
      circle(src, Point(960, 540), 1, Scalar(128, 121, 40));
      cout << r << ' ' << g << ' ' << b << endl;
      
      imshow("222", redFeature);
      imshow("333", redFeature2);
      imshow("111", src);


      waitKey(1); 
   }
   return 0;
}

