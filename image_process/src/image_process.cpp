#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_process/Distance.h>

using namespace cv;
using namespace std;

Rect ROI_def(vector<Point> & squ);
cv_bridge::CvImagePtr cv_ptr;
ros::Publisher  pubDistance; 
vector<Point> squ_temp{Point(227,53),Point(226,54),Point(225,54),Point(224,54),Point(223,54),Point(222,54),Point(221,54),Point(220,54),Point(219,54),Point(218,54),Point(217,54),Point(216,54),Point(215,54),Point(214,54),Point(213,55),Point(212,55),Point(211,55),Point(210,55),Point(209,55),Point(208,55),Point(207,55),Point(206,55),Point(205,55),Point(204,55),Point(203,55),Point(202,55),Point(201,55),Point(200,55),Point(199,56),Point(198,56),Point(197,56),Point(196,56),Point(195,56),Point(194,56),Point(193,56),Point(192,56),Point(191,56),Point(190,56),Point(189,56),Point(188,56),Point(187,56),Point(186,57),Point(185,57),Point(184,57),Point(183,57),Point(182,57),Point(181,57),Point(180,57),Point(179,57),Point(178,57),Point(177,57),Point(176,57),Point(175,57),Point(174,57),Point(173,57),Point(172,58),Point(171,58),Point(170,58),Point(169,58),Point(168,58),Point(167,58),Point(166,58),Point(165,58),Point(164,58),Point(163,58),Point(162,58),Point(161,58),Point(160,58),Point(159,59),Point(158,59),Point(157,59),Point(156,59),Point(155,59),Point(154,59),Point(153,59),Point(152,59),Point(151,59),Point(150,59),Point(150,60),Point(150,61),Point(150,62),Point(150,63),Point(151,64),Point(151,65),Point(151,66),Point(151,67),Point(151,68),Point(151,69),Point(151,70),Point(151,71),Point(151,72),Point(151,73),Point(151,74),Point(151,75),Point(151,76),Point(152,77),Point(152,78),Point(152,79),Point(152,80),Point(152,81),Point(152,82),Point(152,83),Point(152,84),Point(152,85),Point(152,86),Point(152,87),Point(152,88),Point(152,89),Point(152,90),Point(153,91),Point(153,92),Point(153,93),Point(153,94),Point(153,95),Point(153,96),Point(153,97),Point(153,98),Point(153,99),Point(153,100),Point(153,101),Point(153,102),Point(153,103),Point(154,104),Point(154,105),Point(154,106),Point(154,107),Point(154,108),Point(154,109),Point(154,110),Point(154,111),Point(154,112),Point(154,113),Point(154,114),Point(154,115),Point(154,116),Point(154,117),Point(155,118),Point(155,119),Point(155,120),Point(155,121),Point(155,122),Point(155,123),Point(155,124),Point(155,125),Point(155,126),Point(155,127),Point(155,128),Point(155,129),Point(155,130),Point(156,131),Point(156,132),Point(156,133),Point(156,134),Point(156,135),Point(156,136),Point(156,137),Point(156,138),Point(156,139),Point(156,140),Point(156,141),Point(156,142),Point(156,143),Point(156,144),Point(157,145),Point(157,146),Point(158,146),Point(159,146),Point(160,146),Point(161,146),Point(162,146),Point(163,146),Point(164,146),Point(165,146),Point(166,145),Point(167,145),Point(168,145),Point(169,145),Point(170,145),Point(171,145),Point(172,145),Point(173,145),Point(174,145),Point(175,145),Point(176,145),Point(177,145),Point(178,145),Point(179,145),Point(180,144),Point(181,144),Point(182,144),Point(183,144),Point(184,144),Point(185,144),Point(186,144),Point(187,144),Point(188,144),Point(189,144),Point(190,144),Point(191,144),Point(192,144),Point(193,143),Point(194,143),Point(195,143),Point(196,143),Point(197,143),Point(198,143),Point(199,143),Point(200,143),Point(201,143),Point(202,143),Point(203,143),Point(204,143),Point(205,143),Point(206,143),Point(207,142),Point(208,142),Point(209,142),Point(210,142),Point(211,142),Point(212,142),Point(213,142),Point(214,142),Point(215,142),Point(216,142),Point(217,142),Point(218,142),Point(219,142),Point(220,141),Point(221,141),Point(222,141),Point(223,141),Point(224,141),Point(225,141),Point(226,141),Point(227,141),Point(228,141),Point(229,141),Point(230,141),Point(231,141),Point(232,141),Point(233,141),Point(234,140),Point(235,140),Point(236,140),Point(237,140),Point(238,140),Point(239,140),Point(240,140),Point(241,140),Point(242,140),Point(243,140),Point(243,139),Point(243,138),Point(243,137),Point(243,136),Point(243,135),Point(243,134),Point(243,133),Point(243,132),Point(243,131),Point(243,130),Point(243,129),Point(243,128),Point(243,127),Point(243,126),Point(243,125),Point(243,124),Point(243,123),Point(243,122),Point(243,121),Point(243,120),Point(242,119),Point(242,118),Point(242,117),Point(242,116),Point(242,115),Point(242,114),Point(242,113),Point(242,112),Point(242,111),Point(242,110),Point(242,109),Point(242,108),Point(242,107),Point(242,106),Point(241,105),Point(241,104),Point(241,103),Point(241,102),Point(241,101),Point(241,100),Point(241,99),Point(241,98),Point(241,97),Point(241,96),Point(240,95),Point(240,94),Point(240,93),Point(240,92),Point(240,91),Point(240,90),Point(240,89),Point(240,88),Point(240,87),Point(240,86),Point(240,85),Point(240,84),Point(240,83),Point(240,82),Point(240,81),Point(240,80),Point(240,79),Point(240,78),Point(240,77),Point(239,76),Point(239,75),Point(239,74),Point(239,73),Point(239,72),Point(239,71),Point(239,70),Point(239,69),Point(239,68),Point(239,67),Point(239,66),Point(239,65),Point(239,64),Point(239,63),Point(238,62),Point(238,61),Point(238,60),Point(238,59),Point(238,58),Point(238,57),Point(238,56),Point(237,55),Point(237,54),Point(237,53),Point(236,53),Point(235,53),Point(234,53),Point(233,53),Point(232,53),Point(231,53),Point(230,53),Point(229,53),Point(228,53)};

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   Mat origin_img, gray_img, binary_img;
   try
   {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
   
   origin_img=cv_ptr->image;
   if(!origin_img.empty())
   {
		resize(origin_img, origin_img, Size(320, 240));//原图大小为640×480,缩放为320×240
		cv::imshow("origin_img", origin_img);
		
		cvtColor(origin_img,gray_img, CV_BGR2GRAY);//将BGR8转换为灰度图
		threshold(gray_img, binary_img,160,255,THRESH_BINARY);//将灰度图二值化，将灰度值大于160的像素点设为255,其余为0
		imshow("binary_img", binary_img);      
		
		Mat element3(3, 3, CV_8U, Scalar(1));
		morphologyEx(binary_img, binary_img, MORPH_OPEN, element3);//采用3×3大小的核对图像进行形态学开操作（先腐蚀再膨胀）
		imshow("morph_open", binary_img);     
		

		vector<vector<Point> > _contours,contours;
		findContours(binary_img, _contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//寻找出所有轮廓
		for (vector<vector<Point> >::iterator vvPit = _contours.begin(); vvPit != _contours.end(); vvPit++)
		{
		   if (vvPit->size()>60)//筛选出较大轮廓
		   {
		      double match=matchShapes(squ_temp,*vvPit,CV_CONTOURS_MATCH_I2,0);
		      if(match<0.01)//与正方形模板轮廓进行匹配
		         contours.push_back(*vvPit);
		   }	   
		}
		
		//筛选正方形轮廓的调试程序
		/*int i=0;
		for (vector<vector<Point> >::iterator vvPit = contours.begin(); vvPit != contours.end(); vvPit++,i++)
		{
		   double match=matchShapes(squ_temp,*vvPit,CV_CONTOURS_MATCH_I2,0);
		   cout<<i<<":"<<match<<endl;
		   for(vector<Point>::iterator vPit=vvPit->begin();vPit != vvPit->end(); vPit++)
		      cout<<"Point("<<vPit->x<<","<<vPit->y<<"),";
		   cout<<endl;
		}*/

		//显示正方形轮廓
		if(contours.size()>0)
		{
			cv::Mat squ_contour(origin_img.size(),CV_8U,cv::Scalar(255));
			drawContours(squ_contour,contours,-1,cv::Scalar(0),2);
			imshow("squ_contour",squ_contour);
		
			for(vector<vector<Point> >::iterator vvPit=contours.begin();vvPit != contours.end(); vvPit++)
			{ 
				Rect rect=ROI_def(*vvPit);
				//显示ROI矩形轮廓
				rectangle(gray_img,rect,0);
				imshow("ROI",gray_img);
		
				Mat ROI_img=gray_img(rect);
				//cout<<"row="<<ROI_img.rows<<",column="<<ROI_img.cols<<endl;

				vector<Vec3f> circles;
				try
				{
					HoughCircles(ROI_img,circles,CV_HOUGH_GRADIENT,1, ROI_img.rows/2, 200, 60, rect.width/4, rect.width/2);	
				}
				catch(...)
				{
					cout<<"row="<<ROI_img.rows<<",column="<<ROI_img.cols<<endl;
					cout<<"HouthCircles error"<<endl;
				}
				for (size_t i = 0; i < circles.size(); i++)
				{
				   Point center(cvRound(circles[i][0])+rect.x, cvRound(circles[i][1])+rect.y);
				   int radius = cvRound(circles[i][2]);
				   circle(origin_img, center, 3, Scalar(0, 255, 0), -1, 8, 0);
				   circle(origin_img, center, radius, Scalar(0, 0, 255), 1, 8, 0);				
				}
				imshow("circle_img", origin_img);
				if(circles.size()==1)
				{
				   //cout<<circles[0][0]+rect.x<<endl;
				   //cout<<circles[0][1]+rect.y<<endl;
				   //cout<<circles[0][2]<<endl;
				   image_process::Distance dist;
				   dist.x_error=(120-(circles[0][1]+rect.y))/circles[0][2]*0.24;//0.24/radius(in pixel)=x_error/(120-circle[0][1])，飞机的前向对应图像y轴方向的误差，误差为正则飞机向前方飞行才能减小误差
				   dist.y_error=(circles[0][0]+rect.x-160)/circles[0][2]*0.24;//0.24/radius(in pixel)=x_error/(circle[0][0]-160)，飞机的右向对应图像x轴方向的误差，误差为正则飞机向右侧飞行才能减小误差
				   dist.height=50.04384/circles[0][2];// 0.24/height=radius(in pixel)/417.032(focal length in pixel) -> height=50.04384/radius
				   pubDistance.publish(dist);
				   break;
				}
			}
		}
	} 
   
   //在正方形轮廓中检测角点并显示
   /*Mat dst, dst_norm, dst_norm_scaled;
   dst = Mat::zeros(squ_contour.size(), CV_32FC1);
   int blockSize = 3;
   int apertureSize = 3;
   double k = 0.04;
   int thresh = 150;
   int cornerCount = 0;
   cornerHarris(squ_contour, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
   normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
   convertScaleAbs(dst_norm, dst_norm_scaled);
   for (int j = 0; j < dst_norm.rows; j++)
   {
      for (int i = 0; i < dst_norm.cols; i++)
      {
         if ((int)dst_norm.at<float>(j, i) > thresh)
         {
	    circle(origin_img, Point(i, j), 3, Scalar(0, 255, 0), -1, 8, 0);
	    cornerCount++;
	 }
      }
   }
   imshow("harris", origin_img);*/
   
   cv::waitKey(1);
}


Rect ROI_def(vector<Point> & squ)
{
   Rect rect;
   int x_max=squ.begin()->x,x_min=squ.begin()->x,y_max=squ.begin()->y,y_min=squ.begin()->y;
   for(vector<Point>::iterator vPit= squ.begin();vPit != squ.end(); vPit++)
   {
      if(vPit->x > x_max) x_max=vPit->x;
      else if(vPit->x < x_min) x_min=vPit->x;
      if(vPit->y > y_max) y_max=vPit->y;
      else if(vPit->y < y_min) y_min=vPit->y;
   }
   rect.x=x_min;
   rect.y=y_min;
   rect.width=x_max-x_min;
   rect.height=y_max-y_min;
   return rect;
}


int main(int argc,char **argv)
{
   ros::init(argc,argv,"image_process_node");
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("/vrep/image_rect", 1, imageCallback);  
   pubDistance = nh.advertise<image_process::Distance>("/quadrotor/distance",100);    		
   ros::spin();
}

