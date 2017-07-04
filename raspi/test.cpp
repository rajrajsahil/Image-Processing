// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/objdetect/objdetect.hpp"
#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/contrib/contrib.hpp"
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ctime>//for camera
#include <fstream>//for camera
#include <iostream>//for camera
#include <unistd.h>//for camera
#include <raspicam/raspicam_cv.h>//for camera


using namespace std;
using namespace cv;

const float calibration_square_dimention =.0245f;//meters//scp test.cpp pi@ip0f pi:~/home/place
const Size chessboard_dimentions =Size(6,9);
cv::Mat camera_matrix;
cv::Mat distCoeffs;
// cameraMatrix = Mat::eye(3, 3, CV_64F);
// distCoeffs = Mat::zeros(8, 1, CV_64F);
// cv::Mat rvecs;
// cv::Mat tvecs;
std::vector<cv::Mat> rvecs;
std::vector<cv::Mat> tvecs;
vector<vector<Point2f> >image_corners;
vector<vector<Point3f> >world_corners;
//CAMERA CALIBRATION AND RECTIFICATION USING IMAGE;
int  creat_known_position(Size board_size, float square_edge_length)
{       
        int i,j;
        vector<Point3f> object_corners;
        for(i=0;i<board_size.height;i++)
        {
                for(j=0;j<board_size.width;j++)
                {       
                        object_corners.push_back(Point3f(j*square_edge_length,i*square_edge_length,0.0));
                }
        }
        world_corners.push_back(object_corners);
        return 0;
}
int number_of_image_to_take=100;
int number_of_image_to_calibrate=50;
int main()
{       
        int k,l;
        cv::Mat frame;
        //VideoCapture web_cam(0);
        int count=0;
        char * filename = new char[50];
        int i;
        raspicam::RaspiCam_Cv camera;
        camera.set(CV_CAP_PROP_FORMAT,CV_8UC3);
        while(1)
        {
                cout<<"camera:"<<camera.open()<<endl;
                camera.grab();
                camera.retrieve(frame);
                cv::Size image_size;
                image_size=frame.size();
                std::vector<Point2f> corner;
                //bool found =cv::findChessboardCorners(frame,chessboard_dimentions,corner);
                // if(found)
                // {    
                        imwrite("my_image.jpg",frame);//just for check
                        sprintf(filename,"img_for_calibration/image_%i.png",count);
                        imwrite(filename,frame);
                        count++;
                        cout<<count<<endl;
                // }
                //cv::drawChessboardCorners(frame,chessboard_dimentions,corner,found);
                imshow("for_taking images",frame);
                if(count>=number_of_image_to_take)
                {       
                        cout<<"number of images is taken:"<<number_of_image_to_take<<endl;
                        cout<<"process of taking images is over"<<endl;
                        break;
                }
                waitKey(500);
        }
        int recog_image=0;
        destroyWindow("for_taking images");
        for(i=0;i<number_of_image_to_take;i++)
                {       

                        vector<Point2f> corners;
                        sprintf(filename,"img_for_calibration/image_%i.png",i);
                        Mat image = imread(filename,CV_8UC3);
                        bool found =cv::findChessboardCorners(image,chessboard_dimentions,corners);
                        if(found)
                        {
                                image_corners.push_back(corners);
                                creat_known_position(chessboard_dimentions,calibration_square_dimention);
                                recog_image++;
                        }
                }
        cout<<"N. of image recognise:"<<recog_image<<endl;
         if(recog_image>=number_of_image_to_calibrate)
        {       cv::Size image_size;
                // sprintf(filename,"img_for_calibration/image_.png",i);
                Mat image = imread("img_for_calibration/image_0.png",CV_8UC3);
                image_size=image.size();
                cv::calibrateCamera(world_corners,image_corners,image_size,camera_matrix,distCoeffs,rvecs,tvecs);
                cout<<"Camera Matrix"<<camera_matrix.size()<<endl;
                for(k=0;k<camera_matrix.rows;k++)
                {
                        for(l=0;l<camera_matrix.cols;l++)
                        {
                                cout<<"["<<k<<","<<l<<"]"<<camera_matrix.at<double>(k,l)<<endl;
                        }
                }       
                cout<<"Distortion Coeficient"<<distCoeffs.size()<<endl;
                cout<<"Rotation vector"<<rvecs.size()<<endl;
                cout<<"Translation vector"<<tvecs.size()<<endl;
        }
        else
        {       
                cout<<"image recognised"<<recog_image<<endl;

                cout<<"N. of image recognised is less than limiting n. of image required to callibrate"<<endl;
        }
        Mat original_frame;
        Mat undistorted_frame;
        namedWindow("original",WINDOW_AUTOSIZE);
        namedWindow("undistorted_frame",WINDOW_AUTOSIZE);
        while(1)
        {
                camera.grab();
                camera.retrieve(original_frame);
                imshow("original",original_frame);
                cv::undistort(original_frame,undistorted_frame,camera_matrix,distCoeffs,noArray());
                imshow("undistorted_frame",undistorted_frame);
                waitKey(20);
        }
        return 0;
}       

/*
//CAMERA CALIBRATION AND RECTIFICATION USING VIDEO;
int  creat_known_position(Size board_size, float square_edge_length)
{       
        int i,j;
        vector<Point3f> object_corners;
        for(i=0;i<board_size.height;i++)
        {
                for(j=0;j<board_size.width;j++)
                {       
                        object_corners.push_back(Point3f(j*square_edge_length,i*square_edge_length,0.0));
                }
        }
        world_corners.push_back(object_corners);
        return 0;
}
// void Print (const vector<std::vector<Point3f> >& v){
//      int l;
//      for (int i=0; i<v.size();i++)
//      {
//              for(l=0; l<v[i].size();l++);
//      {
//              cout <<" Sahil raj"<< endl;
//              }
//      }
// }
int main()//22-05-2017
{
        int count = 0;
        VideoCapture web_cam(0);
        Mat frame;
        while(web_cam.isOpened())
        {
                web_cam>>frame;
                cv::Size image_size;
                image_size = frame.size();
                vector<Point2f> corners;
                bool found =cv::findChessboardCorners(frame,chessboard_dimentions,corners);
                if(found)
                {       
                        image_corners.push_back(corners);
                        // cout<<"total"<<frame.total()<<endl;
                        // cout<<"size"<<frame.size().height<<endl;
                        // int var=(int)corners.total();
                        //cout<<"corner"<< var <<endl;
                        cout<<"image_corners"<<image_corners.size()<<endl;
                        creat_known_position(chessboard_dimentions,calibration_square_dimention);
                        cout<<"world_corners"<<world_corners.size()<<endl;
                        //Print(world_corners);
                        count++;
                }
                cv::drawChessboardCorners(frame,chessboard_dimentions,corners,found);
                cout<<found<<endl;
                imshow("forcallibration",frame);
                waitKey(100);
                if(count>=100)
                {       
                        destroyWindow("forcallibration");
                        cout<<count<<endl;
                        cv::calibrateCamera(world_corners,image_corners,image_size,camera_matrix,distCoeffs,rvecs,tvecs);
                        break;
                }
        }
        cout<<"outsideloop"<<endl;
        int k,l;
        cout<<"Camera Matrix"<<camera_matrix.size()<<endl;
        cout<<"Distortion Coeficient"<<distCoeffs.size()<<endl;
        cout<<"Rotation vector"<<rvecs.size()<<endl;
        cout<<"Translation vector"<<tvecs.size()<<endl;
        // VideoCapture camera(0);
        Mat distorted_frame;
        Mat undistorted_frame;
        namedWindow("distorted_frame",WINDOW_AUTOSIZE);
        namedWindow("undistorted_frame",WINDOW_AUTOSIZE);
        if(!web_cam.isOpened())
        {
                cout<<"web_cam is stop"<<endl;
        }
        else
        {
                cout<<"web_cam is working"<<endl;
        }
        while(web_cam.isOpened())
        {
                web_cam>>distorted_frame;
                imshow("distorted_frame",distorted_frame);
                cv::undistort(distorted_frame,undistorted_frame,camera_matrix,distCoeffs,noArray());
                imshow("undistorted_frame",undistorted_frame);
                waitKey(20);
        }
        for(k=0;k<camera_matrix.rows;k++)
        {
                for(l=0;l<camera_matrix.cols;l++)
                {
                        cout<<"["<<k<<","<<l<<"]"<<camera_matrix.at<double>(k,l)<<endl;
                }
        }
        
        return 0;
}*/


