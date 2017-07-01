// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/objdetect/objdetect.hpp"
/*
Preprocessor directives are lines included in a program that being with the character #, 
which make them different from a typical source code text. They are invoked by the compiler to process some programs before compilation. 
Preprocessor directives change the text of the source code and the result is a new source code without these directives.
*/
#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include "opencv2/contrib/contrib.hpp"
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <cstdlib>
#include <opencv2/viz.hpp>
using namespace std;
using namespace cv;

/*int main()
{
	Mat left = imread("left.ppm",1);
	Mat right = imread("right.ppm",1);
	int i;
	for(i=0;i<left.cols;i++)
	{
		left.at<Vec3b>(200,i)[2]=255;
		left.at<Vec3b>(200,i)[1]=0;
		left.at<Vec3b>(200,i)[0]=0;
		right.at<Vec3b>(200,i)[2]=255;
		right.at<Vec3b>(200,i)[1]=0;
		right.at<Vec3b>(200,i)[0]=0;
	}
	imshow("letf",left);
	imshow("right",right);
	waitKey(0);
	return 0;
}*/

/*const float calibration_square_dimention =.0245f;//meters//scp test.cpp pi@ip0f pi:~/home/place
const Size chessboard_dimentions =Size(9,6);
cv::Mat camera_matrix1;
cv::Mat camera_matrix2;
cv::Mat distCoeffs1;
cv::Mat distCoeffs2;
cv::Mat rectificaton1;
cv::Mat rectificaton2;
cv::Mat projection1;
cv::Mat projection2;
cv::Mat reprojection;
cv::Mat essential;
cv::Mat fundamental;
cv::Mat F;
cv::Mat H1;
cv::Mat H2;
cv::Mat rotation;
cv::Mat translation;


// cameraMatrix = Mat::eye(3, 3, CV_64F);
// distCoeffs = Mat::zeros(8, 1, CV_64F);
// cv::Mat rvecs;
// cv::Mat tvecs;
// std::vector<cv::Mat> rotation;
// std::vector<cv::Mat> translation;
// std::vector<cv::Mat> essential;
// std::vector<cv::Mat> fundamental;
std::vector<cv::Mat> rvecs1;
std::vector<cv::Mat> tvecs1;
std::vector<cv::Mat> rvecs2;
std::vector<cv::Mat> tvecs2;
vector<vector<Point2f> >image_corners_left;
vector<vector<Point2f> >image_corners_right;
vector<vector<Point3f> >world_corners;

//STEREO CALIBRATION USING IMAGES
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
int number_of_image_taken=5;

int main()
{
	int i=0;
	vector<Point2f> corners1;
	vector<Point2f> corners2;
	int useful_image = 0;
	int permition;
	cv::Size image_size;
	char * filename = new char[50];
	ofstream file("test.txt");//out and trunk mode;
	for(i=0;i<number_of_image_taken;i++)
	{
		sprintf(filename,"stereo_images/get_image_%d.ppm",i);
		Mat stereo =imread(filename,1);
		//imshow("stereo",stereo);
		Mat left(stereo.rows,stereo.cols/2,CV_8UC3,Scalar(0,0,0));
		image_size=left.size();
		// cout<<"cols:"<<stereo.cols/2<<"rows:"<<stereo.rows<<endl;
		// cout<<"cols:"<<left.cols<<"rows:"<<left.rows<<endl;
		stereo(Rect(0,0,stereo.cols/2,stereo.rows)).copyTo(left);
		//cout<<"hello0"<<endl;
		Mat right(stereo.rows,stereo.cols/2,CV_8UC3,Scalar(0,0,0));
		stereo(Rect(stereo.cols/2,0,stereo.cols/2,stereo.rows)).copyTo(right);
		sprintf(filename,"left/left_%d.ppm",i);
		imwrite(filename,left);
		sprintf(filename,"right/right_%d.ppm",i);
		imwrite(filename,right);
		vector<Point2f> corners_left;
		bool found_left =cv::findChessboardCorners(left,chessboard_dimentions,corners_left);
		vector<Point2f> corners_right;
		bool found_right =cv::findChessboardCorners(right,chessboard_dimentions,corners_right);
		if(found_right&&found_left)
		{
			image_corners_left.push_back(corners_left);
			
			image_corners_right.push_back(corners_right);

			creat_known_position(chessboard_dimentions,calibration_square_dimention);
			useful_image++;
			cout<<useful_image<<endl;
		}
	}
		
	cout<<"useful_image"<<useful_image<<endl;
	int k,l;
	cv::calibrateCamera(world_corners,image_corners_left,image_size,camera_matrix1,distCoeffs1,rvecs1,tvecs1);
	cv::calibrateCamera(world_corners,image_corners_right,image_size,camera_matrix2,distCoeffs2,rvecs2,tvecs2);
	cout<<"camera_matrix1:"<<camera_matrix1.size()<<endl;
	cout<<camera_matrix1<<endl;
	
	cout<<"camera_matrix2:"<<camera_matrix2.size()<<endl;
	cout<<camera_matrix2<<endl;
	
	cout<<"calibration started"<<endl;
	cv::stereoCalibrate(world_corners,image_corners_left,image_corners_right,camera_matrix1,distCoeffs1,camera_matrix2,distCoeffs2,image_size,rotation,translation,essential,fundamental,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),CV_CALIB_USE_INTRINSIC_GUESS);
	cout<<"calibration done"<<endl;
	file<<"CALIBRATION DATA"<<endl;
	file<<"camera_matrix1(left camera):"<<camera_matrix1.size()<<endl;
	file<<camera_matrix1<<endl;
	file<<"distCoeffs1(left camera):"<<distCoeffs1.size()<<endl;
	file<<distCoeffs1<<endl;
	file<<"camera_matrix2(right camera):"<<camera_matrix2.size()<<endl;
	file<<camera_matrix2<<endl;
	file<<"distCoeffs2(right camera):"<<distCoeffs2.size()<<endl;
	file<<distCoeffs2<<endl;
	file<<"rotation matrix:"<<rotation.size()<<endl;
	file<<rotation<<endl;
	file<<"Translation matrix:"<<translation.size()<<endl;
	file<<translation<<endl;
	file<<"essential:"<<essential.size()<<endl;
	file<<"determinant(essential):"<<determinant(essential)<<endl;
	file<<essential<<endl;	
	file<<"fundamental:"<<fundamental.size()<<endl;
	file<<fundamental<<endl;	
	// for(k=0;k<rotation.size();k++)
	// {
	// 	file<<"["<<k<<"]"<<rotation[k]<<endl;
	// }
		
				
	
	cout<<"rectification started"<<endl;
	cv::stereoRectify(camera_matrix1,distCoeffs1,camera_matrix2,distCoeffs2,image_size,rotation,translation,rectificaton1,rectificaton2,projection1,projection2,reprojection,cv::CALIB_ZERO_DISPARITY,0);
	cout<<"Rectification Done"<<endl;
	file<<"RECTIFICATION DATA"<<endl;
	file<<"rectificaton1:"<<rectificaton1.size()<<endl;
	file<<rectificaton1<<endl;
	file<<"rectificaton2:"<<rectificaton2.size()<<endl;
	file<<rectificaton2<<endl;
	file<<"projection1:"<<projection1.size()<<endl;
	file<<projection1<<endl;
	file<<"projection2:"<<projection2.size()<<endl;
	file<<projection2<<endl;
	file<<"reprojection:"<<reprojection.size()<<endl;
	file<<reprojection<<endl;
	cv::Mat map1_1;
	cv::Mat map1_2;
	cv::Mat map2_1;
	cv::Mat map2_2;
	// cout<<"hello"<<endl;
	// stereoRectifyUncalibrated(corners1,corners2,fundamental,image_size,H1,H2);
	// cout<<"hello"<<endl;
	// cv:: Mat R1;
	// cv:: Mat R2;
	// R1 = camera_matrix1.inv()*H1*camera_matrix1;
	// R1 = camera_matrix2.inv()*H2*camera_matrix2;
	cv::initUndistortRectifyMap(camera_matrix1,distCoeffs1,rectificaton1,projection1,image_size,CV_16SC2,map1_1,map1_2);
	cv::initUndistortRectifyMap(camera_matrix2,distCoeffs2,rectificaton2,projection2,image_size,CV_16SC2,map2_1,map2_2);
	cout<<"map1_1:"<<map1_1.size()<<endl;
	cout<<"map1_1 type"<<map1_1.type()<<endl;
	cout<<"map1_1 type"<<map1_2.type()<<endl;
	// imwrite("map1_1.jpg",map1_1);
	// imwrite("map1_2.jpg",map1_2);
	// imwrite("map2_1.jpg",map2_1);
	// imwrite("map2_2.jpg",map2_2);
	// cout<<"map1_1:"<<map1_1.at<Vec2s>(0,0)<<endl;
	file<<map1_1<<endl;
	file<<map1_2<<endl;
	file<<map2_1<<endl;
	file<<map2_2<<endl;
	Mat image_left =imread("left/left_38.ppm",1);

	file<<"Image left size"<<image_left.size()<<endl;
	Mat dst1;//(map1_1.cols,map1_1.rows,CV_8UC3,Scalar(0));
	remap(image_left,dst1,map1_1,map1_2,CV_INTER_LINEAR,BORDER_CONSTANT);
	imwrite("left.ppm",dst1);
	//cv::undistort(image_left,dst1,camera_matrix1,distCoeffs1,noArray());
	Mat image_right=imread("right/right_38.ppm",1);
	Mat dst2;//(map2_1.cols,map2_1.rows,CV_8UC3,Scalar(0));
	remap(image_right, dst2, map2_1, map2_2, CV_INTER_LINEAR,BORDER_CONSTANT);
	imwrite("right.ppm",dst2);
	//cv::undistort(image_right,dst2,camera_matrix2,distCoeffs2,noArray());
	// imshow("left1",image_left);
	// imshow("right1",image_right);
	imshow("left",dst1);
	//file<<dst1<<endl;
	imshow("right",dst2);
	file<<"dst2"<<dst2.size()<<endl;
	//file<<dst2<<endl;
	waitKey(0);
	return 0;
}*/

/*
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
int number_of_image_to_take=200;
int number_of_image_to_calibrate=50;
int main()
{	
	int k,l;
	Mat frame;
	VideoCapture web_cam(0);
	int count=0;
	char * filename = new char[50];
	int i;
	while(web_cam.isOpened())
	{
		web_cam>>frame;
		cv::Size image_size;
		image_size=frame.size();
		std::vector<Point2f> corner;
		//bool found =cv::findChessboardCorners(frame,chessboard_dimentions,corner);
		// if(found)
		// {	
			imwrite("my_image.jpg",frame);
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
	{	cv::Size image_size;
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
	return 0;
}*/	

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
// 	int l;
//   	for (int i=0; i<v.size();i++)
//   	{
//   		for(l=0; l<v[i].size();l++);
//     	{
//     		cout <<" Sahil raj"<< endl;
//   		}
// 	}
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
/*int main()
{
	Mat map1_x;
	Mat map1_2;
	Mat map2_1;
	Mat map2_2;
	int count=0;
	ifstream myfiles;
	myfiles.open("map1_1");
	if(!myfiles.is_open())
	{
		cout<<"file is not open"<<endl;
	}
	while(myfiles.eof())
	{
		count++;
	}
	cout<<"cunt"<<count<<endl;
	return 0;
}*/

/*int main()
{	

	Mat stereo = imread("inside4.ppm");
	cout<<"hello"<<endl;
	Mat img1(stereo.rows,stereo.cols/2,CV_8UC3,Scalar(0,0,0));
	stereo(Rect(0,0,stereo.cols/2,stereo.rows)).copyTo(img1);
	Mat img2(stereo.rows,stereo.cols/2,CV_8UC3,Scalar(0,0,0));
	stereo(Rect(stereo.cols/2,0,stereo.cols/2,stereo.rows)).copyTo(img2);


	char val;
	int val1;
	Mat map1_x(img1.rows,img1.cols,CV_16SC2,Scalar(0));
	Mat map2_x(img1.rows,img1.cols,CV_16SC2,Scalar(0));
	Mat map1_y(img1.rows,img1.cols,CV_16UC1,Scalar(0));
	Mat map2_y(img1.rows,img1.cols,CV_16UC1,Scalar(0));
	ifstream file;
	ofstream save;
	file.close();
	save.close();

	file.open("new_map1_1");
	if(file.good())
	{
		for(int i=0;i<1080;i++)
		{	
			
			if(i==0)
				{
					file>>val;
				}
			for(int j=0;j<960;j++)
			{	
				file>>val1;
				map1_x.at<Vec2s>(i,j)[0]=val1;
				file>>val;

				file>>val1;
				map1_x.at<Vec2s>(i,j)[1]=val1;
				file>>val;
			}

		}
	}
	else
	{
		cout<<"prob"<<endl;
	}
	save.open("map1_x");
	save<<map1_x<<endl;
	file.close();
	save.close();
cout<<"hello"<<endl;

	file.open("new_map2_1");
	if(file.good())
	{
		for(int k=0;k<1080;k++)
		{	
			
			if(k==0)
				{
					file>>val;
				}
			for(int l=0;l<960;l++)
			{	
				file>>val1;
				map2_x.at<Vec2s>(k,l)[0]=val1;
				file>>val;

				file>>val1;
				map2_x.at<Vec2s>(k,l)[1]=val1;
				file>>val;
			}

		}
	}
	else
	{
		cout<<"prob"<<endl;
	}
	save.open("map2_x");
	save<<map2_x<<endl;
	file.close();
	save.close();

cout<<"hello"<<endl;
	file.open("new_map1_2");
	if(file.good())
	{
		for(int k=0;k<1080;k++)
		{	
			
			if(k==0)
				{
					file>>val;
				}
			for(int l=0;l<960;l++)
			{	
				file>>val1;
				map1_y.at<unsigned short>(k,l)=val1;
				file>>val;
			}

		}
	}
	else
	{
		cout<<"prob"<<endl;
	}
	save.open("map1_y");
	save<<map1_y<<endl;
	file.close();
	save.close();

	
	file.open("new_map2_2");
	if(file.good())
	{
		for(int k=0;k<1080;k++)
		{	
			
			if(k==0)
				{
					file>>val;
				}
			for(int l=0;l<960;l++)
			{	
				file>>val1;
				map2_y.at<unsigned short>(k,l)=val1;
				file>>val;
			}

		}
	}
	else
	{
		cout<<"prob"<<endl;
	}
	save.open("map2_y");
	save<<map2_y<<endl;
	file.close();
	save.close();

cout<<"done"<<endl;
	Mat img11;
	Mat img22;
	cout<<"done"<<endl;
	cout<<"mapx:"<<map1_x.size()<<endl;
	cout<<"mapy:"<<map1_y.size()<<endl;
	remap(img1, img11, map1_x, map1_y, CV_INTER_LINEAR,BORDER_CONSTANT);
	imshow("left",img1);
	imshow("left_rectify",img11);
	imwrite("left.ppm",img11);
	remap(img2, img22, map2_x, map2_y, CV_INTER_LINEAR,BORDER_CONSTANT);
	imshow("right",img1);
	imshow("right_rectify",img22);
	imwrite("right.ppm",img22);
	waitKey(0);
	// imwrite("left",img11);
	// imwrite("right",img22);
	// imshow("left",img11);
	// imshow("right",img22);

	return 0;
}*/
/////SGBM


Mat g1, g2;
Mat points;
Mat reprojection(4,4,CV_64F);
Mat disp, disp8;
Mat image_left=imread("left.ppm",1);
Mat image_right=imread("right.ppm",1);
Mat img1=imread("left.ppm",0);
Mat img2=imread("right.ppm",0);
// Mat img1;
// Mat img2;
Mat img11;
Mat img22;
Mat img111;
Mat img222;
Mat img1111;
Mat img2222;
Mat disparity;
int SADWindowSize1=5;
int numberOfDisparities1=173;
int preFilterCap1=11;
int minDisparity1 =0;
int textureThreshold1=3;
int uniquenessRatio1=3;
int speckleWindowSize1=0;
int speckleRange1=5;
int disp12MaxDiff1=228;
int p11 =554;
int p21 =3408;
void therosole(int , void*)
{
// if (!(strcmp(method, "BM")))
// {
	//destroyWindow("disp8");
	// namedWindow("disp8",WINDOW_NORMAL);
 //    StereoBM sbm;
 //    while((SADWindowSize1)%2==0)
 //    {
 //    	SADWindowSize1 = SADWindowSize1-1;
 //    }
 //    sbm.state->SADWindowSize = SADWindowSize1;
 //    while(numberOfDisparities1%16!=0)
 //    {
 //    	numberOfDisparities1=numberOfDisparities1+1;
 //    }
 //    sbm.state->numberOfDisparities = numberOfDisparities1;
 //    //sbm.state->preFilterSize = preFilterSize1;
 //    sbm.state->preFilterCap = preFilterCap1;
 //    sbm.state->minDisparity = -minDisparity1;
 //    sbm.state->textureThreshold = textureThreshold1;
 //    sbm.state->uniquenessRatio = uniquenessRatio1;
 //    sbm.state->speckleWindowSize = speckleWindowSize1;
 //    sbm.state->speckleRange = speckleRange1;
 //    sbm.state->disp12MaxDiff = disp12MaxDiff1;
 //    sbm(g1, g2, disp);

//}
// else if (!(strcmp(method, "SGBM")))
// {
    StereoSGBM sbm;
       while((SADWindowSize1)%2==0)
    {
    	SADWindowSize1 = SADWindowSize1-1;
    }
    sbm.SADWindowSize =SADWindowSize1 ;
    while(numberOfDisparities1%16!=0)
    {
    	numberOfDisparities1=numberOfDisparities1+1;
    }
    sbm.numberOfDisparities = numberOfDisparities1;
    sbm.preFilterCap = preFilterCap1;
    sbm.minDisparity = -minDisparity1;
    sbm.uniquenessRatio = uniquenessRatio1;
    sbm.speckleWindowSize = speckleWindowSize1;
    sbm.speckleRange = speckleRange1;
    sbm.disp12MaxDiff = disp12MaxDiff1;
    sbm.fullDP = false;
    sbm.P1 = p11;
    sbm.P2 = p21;
    sbm(img1111,img2222, disp);
// }


	normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
	



	
	Mat disparity1;
	medianBlur (disp8,disparity1,5);
	GaussianBlur(disparity1,disparity,Size (5,5),0,0,BORDER_DEFAULT);

	cout<<"image1_1"<<img1.type()<<endl;
cv::reprojectImageTo3D(disparity,points,reprojection,false,-1);
	cout<<"points type:"<<points.type()<<endl;
	cout<<"point size"<<points.size()<<endl;
	ofstream point_cloud_file;
    point_cloud_file.open ("point_cloud.xyz");
    for(int i = 0; i < points.rows; i++) {
        for(int j = 0; j < points.cols; j++) {
            if(points.at<Vec3f>(i,j)[2] < 10) {
                point_cloud_file << points.at<Vec3f>(i,j)[0] << " " << points.at<Vec3f>(i,j)[1] << " " << points.at<Vec3f>(i,j)[2] 
                    << " " << static_cast<unsigned>(image_left.at<uchar>(i,j)) << " " << static_cast<unsigned>(image_left.at<uchar>(i,j)) << " " << static_cast<unsigned>(image_left.at<uchar>(i,j)) << endl;
        	}
        }
    }
    cout<<"image1_2"<<img1.type()<<endl;
    imshow("left_image",img1);
    point_cloud_file.close();


	//imwrite("disparity_blur.ppm",disp8);
	imshow("disp8", disparity);

viz::Viz3d myWindow("Coordinate Frame");

    while (!myWindow.wasStopped())
    {
        /// Create a cloud widget
        viz::WCloud cw(points, viz::Color::red());

        /// Display it in a window
        myWindow.showWidget("CloudWidget1", cw);

        myWindow.spinOnce(1, true);
    }


	//imshow("3d_image",d_image);
	//ofstream file("3d_image.ply");
	// file<<d_image<<endl;
}
int main()
{
	// resize(img01 ,img1,Size(),0.5,0.5);
	// resize(img02 ,img2,Size(),0.5,0.5);
	//imwrite("half1.ppm",img1);
	equalizeHist(img1,img11);//only one chanel is allowed
	equalizeHist(img2,img22);
	GaussianBlur(img11,img111,Size (5,5),0,0,BORDER_DEFAULT);
	GaussianBlur(img22,img222,Size (5,5),0,0,BORDER_DEFAULT);
	medianBlur (img111,img1111,5);
	medianBlur (img222,img2222,5);
	reprojection.at<double>(0,0)=1;
	reprojection.at<double>(0,1)=0;
	reprojection.at<double>(0,2)=0;
	reprojection.at<double>(0,3)=-365.5749244689941;
	reprojection.at<double>(1,0)=0;
	reprojection.at<double>(1,1)=1;
	reprojection.at<double>(1,2)=0;
	reprojection.at<double>(1,3)=-695.0291175842285;
	reprojection.at<double>(2,0)=0;
	reprojection.at<double>(2,1)=0;
	reprojection.at<double>(2,2)=0;
	reprojection.at<double>(2,3)=1223.222888161194;
	reprojection.at<double>(3,0)=0;
	reprojection.at<double>(3,1)=0;
	reprojection.at<double>(3,2)=12.4291181632413;
	reprojection.at<double>(3,3)=-0;
	cout<<"reprojection:"<<reprojection.size()<<endl;
	cout<<reprojection<<endl;
	namedWindow("disp8",WINDOW_NORMAL);
	namedWindow("disp",WINDOW_NORMAL);
	//namedWindow("3d_image",WINDOW_NORMAL);

	createTrackbar("SADWindowSize","disp",&SADWindowSize1,27,therosole);
	createTrackbar("numberOfDisparities","disp",&numberOfDisparities1,500,therosole);
	createTrackbar("preFilterCap","disp",&preFilterCap1,27,therosole);
	createTrackbar("minDisparity","disp",&minDisparity1,100,therosole);
	//createTrackbar("textureThreshold","disp",&textureThreshold1,800,therosole);
	createTrackbar("uniquenessRatio","disp",&uniquenessRatio1,15,therosole);
	createTrackbar("speckleWindowSize","disp",&speckleWindowSize1,200,therosole);
	createTrackbar("speckleRange","disp",&speckleRange1,5,therosole);
	createTrackbar("disp12MaxDiff","disp",&disp12MaxDiff1,250,therosole);
	createTrackbar("p1","disp",&p11,800,therosole);
	createTrackbar("p2","disp",&p21,5000,therosole);
	therosole(0,0);

	// namedWindow("left",WINDOW_NORMAL);
	// namedWindow("right",WINDOW_NORMAL);
	// namedWindow("disp",WINDOW_NORMAL);
	// imshow("left", img1);
	// imshow("right", img2);
	
	waitKey(0);

	return(0);
}

/*int main()//14.5.2017
{	
	int minDisparity;
    int numberOfDisparities;
    int SADWindowSize;
    int preFilterCap;
    int uniquenessRatio;
    int P1, P2;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    bool fullDP;
    Mat left = imread("left.png",1);
    Mat right = imread("right.png",1);
    Mat disp;
	StereoSGBM();
    StereoSGBM(int minDisparity, int numDisparities, int SADWindowSize,
               int P1=0, int P2=0, int disp12MaxDiff=0,
               int preFilterCap=0, int uniquenessRatio=0,
               int speckleWindowSize=0, int speckleRange=0,
               bool fullDP=false);
    virtual ~StereoSGBM();

    virtual void operator()(left,rigth,disp);
    namedWindow("disparity_map",WINDOW_NORMAL);
    imshow("disparity_map",disp);
    waitKey(0);
    return 0;
}*/
/*int main()//13.5.2017(disparity map using intrinsic matrix)
{	
	int j,i,k,AID,Lr,Lg,Lb,Rr,Rg,Rb,min_diff,match_point;
	int ther_min_aid=5;
	float Z;
	float lfx=4152.073;
	float lxo=1288.127;
	float rfx=4152.073;
	float rxo=1501.231;
	float b=176.252;
	Mat left = imread("left.png",1);
	Mat right = imread("right.png",1);
	Mat disparity_image(left.rows,left.cols,CV_8UC1,Scalar(0));
	for(j=0;j<left.rows;j++)
	{	
		cout<<j<<endl;
		for(i=0;i<left.cols;i++)
		{	//cout<<i<<endl;
			Lr=left.at<Vec3b>(j,i)[2];//Lr:left image red color intensity and same for Lg and Lb;
			Lg=left.at<Vec3b>(j,i)[1];
			Lb=left.at<Vec3b>(j,i)[0];
			min_diff=100;
			match_point=right.cols+1;
			for(k=0;k<i;k++)
			{	//cout<<"k="<<k<<endl;
				Rr=right.at<Vec3b>(j,k)[2];//Lr:left image red color intensity and same for Lg and Lb;
				Rg=right.at<Vec3b>(j,k)[1];
				Rb=right.at<Vec3b>(j,k)[0];	
				AID = sqrt((Lr-Rr)*(Lr-Rr)+(Lg-Rg)*(Lg-Rg)+(Lb-Rb)*(Lb-Rb));
				//cout<<"AID"<<AID<<endl;
				if(AID<=min_diff)
				{	//cout<<"Sahil"<<endl;
					min_diff=AID;
					match_point=k;
				}
				//cout<<"i:"<<i<<"k:"<<k<<"AID"<<AID<<"match_point"<<match_point<<endl;
				if(k==(i-1)&&match_point!=(right.cols+1)&&min_diff<ther_min_aid)
				{
					Z=b/(((i-lxo)/lfx)-((match_point-rxo)/rfx));
					//cout<<"j:"<<j<<"i:"<<i<<",K:"<<match_point<<",diff"<<min_diff<<",Z="<<Z<<endl;
					disparity_image.at<uchar>(j,i)=255-Z/15;
				}
			}
		}
	}
	namedWindow("disparity_image",WINDOW_NORMAL);
	imshow("disparity_image",disparity_image);
	waitKey(0);
	return 0;
}*/

// int main()//12.5.2017 (disparity map)
// {	
// 	int i,j,k,Xl,Lb,Lg,Lr,Rb,Rg,Rr,AID,d,zindex;
// 	int ther_min_aid=10;
// 	int min_aid;
// 	int camera=7650;
// 	Mat left = imread("left.ppm",1);
// 	Mat right = imread("right.ppm",1);
// 	int Xr = right.cols+1;
// 	Mat disparity_map(left.rows,left.cols,CV_8UC1,Scalar(0));
// 	for(j=0;j<left.rows;j++)
// 	{	
// 		cout<<j<<endl;
// 		for(i=0;i<left.cols;i++)
// 		{	
// 			Xl=i;
// 			Lr=left.at<Vec3b>(j,i)[2];//Lr:left image red color intensity and same for Lg and Lb;
// 			Lg=left.at<Vec3b>(j,i)[1];
// 			Lb=left.at<Vec3b>(j,i)[0];
// 			min_aid = 100;
// 			for(k=0;k<i;k++)
// 			{
// 				//cout<<i<<endl;
				
// 				Rr=right.at<Vec3b>(j,k)[2];//Rr:Right image red color intensity and same for Rg and Rb;
// 				Rg=right.at<Vec3b>(j,k)[1];
// 				Rb=right.at<Vec3b>(j,k)[0];
// 				//cout<<"sahil"<<endl;
// 				AID  = sqrt((Lr-Rr)*(Lr-Rr)+(Lg-Rg)*(Lg-Rg)+(Lb-Rb)*(Lb-Rb));//AID:Absolute intensity diffrence;
// 				if(AID<=min_aid)
// 				{
// 					min_aid = AID;
// 					Xr = k;
// 				}
// 				if(k==(i-1)&&Xl!=0&&Xr!=right.cols+1&&min_aid<=ther_min_aid)
// 				{	
					
// 					d=(Xl-Xr);
// 					zindex=camera/d;
// 					cout<<j<<","<<i<<"Xl-Xr"<<Xl-Xr<<"k"<<k<<"zindex"<<zindex<<endl;
// 					/*if(d<=30)
// 					{
// 						d=0;
// 					}
// 					else if(d>30&&d<=60)
// 					{
// 						d=45;
// 					}
// 					else if(d>60&&d<=90)
// 					{
// 						d=75;
// 					}
// 					else if(d>90&&d<130)
// 					{
// 						d=110;
// 					}
// 					else if(d>=130&&d<170)
// 					{
// 						d=150;
// 					}
// 					else if(d>=170&&d<210)
// 					{
// 						d=190;
// 					}
// 					else if(d>=210&&d<250)
// 					{
// 						d=230;
// 					}
// 					else
// 					{
// 						d=255;
// 					}*/
// 					disparity_map.at<uchar>(j,i)=(255-zindex);
// 				}
// 			}
// 		}
// 	}
// 	namedWindow("disparity_image",WINDOW_NORMAL);
// 	imshow("disparity_image",disparity_map);
// 	waitKey(0);
// 	return 0;
// }
/*int main()
{	
	int i,j,k,l,m;
	int point=0;
	int min =100;
	int minpoint;
	int diff1,diff;
	int rdiff,bdiff,gdiff;
	Mat left = imread("left.png",1);
	Mat right =imread("right.png",1);
	Mat var(left.cols,left.rows,CV_8UC1,Scalar(255));
	int red = left.at<Vec3b>(700,700)[2];
	int green =left.at<Vec3b>(700,700)[1];
	int blue =left.at<Vec3b>(700,700)[0];
	for(i=0;i<left.cols;i++)
	{
		rdiff = (right.at<Vec3b>(700,i)[2]-red)*(right.at<Vec3b>(700,i)[2]-red);
		gdiff = (right.at<Vec3b>(700,i)[1]-green)*(right.at<Vec3b>(700,i)[1]-green);
		bdiff = (right.at<Vec3b>(700,i)[0]-blue)*(right.at<Vec3b>(700,i)[0]-blue);
		diff =sqrt(rdiff+bdiff+gdiff);
		for(j=left.rows;j>left.rows-diff;j--)
		{
			var.at<uchar>(j,i)=0;
		}
	}
	for(k=0;k<left.cols;k++)
	{	
		left.at<Vec3b>(700,k)[2]=0;
		left.at<Vec3b>(700,k)[1]=0;
		left.at<Vec3b>(700,k)[0]=255;
		right.at<Vec3b>(700,k)[2]=0;
		right.at<Vec3b>(700,k)[1]=0;
		right.at<Vec3b>(700,k)[0]=255;
	}
	for(l=0;l<right.rows;l++)
	{
		right.at<Vec3b>(l,551)[2]=0;
		right.at<Vec3b>(l,551)[1]=0;
		right.at<Vec3b>(l,551)[0]=255;
		left.at<Vec3b>(l,700)[0]=255;
		left.at<Vec3b>(l,700)[1]=0;
		left.at<Vec3b>(l,700)[2]=0;
	}
	namedWindow("original",WINDOW_NORMAL);
	imshow("original",var);
	namedWindow("left",WINDOW_NORMAL);
	imshow("left",left);
	namedWindow("right",WINDOW_NORMAL);
	imshow("right",right);
	waitKey(0);
	return 0;
}*/
/* simle two color rectangle
int main() 
{
	// Mat var1 = imread("pic.jpg",1);
	// namedWindow("windows1",WINDOW_AUTOSIZE);
	// imshow("windows1",var1);
	// waitKey(0);

	int i=0;
	int j=0;
	Mat var1(200,400,CV_8UC1,Scalar(0));
	for(i=0;i<200;i++)
	{
		for(j=0;j<400;j++)
		{
			if(j>=300)
			{
				var1.at<uchar>(i,j)=0;	
			}
			else
			{
			var1.at<uchar>(i,j)=255;
			}
		}
	}
	namedWindow("windows1",WINDOW_AUTOSIZE);
	imshow("windows1",var1);
	waitKey(0);
	return 0;
}*/
/////////////////chessboard
/*
int main()
{	
	int i=0;
	int j=0;
	int dim = 16;
	Mat var1(128*2,128*2,CV_8UC1,Scalar(0));
	for(i=0;i<var1.rows;i++)
	{
		if((i/dim)%2==0)
		{
			for(j=0;j<var1.cols;j++)
			{
				if((j/dim)%2==1)
				{
					var1.at<uchar>(i,j)=255;
				}
			}
		}
		else
		{
			for(j=0;j<var1.cols;j++)
			{
				if((j/dim)%2==0)
				{
					var1.at<uchar>(i,j)=255;
				}
			}
		}
	}
	namedWindow("chessboard",WINDOW_AUTOSIZE);
	imshow("chessboard",var1);
	waitKey(0);
	return 0;

}
*/
/////////////////


// int main()
// {
// 	int i=0;
// 	int j=0;
// 	Mat var1 = imread("pic.jpg",1);
// 	Mat var2(var1.rows,var1.cols,CV_8UC1,Scalar(0));
// 	Mat var3(var1.rows,var1.cols,CV_8UC1,Scalar(0));
// 	for(i=0;i<var1.rows;i++)
// 	{
// 		for(j=0;j<var1.cols;j++)
// 		{
// 			var2.at<uchar>(i,j)=(var1.at<Vec3b>(i,j)[0]+var1.at<Vec3b>(i,j)[1]+var1.at<Vec3b>(i,j)[1])/3;
// 			var3.at<uchar>(i,j)=((var1.at<Vec3b>(i,j)[0]+var1.at<Vec3b>(i,j)[1]+var1.at<Vec3b>(i,j)[1])/3)>150? 255:0;
// 		}
// 	}

// 	namedWindow("original",WINDOW_AUTOSIZE);
// 	imshow("original",var1);
// 	namedWindow("grayscale",WINDOW_AUTOSIZE);
// 	imshow("grayscale",var2);
// 	namedWindow("binary",WINDOW_AUTOSIZE);
// 	imshow("binary",var3);
// 	waitKey(0);
// 	return 0;
// }


/*// function to convert color   basis;
int main()
{
	int i=0;
 	int j=0;
 	Mat var1 = imread("pic.jpg",1);
 	Mat var2(var1.rows,var1.cols,CV_8UC1,Scalar(0));
 	Mat var3(var1.rows,var1.cols,CV_8UC1,Scalar(0));
 	cvtColor(var1,var2,CV_BGR2HSV);//CV_RGB2RAY,CV_BGR2HSV,CV_HSV2BGR
 	namedWindow("cvtgray",WINDOW_AUTOSIZE);
 	imshow("cvtgray",var2);
 	namedWindow("original",WINDOW_AUTOSIZE);
 	imshow("original",var1);
 	waitKey(0);
 	return 0;

}*/
/*
int a;
Mat var1=imread("pic.jpg",1);
Mat red(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
void therosole( int, void* )
{	
	int i,j;
	for(i=0;i<var1.rows;i++)
		{
			for(j=0;j<var1.cols;j++)
			{
				red.at<Vec3b>(i,j)[0]=(var1.at<Vec3b>(i,j)[0])>a? (var1.at<Vec3b>(i,j)[0]):0;
				red.at<Vec3b>(i,j)[1]=(var1.at<Vec3b>(i,j)[1])>a? (var1.at<Vec3b>(i,j)[1]):0;
				red.at<Vec3b>(i,j)[2]=(var1.at<Vec3b>(i,j)[2])>a? (var1.at<Vec3b>(i,j)[2]):0;

			}
		}
	imshow("windows1",red);
}
int main()
{	
	int i=0;
	int j=0;
	namedWindow("windows1",WINDOW_AUTOSIZE);
	createTrackbar("track","windows1",&a,255,therosole);
	therosole(0,0);
	waitKey(0);
	return 0;
}*/



/*
int main()
{	
	int i=0;
	int j=0;
	int k=0;
	int a[3];
	int tol[3];
	Mat var1=imread("thero.jpg",1);
	Mat tolr(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
	namedWindow("windows",WINDOW_AUTOSIZE);
	createTrackbar("red","windows",&a[2],255);
	createTrackbar("Green","windows",&a[1],255);
	createTrackbar("Blue","windows",&a[0],255);
	createTrackbar("tollerence Red","windows",&tol[2],255);
	createTrackbar("tollerence Green","windows",&tol[1],255);
	createTrackbar("tollerence Blue","windows",&tol[0],255);
	while(1)
	{	
		for(i=0;i<var1.rows;i++)
		{
			for(j=0;j<var1.cols;j++)
			{   
				for(k=0;k<3;k++)
				{
					if(var1.at<Vec3b>(i,j)[k]>(a[k]-tol[k]) && var1.at<Vec3b>(i,j)[k]<(a[k]+tol[k]))
					{
						tolr.at<Vec3b>(i,j)[k]=var1.at<Vec3b>(i,j)[k];
					}
					else
					{
						tolr.at<Vec3b>(i,j)[k]=0;
					}
				}
			}
		}
		imshow("windows",tolr);
		waitKey(10);
	}
	
	return 0;
}
*/
/*
//////doubling the size of image
int main()
{	
	int i,j,k;
	Mat var1=imread("pic.jpg",1);
	Mat doublesize(var1.rows*2,var1.cols*2,CV_8UC3);
	for(i=0;i<var1.rows;i++)
	{
		for(j=1;j<var1.cols;j++)
		{	for(k=0;k<3;k++)
			{
			doublesize.at<Vec3b>(i*2+1,j*2+1)[k]=var1.at<Vec3b>(i,j)[k];
			doublesize.at<Vec3b>(i*2+1,j*2)[k]=var1.at<Vec3b>(i,j)[k];
			doublesize.at<Vec3b>(i*2,j*2+1)[k]=var1.at<Vec3b>(i,j)[k];
			doublesize.at<Vec3b>(i*2,j*2)[k]=var1.at<Vec3b>(i,j)[k];
			}
		}
	}
	namedWindow("doublethesize",WINDOW_AUTOSIZE);
	imshow("doublethesize",doublesize);
	namedWindow("original",WINDOW_AUTOSIZE);
	imshow("original",var1);
	waitKey(0);
	return 0;
}
*/
/* half the size
int main()
{	
	int i,j,k;
	Mat var1=imread("pic.jpg",1);
	Mat halfsize(var1.rows/2,var1.cols/2,CV_8UC3);
	for(i=0;i<var1.rows/2;i++)
	{
		for(j=0;j<var1.cols/2;j++)
		{
			for(k=0;k<3;k++)
			{
				halfsize.at<Vec3b>(i,j)[k]=(var1.at<Vec3b>(i*2,j*2)[k] + var1.at<Vec3b>(i*2,j*2+1)[k] + var1.at<Vec3b>(i*2+1,j*2)[k] + var1.at<Vec3b>(i*2+1,j*2+1)[k])/4;
			}
		}
	}
	namedWindow("halfthesize",WINDOW_AUTOSIZE);
	imshow("halfthesize",halfsize);
	waitKey(0);
	return 0;
}
*/
////////// histogram

// int max(int n[256])
// {	int number;
// 	int k=0;
// 	int max=0;
// 	for(k=0;k<256;k++)
// 	{
// 		if(n[k]>max)
// 		{
// 			max = n[k];
// 			number = k;
// 		}
// 	}
// 	//cout<< "number=>"<<number <<endl;
// 	return max/50;
// }
// int main()
// {
// 	int n[256];
// 	int i,j,k,l,m,maximum;
// 	Mat var1=imread("pic.jpg",1);
// 	for(i=0;i<256;i++)
// 	{
// 		n[i]=0;
// 	}
// 	for(i=0;i<var1.rows;i++)
// 	{
// 		for(j=0;j<var1.cols;j++)
// 		{
// 			n[var1.at<Vec3b>(i,j)[0]]++;
// 		}

// 	}
// 	Mat histo(max(n),255,CV_8UC3,Scalar(0,0,255));
// 	maximum = max(n);
// 	cout << maximum <<endl;
	
// 	for(l=0;l<255;l++)
// 	{
// 		for(m=0;m<(maximum-(n[l]/50));m++)
// 		{
// 			histo.at<Vec3b>(m,l)[2]=0;
// 		}
// 	}
// 	namedWindow("histogram",WINDOW_AUTOSIZE);
// 	imshow("histogram",histo);
// 	waitKey(0);
	
// 	return 0;

// }

//////////ALL FILTERS
/*int main()
{	
	int i,j,k,gx,gy;
	Mat var1= imread("pic.jpg",1);
	Mat blur(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
	Mat gauss(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
	Mat sobal(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
	Mat xsobal(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
	for(i=1;i<var1.rows-1;i++)
	{
		for(j=1;j<var1.cols-1;j++)
		{	
			for(k=0;k<3;k++)
			{
			gx= (var1.at<Vec3b>(i-1,j-1)[k]*(-1) + var1.at<Vec3b>(i,j-1)[k]*(-2) + var1.at<Vec3b>(i+1,j-1)[k]*(-1) + var1.at<Vec3b>(i+1,j)[k]*0 + var1.at<Vec3b>(i+1,j+1)[k] + var1.at<Vec3b>(i,j+1)[k]*2 + var1.at<Vec3b>(i-1,j+1)[k] + var1.at<Vec3b>(i-1,j)[k]*0 + var1.at<Vec3b>(i,j)[k]*0);
			gy= (var1.at<Vec3b>(i-1,j-1)[k] + var1.at<Vec3b>(i,j-1)[k]*0 + var1.at<Vec3b>(i+1,j-1)[k]*(-1) + var1.at<Vec3b>(i+1,j)[k]*(-2) + var1.at<Vec3b>(i+1,j+1)[k]*(-1) + var1.at<Vec3b>(i,j+1)[k]*0 + var1.at<Vec3b>(i-1,j+1)[k] + var1.at<Vec3b>(i-1,j)[k]*2 + var1.at<Vec3b>(i,j)[k]*0);
			sobal.at<Vec3b>(i,j)[k]=sqrt(gx*gx+gy*gy);
			xsobal.at<Vec3b>(i,j)[k]=sqrt(gx*gx);
			gauss.at<Vec3b>(i,j)[k]=(var1.at<Vec3b>(i-1,j-1)[k] + var1.at<Vec3b>(i,j-1)[k]*2 + var1.at<Vec3b>(i+1,j-1)[k] + var1.at<Vec3b>(i+1,j)[k]*2 + var1.at<Vec3b>(i+1,j+1)[k] + var1.at<Vec3b>(i,j+1)[k]*2 + var1.at<Vec3b>(i-1,j+1)[k] + var1.at<Vec3b>(i-1,j)[k]*2 + var1.at<Vec3b>(i,j)[k]*4)/16;
			blur.at<Vec3b>(i,j)[k]=(var1.at<Vec3b>(i-1,j-1)[k] + var1.at<Vec3b>(i,j-1)[k] + var1.at<Vec3b>(i+1,j-1)[k] + var1.at<Vec3b>(i+1,j)[k] + var1.at<Vec3b>(i+1,j+1)[k] + var1.at<Vec3b>(i,j+1)[k] + var1.at<Vec3b>(i-1,j+1)[k] + var1.at<Vec3b>(i-1,j)[k] + var1.at<Vec3b>(i,j)[k])/9;
			}
		}
	}
	namedWindow("Mean filter",WINDOW_AUTOSIZE);
	imshow("Mean filter",blur);
	namedWindow("X Sobal filter",WINDOW_AUTOSIZE);
	imshow("X Sobal filter",xsobal);
	namedWindow("Sobal filter",WINDOW_AUTOSIZE);
	imshow("Sobal filter",sobal);
	namedWindow("Gauss filter",WINDOW_AUTOSIZE);
	imshow("Gauss filter",gauss);
	namedWindow("original",WINDOW_AUTOSIZE);
	imshow("original",var1);
	waitKey(0);
	return 0;
}*/
/*int a;
Mat var1=imread("pic.jpg",0);
Mat var2(var1.rows,var1.cols,CV_8UC1,Scalar(0));
void change(int ,void*)
{	int i,j;
	for(i=0;i<var1.rows;i++)
	{
		for(j=1;j<var1.cols-1;j++)
		{
			var2.at<uchar>(i,j)=(var1.at<uchar>(i,j+1)-var1.at<uchar>(i,j-1))>a?0:255;
		}
	}
	for(i=1;i<var1.rows-1;i++)
	{
		for(j=0;j<var1.cols;j++)
		{
			var2.at<uchar>(i,j)=(var1.at<uchar>(i+1,j)-var1.at<uchar>(i-1,j))>a?0:255;
		}
	}
	imshow("borderchecking",var2);
}
int main()
{	
	
	
	namedWindow("borderchecking",WINDOW_AUTOSIZE);
	createTrackbar("limit","borderchecking",&a,255,change);
	
	
	
	namedWindow("original",WINDOW_AUTOSIZE);
	imshow("original",var1);
	waitKey(0);
	return 0;
}*/

 /*//given pattern
int main()
{	int k=0;
	int i,j;
	Mat pattern(200,200,CV_8UC3,Scalar(0,0,0));
	for(i=0;i<200;i++)
	{	k++;
		if(i<100)
		{

		for(j=0;j<200;j++)
		{	
			if(j<k)
			{
				pattern.at<Vec3b>(i,j)[0]=255;
				pattern.at<Vec3b>(i,j)[1]=255;
				pattern.at<Vec3b>(i,j)[2]=255;
			}
			else if(j>=k && j<200-k)
			{
			pattern.at<Vec3b>(i,j)[0]=255;
			}
			else if(j>(200-k))
			{
				pattern.at<Vec3b>(i,j)[1]=255;
			}
		}
		}
		else
		{
			for(j=0;j<200;j++)
			{	if(j<=(200-k))
				{
					pattern.at<Vec3b>(i,j)[0]=255;
					pattern.at<Vec3b>(i,j)[1]=255;
					pattern.at<Vec3b>(i,j)[2]=255;
									}
				else if(j>200-k && j<k)
				{
					pattern.at<Vec3b>(i,j)[2]=255;
				}
				else
				{
					pattern.at<Vec3b>(i,j)[1]=255;
				}
			}
		}
	}
	namedWindow("desgine",WINDOW_AUTOSIZE);
	imshow("desgine",pattern);
	waitKey(0);
	return 0;
}
*/
//////// EDGE DETECTION
/*int main()
{	
	int i,j,lx,ly,l,a;
	Mat var1=imread("pic.jpg",0);
	Mat edgedetect(var1.rows,var1.cols,CV_8UC1,Scalar(0));
	namedWindow("Edge Detection",WINDOW_AUTOSIZE);
	createTrackbar("track","Edge Detection",&a,255);
	while(1)
	{
	for(i=0;i<var1.rows;i++)
	{
		for(j=0;j<var1.cols;j++)
		{
			lx=(var1.at<uchar>(i,j+1)+(-1)*var1.at<uchar>(i,j-1))/2;
			ly=(var1.at<uchar>(i-1,j)+(-1)*var1.at<uchar>(i+1,j))/2;
			l=sqrt(lx*lx+ly*ly);
			
			if(l>a)
			{
				edgedetect.at<uchar>(i,j)=255;
			}
			else
			{
				edgedetect.at<uchar>(i,j)=0;
			}
		}
	}
	imshow("Edge Detection",edgedetect);
	waitKey(10);
	}
	return 0;
}*/

////sobel filter
/*int main()
{
	int i,j,gx,gy,lx,ly;
	Mat var1=imread("pic.jpg",0);
	Mat var2(var1.rows,var1.cols,CV_8UC1,Scalar(0));
	Mat var3(var1.rows,var1.cols,CV_8UC1,Scalar(0));
	for(i=1;i<var1.rows-1;i++)
	{
		for(j=1;j<var1.cols-1;j++)
		{   
			gx= (var1.at<uchar>(i-1,j-1)*(-1) + var1.at<uchar>(i,j-1)*(-2) + var1.at<uchar>(i+1,j-1)*(-1) + var1.at<uchar>(i+1,j)*0 + var1.at<uchar>(i+1,j+1) + var1.at<uchar>(i,j+1)*2 + var1.at<uchar>(i-1,j+1) + var1.at<uchar>(i-1,j)*0 + var1.at<uchar>(i,j)*0);
			gy= (var1.at<uchar>(i-1,j-1)*(-1) + var1.at<uchar>(i,j-1)*0 + var1.at<uchar>(i+1,j-1)*(1) + var1.at<uchar>(i+1,j)*(2) + var1.at<uchar>(i+1,j+1)*(1) + var1.at<uchar>(i,j+1)*0 + var1.at<uchar>(i-1,j+1)*(-1) + var1.at<uchar>(i-1,j)*(-2) + var1.at<uchar>(i,j)*0);
			lx=(var1.at<uchar>(i,j+1)+(-1)*var1.at<uchar>(i,j-1))/2;
			ly=(var1.at<uchar>(i-1,j)+(-1)*var1.at<uchar>(i+1,j))/2;
			var2.at<uchar>(i,j) = sqrt(lx*lx+ly*ly);
			var3.at<uchar>(i,j) = (sqrt(gx*gx+gy*gy))/8;
		}		
	}
	namedWindow("half",WINDOW_AUTOSIZE);
	imshow("half",var2);
	namedWindow("sobal by 8",WINDOW_AUTOSIZE);
	imshow("sobal by 8",var3);
	waitKey(0);
	return 0;
	

}*/

/// canning filter with trackbar (DOUBLE THEROSOLE)
/*int main()
{	
	int i,j,k,l,gx,gy,I,max,p,q;
	p=q=0;
	int Q;
	float val,a,b,c,d,min;
	Mat var1=imread("pic.jpg",0);
	Mat canny(var1.rows,var1.cols,CV_8UC1,Scalar(0));
	Mat final1(var1.rows,var1.cols,CV_8UC1,Scalar(0));
	namedWindow("Track with canninng",WINDOW_AUTOSIZE);
	createTrackbar("Range1","Track with canninng",&p,255);
	createTrackbar("Range2","Track with canninng",&q,255);
	for(i=1;i<var1.rows;i++)//gaussian
	{
		for(j=1;j<var1.cols;j++)
		{
			canny.at<uchar>(i,j)=(var1.at<uchar>(i-1,j-1) + var1.at<uchar>(i,j-1)*(2) + var1.at<uchar>(i+1,j-1) + var1.at<uchar>(i+1,j)*(2) + var1.at<uchar>(i+1,j+1) + var1.at<uchar>(i,j+1)*(2) + var1.at<uchar>(i-1,j+1) + var1.at<uchar>(i-1,j)*(2) + var1.at<uchar>(i,j)*(4))/16;
		}
	}
	for(i=1;i<var1.rows-1;i++)
	{
		for(j=1;j<var1.cols-1;j++)
		{
			gx= (var1.at<uchar>(i-1,j-1)*(-1) + var1.at<uchar>(i,j-1)*(-2) + var1.at<uchar>(i+1,j-1)*(-1) + var1.at<uchar>(i+1,j)*0 + var1.at<uchar>(i+1,j+1) + var1.at<uchar>(i,j+1)*2 + var1.at<uchar>(i-1,j+1) + var1.at<uchar>(i-1,j)*0 + var1.at<uchar>(i,j)*0);
			gy= (var1.at<uchar>(i-1,j-1)*(-1) + var1.at<uchar>(i,j-1)*0 + var1.at<uchar>(i+1,j-1)*(1) + var1.at<uchar>(i+1,j)*(2) + var1.at<uchar>(i+1,j+1)*(1) + var1.at<uchar>(i,j+1)*0 + var1.at<uchar>(i-1,j+1)*(-1) + var1.at<uchar>(i-1,j)*(-2) + var1.at<uchar>(i,j)*0);
			canny.at<uchar>(i,j) = (int)sqrt(gx*gx+gy*gy);
			//ar[i][j]=atan(gy/gx);
		}
	}
	
		//float ar[A.rows][A.cols];
	
	//cout << "sahil"<<endl;
	while(1)
	{
	for(i=1;i<var1.rows-1;i++)
	{
		for(j=1;j<var1.cols-1;j++)
		{   
			
			gx= (var1.at<uchar>(i-1,j-1)*(-1) + var1.at<uchar>(i,j-1)*(-2) + var1.at<uchar>(i+1,j-1)*(-1) + var1.at<uchar>(i+1,j)*0 + var1.at<uchar>(i+1,j+1) + var1.at<uchar>(i,j+1)*2 + var1.at<uchar>(i-1,j+1) + var1.at<uchar>(i-1,j)*0 + var1.at<uchar>(i,j)*0);
			//cout << "gx" <<gx<<endl;
			//cout << "gy" <<gy<<endl;
			gy= (var1.at<uchar>(i-1,j-1)*(-1) + var1.at<uchar>(i,j-1)*0 + var1.at<uchar>(i+1,j-1)*(1) + var1.at<uchar>(i+1,j)*(2) + var1.at<uchar>(i+1,j+1)*(1) + var1.at<uchar>(i,j+1)*0 + var1.at<uchar>(i-1,j+1)*(-1) + var1.at<uchar>(i-1,j)*(-2) + var1.at<uchar>(i,j)*0);
			final1.at<uchar>(i,j) =(int) sqrt(gx*gx+gy*gy);
			if(gx==0)
			{
				Q = 90;
			}
			else
			{
				val = abs((int)atan(gy/gx)*(180/22)*7);
				a = abs(val-0);
				b = abs(val-45);
				c = abs(val-90);
				d = abs(val-135);
				
					// if(gx*gy<0)
					// 	val+=90;
				
				min = a<b?a:b;
				min = min<c?min:c;
				min = min<d?min:d;
				if(min==a)
				{
					Q = 0;
				} 
				else if(min==b)
				{
					Q = 45;
				}
				else if(min==c)
				{
					Q = 90;
				}
				else
				{
					Q = 135;
				}

			 }
			//cout<< Q <<endl;
			 // cout<< "p = " << p<<endl;
			 // cout<< "q = " << q<<endl;


			if(Q==0)
			{
				max = canny.at<uchar>(i,j-1)>canny.at<uchar>(i,j+1)?canny.at<uchar>(i,j-1):canny.at<uchar>(i,j+1);
				if(final1.at<uchar>(i,j)<max)
				{
					final1.at<uchar>(i,j)=0;
				}
				else if(p<final1.at<uchar>(i,j)<q)
				{
					final1.at<uchar>(i,j) = 255;
				}
				else
				{
					final1.at<uchar>(i,j) = 0;
				}


			}
			else if(Q==45)
			{
				max = canny.at<uchar>(i+1,j-1)>canny.at<uchar>(i-1,j+1)?canny.at<uchar>(i+1,j-1):canny.at<uchar>(i-1,j+1);
				if(final1.at<uchar>(i,j)<max)
				{
					final1.at<uchar>(i,j)=0;
				}
				else if(p<final1.at<uchar>(i,j)<q)
				{
					final1.at<uchar>(i,j) = 255;
				}
				else
				{
					final1.at<uchar>(i,j) = 0;
				}
			}
			else if(Q==90)
			{
				max = canny.at<uchar>(i-1,j)>canny.at<uchar>(i+1,j)?canny.at<uchar>(i-1,j):canny.at<uchar>(i+1,j);
				if(final1.at<uchar>(i,j)<max)
				{
					final1.at<uchar>(i,j)=0;
				}
				else if(p<final1.at<uchar>(i,j)<q)
				{
					final1.at<uchar>(i,j) = 255;
				}
				else
				{
					final1.at<uchar>(i,j) = 0;
				}			
			}
			else
			{
				max = canny.at<uchar>(i-1,j-1)>canny.at<uchar>(i+1,j+1)?canny.at<uchar>(i-1,j-1):canny.at<uchar>(i+1,j+1);
				if(final1.at<uchar>(i,j)<max)
				{
					final1.at<uchar>(i,j)=0;
				}
				else if(p<final1.at<uchar>(i,j)<q)
				{
					final1.at<uchar>(i,j) = 255;
				}
				else
				{
					final1.at<uchar>(i,j) = 0;
				}
			}
		}
	}
	imshow("Track with canninng",final1);
	waitKey(10);
	}
	return 0;
}*/

/*////DEPTH FIRST SEARCH (DFS)(STAKS) by using Point2d build function in OPEN CV
int main()
{	
	int count=0;
	int i,j,k,l;
	Mat var1 = imread("pic.jpg",1);
	
	Mat dfs(var1.rows,var1.cols,CV_8UC3,Scalar(255,255,255));
	stack<Point2d> s;
	for(i=0;i<var1.rows;i++)
	{
		for(j=0;j<var1.cols;j++)
		{
			if(var1.at<Vec3b>(i,j)[2]==255 && dfs.at<Vec3b>(i,j)[2]!=0)
			{
				s.push(Point2d(i,j));
				count++;
				
				while(!s.empty())
				{
					Point2d a = s.top();
					s.pop();
					dfs.at<Vec3b>(a.x,a.y)[2] = 0;
					//cout<<"a.x,a.y = "<<a.x<<","<<a.y<<endl;
					for(k=-1;k<2;k++)
					{	
						for(l=-1;l<2;l++)
						{	if(((a.x)+k)>=0 && ((a.x)+k)<var1.rows && ((a.y)+l)<var1.cols && ((a.x)+k)>=0)
							{
								if(var1.at<Vec3b>((a.x)+k,(a.y)+l)[2]==255 && dfs.at<Vec3b>((a.x)+k,(a.y)+l)[2]!=0)
								{	

									
									s.push(Point2d((a.x)+k,(a.y)+l));
									dfs.at<Vec3b>((a.x)+k,(a.y)+l)[2]=0;

								}
							}
						}
					}

				}
			}
		}
	}
	cout<<count<<endl;
	namedWindow("original",WINDOW_AUTOSIZE);
	imshow("original",var1);
	namedWindow("for dfs",WINDOW_AUTOSIZE);
	imshow("for dfs",dfs);

	waitKey(0);
	return 0;
}
*/
////Erosion and DILATION

// int a=0;
// int i,j;
// Mat var1=imread("rstvaban.jpg",0);
// Mat var2=imread("rstvaban.jpg",0);
// Mat var3=imread("rstvaban.jpg",0);

// void limit(int , void*)
// {	
// 	var1=imread("rstvaban.jpg",0);
// 	var2=imread("rstvaban.jpg",0);
// 	for(i=0;i<var1.rows;i++)
// 	{
// 		for(j=0;j<var1.cols;j++)
// 		{
// 			if(var1.at<uchar>(i,j)>a)
// 			{
// 				var1.at<uchar>(i,j) = 255;
// 			}
// 			else
// 			{
// 				var1.at<uchar>(i,j) = 0;
// 			}
// 		}
// 	}
// 	for(i=1;i<var1.rows-1;i++)
// 	{
// 		for(j=1;j<var1.cols-1;j++)
// 		{	

// 			if(var1.at<uchar>(i-1,j-1)==0||var1.at<uchar>(i,j-1)==0||var1.at<uchar>(i+1,j-1)==0||var1.at<uchar>(i+1,j)==0||var1.at<uchar>(i+1,j+1)==0||var1.at<uchar>(i,j+1)==0||var1.at<uchar>(i-1,j+1)==0||var1.at<uchar>(i-1,j)==0)
// 			{
// 				var2.at<uchar>(i,j)=0;
// 			}
// 			else
// 			{
// 				var2.at<uchar>(i,j)=255;
// 			}
// 		}
// 	}

// 	for(i=1;i<var1.rows-1;i++)
// 	{
// 		for(j=1;j<var1.cols-1;j++)
// 		{	

// 			if(var2.at<uchar>(i-1,j-1)==255||var2.at<uchar>(i,j-1)==255||var2.at<uchar>(i+1,j-1)==255||var2.at<uchar>(i+1,j)==255||var2.at<uchar>(i+1,j+1)==255||var2.at<uchar>(i,j+1)==255||var2.at<uchar>(i-1,j+1)==255||var2.at<uchar>(i-1,j)==255)
// 			{
// 				var3.at<uchar>(i,j)=255;
// 			}
// 			else
// 			{
// 				var3.at<uchar>(i,j)=0;
// 			}
// 		}
// 	}

	
// 	imshow("DILUTION",var3);
// 	namedWindow("binary",WINDOW_AUTOSIZE);
// 	imshow("binary",var1);
// }

// int main()
// {	
	
// 	namedWindow("DILUTION",WINDOW_AUTOSIZE);
// 	createTrackbar("therosole","DILUTION",&a,255,limit);
// 	limit(0,0);
// 	waitKey(0);
// 	return 0;

// }

/*
int main()
{	
	int i,j,k,gx,gy;
	Mat var1= imread("pic.jpg",1);
	
	Mat sobal(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
	
	for(i=1;i<var1.rows-1;i++)
	{
		for(j=1;j<var1.cols-1;j++)
		{	
			for(k=0;k<3;k++)
			{
			gx= (var1.at<Vec3b>(i-1,j-1)[k]*(-1) + var1.at<Vec3b>(i,j-1)[k]*(-2) + var1.at<Vec3b>(i+1,j-1)[k]*(-1) + var1.at<Vec3b>(i+1,j)[k]*0 + var1.at<Vec3b>(i+1,j+1)[k] + var1.at<Vec3b>(i,j+1)[k]*2 + var1.at<Vec3b>(i-1,j+1)[k] + var1.at<Vec3b>(i-1,j)[k]*0 + var1.at<Vec3b>(i,j)[k]*0);
			gy= (var1.at<Vec3b>(i-1,j-1)[k] + var1.at<Vec3b>(i,j-1)[k]*0 + var1.at<Vec3b>(i+1,j-1)[k]*(-1) + var1.at<Vec3b>(i+1,j)[k]*(-2) + var1.at<Vec3b>(i+1,j+1)[k]*(-1) + var1.at<Vec3b>(i,j+1)[k]*0 + var1.at<Vec3b>(i-1,j+1)[k] + var1.at<Vec3b>(i-1,j)[k]*2 + var1.at<Vec3b>(i,j)[k]*0);
			sobal.at<Vec3b>(i,j)[k]=sqrt(gx*gx+gy*gy);
			
			}
		}
	}
	
	namedWindow("Sobal filter",WINDOW_AUTOSIZE);
	imshow("Sobal filter",sobal);
	namedWindow("original",WINDOW_AUTOSIZE);
	imshow("original",var1);
	waitKey(0);
	return 0;
}
*/
/*////Assignment
Mat var1 =imread("shapes.jpg",1);
void fillcolor(int event, int j, int i, int flags, void* userdata)
{	
	int k,l;
		
	 if  ( event == EVENT_LBUTTONDOWN )
     	{	Mat dfs(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
     		stack<Point2d> s;
          	//cout << "Left button of the mouse is clicked - position (" << i << ", " << j << ")" << endl;
          
				s.push(Point2d(i,j));
				var1.at<Vec3b>(i,j)[2]=255;
				
				
				while(!s.empty())
				{	
					//cout<<"hai"<<endl;
					Point2d a = s.top();
					s.pop();
					dfs.at<Vec3b>(a.x,a.y)[2] = 255;
					var1.at<Vec3b>(a.x,a.y)[2] = 0;
					
					
					for(k=-1;k<2;k++)
					{	
						for(l=-1;l<2;l++)
						{	if(((a.x)+k)>=0 && ((a.x)+k)<var1.rows && ((a.y)+l)<var1.cols && ((a.x)+k)>=0)
							{
								if(var1.at<Vec3b>((a.x)+k,(a.y)+l)[0]==255&&var1.at<Vec3b>((a.x)+k,(a.y)+l)[1]==255&&var1.at<Vec3b>((a.x)+k,(a.y)+l)[2]==255 && dfs.at<Vec3b>((a.x)+k,(a.y)+l)[2]!=255)
								{	
									
									s.push(Point2d((a.x)+k,(a.y)+l));
									dfs.at<Vec3b>((a.x)+k,(a.y)+l)[2]=255;
									var1.at<Vec3b>((a.x)+k,(a.y)+l)[2]=0;

								}
							}
						}
					}

				}
				//cout<<"hello"<<endl;
			
				
     }

    			imshow("fill bucket",var1);
				waitKey(0);
     
}
int main()
{
	Mat var1=imread("shapes.jpg",1);
	namedWindow("fill bucket",WINDOW_AUTOSIZE);
	setMouseCallback("fill bucket", fillcolor, NULL);
	fillcolor(0,0,0,0,0);
	waitKey(0);
	return 0;
}
*/

/*//////HOUGH TRANSFORMATION (making line and checking each point)

int main()
{	
	int i,j,lx,ly,l;
	int r,q;
	int max =0;
	float dis,tdis;
	tdis=10.0;
	int m,c;
	int a=150;
	Mat var1=imread("pic.jpg",0);
	int maxr =(int)sqrt((var1.rows)*(var1.rows)+(var1.cols)*(var1.cols));
	int hough[1060][91];
	Mat edgedetect(var1.rows,var1.cols,CV_8UC1,Scalar(0));
	namedWindow("Edge Detection",WINDOW_AUTOSIZE);
	
	
	for(i=0;i<var1.rows;i++)
	{
		for(j=0;j<var1.cols;j++)
		{
			lx=(var1.at<uchar>(i,j+1)+(-1)*var1.at<uchar>(i,j-1))/2;
			ly=(var1.at<uchar>(i-1,j)+(-1)*var1.at<uchar>(i+1,j))/2;
			l=sqrt(lx*lx+ly*ly);
			
			if(l>a)
			{
				edgedetect.at<uchar>(i,j)=255;
			}
			else
			{
				edgedetect.at<uchar>(i,j)=0;
			}
		}
	}
	//cout<<"hai"<<endl;
	for(r=0;r<maxr;r++)
	{
		for(q=0;q<91;q++)
		{
			hough[r][q]=0;

		}
	}
	//cout<<hough[5][1]<<endl;
	int count = 0;
	for(r=0;r<maxr;r++)
	{	
		for(q=0;q<91;q++)
		{
			for(i=0;i<var1.rows;i++)
			{
				for(j=0;j<var1.cols;j++)
				{	
					//cout<<count++<<endl;
					if(edgedetect.at<uchar>(i,j)==255)
					{
						//cout<<i<<","<<j<<endl;
						if(q!=0 && q!=90)
						{
							dis = abs((j+(i/tan(q))-(r/sin(q)))/(sqrt(1+(1/tan(q))*(1/tan(q)))));
							cout<<dis<<endl;
							if(dis<=tdis)
							{
								hough[r][q]++;
							}	
						}
						else if(q==0)
						{
							dis =abs(i-r);
							cout<<"usfffffffffffffffffffffffffffffffffffffffffffffffffffffffffffbdvhbdfyuvj"<<endl;
							if(dis<=tdis)
							{
								hough[r][q]++;
							}
						}
						else
						{
							dis =abs(j-r);
							if(dis<=tdis)
							{
								hough[r][q]++;
							}
						}
					}
				}
			}
		}
	}
	
	for(r=0;r<maxr;r++)
	{
		for(q=0;q<91;q++)
		{
			if(hough[r][q]>=max)
			{
				max= hough[r][q];
				m=r;
				c=q;
			}
		}
	}
	cout << "Y" << "X(" << (1/tan(c))<< ")="<< m*(1/(sin(q)))<<endl;
	imshow("Edge Detection",edgedetect);
	waitKey(0);
	
	return 0;
}
*/
/*////another haugh algorithm
int main()
{
	int i,j,lx,ly,l;
	int count=0;
	int r,q;
	int max =0;
	float dis,tdis;
	tdis=10.0;
	int m,c;
	int a=50;
	Mat var1=imread("shapes.jpg",0);
	int maxr =(int)sqrt((var1.rows)*(var1.rows)+(var1.cols)*(var1.cols));
	int hough[maxr][360];
	Mat edgedetect(var1.rows,var1.cols,CV_8UC1,Scalar(0));
	Mat line(var1.rows,var1.cols,CV_8UC3,Scalar(0,0,0));
	namedWindow("Edge Detection",WINDOW_AUTOSIZE);
	
	
	for(i=0;i<var1.rows;i++)
	{
		for(j=0;j<var1.cols;j++)
		{
			lx=(var1.at<uchar>(i,j+1)+(-1)*var1.at<uchar>(i,j-1))/2;
			ly=(var1.at<uchar>(i-1,j)+(-1)*var1.at<uchar>(i+1,j))/2;
			l=sqrt(lx*lx+ly*ly);
			
			if(l>a)
			{
				edgedetect.at<uchar>(i,j)=255;
			}
			else
			{
				edgedetect.at<uchar>(i,j)=0;
			}
		}
	}
	for(r=0;r<maxr;r++)
	{
		for(q=0;q<=360;q++)
		{
			hough[r][q]=0;

		}
	}
	for(i=0;i<var1.rows;i++)
	{
		for(j=0;j<var1.cols;j++)
		{
			if(edgedetect.at<uchar>(i,j)==255)
			{
				for(q=0;q<360;q++)
				{	//cout<<"hai"<<endl;
					r=round(abs(j*cos((q*22)/(7*180))+i*sin((q*22)/(7*180))));
					//cout<<r<<endl;
					hough[r][q]++;
					//cout<<r<<","<<q<<"="<<hough[r][q]<<endl;

				}
			}	
		}

	}

	for(r=0;r<maxr;r++)
	{
		for(q=0;q<91;q++)
		{
			if(hough[r][q]>=max)
			{
				max= hough[r][q];
				m=r;
				c=q;
			}
		}
	}
	for(i=0;i<var1.rows;i++)
	{
		for(j=0;j<var1.cols;j++)
		{
			if(abs(j*cos((57*22)/(7*180))+i*sin((57*22)/(7*180)))==253)
			{
				line.at<Vec3b>(i,j)[2]=255;
			}
		}
	}
	//edgedetect.at<uchar>(220,2)=255;
	// cout << "Y +" << "X(" << (1/tan((c*22)/(7*180)))<< ")="<< m*(1/(sin((c*22)/(7*180))))<<endl;
	// cout<<c<<endl;
	cout<< m <<","<< c <<"="<<max<<endl;
	namedWindow("linewindow",WINDOW_AUTOSIZE);
	imshow("linewindow",line);
	imshow("Edge Detection",edgedetect);

	waitKey(0);
	return 0;
}*/

///haugh copied from internet
/*int main()
{
	Mat src = imread("rstvaban.jpg", 0);
	Mat dst,cdst;
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);
	vector<Vec4i> lines;
  	HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
  	for( size_t i = 0; i < lines.size(); i++ )
  	{
    	Vec4i l = lines[i];
    	line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  	}	
	imshow("windows",dst);
	namedWindow("windows2",WINDOW_AUTOSIZE);
	imshow("windows2",cdst);
	waitKey(0);
	return 0;
}*/

//////// canning video in web camera
/*Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold =50;
//int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;



// *
//  * @function CannyThreshold
//  * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);//all dst pixels is initialise to 0;

  src.copyTo( dst, detected_edges);//mask – Operation mask. Its non-zero elements indicate which matrix elements need to be copied.
  
  imshow("Windows", dst );
 }
int main()
{
	VideoCapture(a);
	a.open(0);
	namedWindow("Windows",WINDOW_AUTOSIZE);//WINDOW_NORMAL,WINDOW_OPENGL
	createTrackbar("track","Windows",&lowThreshold,255,CannyThreshold);//userdata – User data that is passed as is to the callback. It can be used to handle trackbar events without using global variables
	while(a.isOpened())
	{
		a>>src;
		dst.create( src.size(), src.type() );
		cvtColor( src, src_gray, CV_BGR2GRAY );
		CannyThreshold(0,0);
		waitKey(30);
	}
	return 0;

}*/
/*int main()
{
	Mat var1=imread("pic.jpg",1);
	Mat img;
	// line(var1,,200,Scalar(255,0,0),10,8,0);
	cvLine("pic.jpg",(0,0),(511,511),(255,0,0),5);
	namedWindow("windows",WINDOW_AUTOSIZE);
	imshow("windows",img);
	return 0;
}*/

/*////getting picture from web camera
Mat img;
bool play=true;
// String s="../Pictures/image";
// int ctr=0;

void saveimg(int event, int j, int i, int flags, void* userdata)
{	
	if  ( event == EVENT_LBUTTONDOWN )
	{
		cout<<"sahil"<<endl;
		imwrite( "../Pictures/image.jpg", img);
		play=false;
	}
}
int main()
{
	VideoCapture web_cam(0);
	namedWindow("camera",WINDOW_AUTOSIZE);
	setMouseCallback("camera", saveimg, NULL);
	while(web_cam.isOpened())
	{
		web_cam>>img;
		imshow("camera",img);
		waitKey(30);
		if(play== false)
		{
			break;
		}

	}
	return 0;
}*/


// Mat img;
// Mat result;
// int a=35;
// void change(int, void*)
// {
// 		//cvtColor(img,gray,CV_BGR2GRAY);
// 		//blur( gray, gray, Size(3,3) );
// 		//Canny(gray,canny_edge,a,a*3,3);
// 	int i;
// 	int j; 
// 		for(i=0;i<img.cols;i++)
// 		{
// 			for(j=0;j<img.rows;j++)
// 			{
// 				if(img.at<Vec3b>(i,j)[0]+img.at<Vec3b>(i,j)[1]+img.at<Vec3b>(i,j)[2]>a)
// 				{
// 					img.at<Vec3b>(i,j)[0]=255;
// 					img.at<Vec3b>(i,j)[1]=255;
// 					img.at<Vec3b>(i,j)[2]=255;
// 				}
// 				else
// 				{
// 					img.at<Vec3b>(i,j)[0]=0;
// 					img.at<Vec3b>(i,j)[1]=0;
// 					img.at<Vec3b>(i,j)[2]=0;
// 				}
// 			}
// 		}  
// 		imshow("detect_marker",img);
// }
// int main()
// {
// 	VideoCapture saved_video;
// 	saved_video.open("task_marker_22_seconds.avi");
// 	namedWindow("detect_marker",WINDOW_AUTOSIZE);
// 	createTrackbar("therosole","detect_marker",&a,250,change);
// 	cout<<"bye"<<endl;
// 	while(saved_video.isOpened())
// 	{
// 		saved_video >> img;
// 		change(0,0);
// 		cout<<"hai"<<endl;
// 		waitKey(30);
// 	}
// 	return 0;
// }


// /*Mat src; Mat src_gray;
// int thresh = 100;
// int max_thresh = 255;
// RNG rng(12345);

// /// Function header
// void thresh_callback(int, void* );

// /** @function main */
// int main()
// {
//   /// Load source image and convert it to gray
// 	VideoCapture a;
// 	a.open("task_marker_22_seconds.avi");
// 	namedWindow( "source", CV_WINDOW_AUTOSIZE );
// 	createTrackbar( " Threshold:", "source", &thresh, max_thresh, thresh_callback );
  
//   while(a.isOpened())
//   {
//   	a>>src;
//   	imshow( "source", src );
//   	cvtColor( src, src_gray, CV_BGR2GRAY );
//   	blur( src_gray, src_gray, Size(3,3) );
//   	thresh_callback( 0, 0 );
// 	waitKey(30);
//   }

//   /// Convert image to gray and blur it
  

//   /// Create Window
//   //char* source_window = "Source";
  

  
  
//   return(0);
// }

// /** @function thresh_callback */
// void thresh_callback(int, void* )
// {
//   Mat threshold_output;
//   vector<vector<Point> > contours;
//   vector<Vec4i> hierarchy;

//   /// Detect edges using Threshold
//   threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
//   /// Find contours
//   findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//   /// Approximate contours to polygons + get bounding rects and circles
//   vector<vector<Point> > contours_poly( contours.size() );
//   vector<Rect> boundRect( contours.size() );
//   //vector<Point2f>center( contours.size() );
//   //vector<float>radius( contours.size() );

//   for( int i = 0; i < contours.size(); i++ )
//      { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
//        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
//        //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
//      }


//   /// Draw polygonal contour + bonding rects + circles
//   Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
//   for( int i = 0; i< contours.size(); i++ )
//      {
//        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
//        rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
//        //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
//      }

//   /// Show in a window
//   namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//   imshow( "Contours", drawing );
// }*/



//////RGB to HSV
/*int hh=180;
int hl=0;
int sh=255;
int vh=255;
int sl=0;
int vl=0;
Mat rgb(500,500,CV_8UC3,Scalar(0,0,0));
Mat hsv(rgb.rows,rgb.cols,CV_8UC3,Scalar(0,0,0));

void change(int , void*)
{	
	//cvtColor(rgb,hsv,CV_BGR2HSV);
	Mat hsv2;
	inRange(hsv,Scalar(vl,sl,hl), Scalar(vh,sh,hh),hsv2);
	imshow("hsvwindow",hsv2);
}
int main()
{	
	int i,j;
	Mat rgb(500,500,CV_8UC3,Scalar(0,0,0));
	for(i=0;i<500;i++)
	{
		for(j=0;j<500;j++)
		{
			if(i<100)
			{
				rgb.at<Vec3b>(i,j)[0]=200;
			}
			else if(i>150&& i<300)
			{
				rgb.at<Vec3b>(i,j)[1]=200;
			}
			else if(i>400 &&i<450)
			{
				rgb.at<Vec3b>(i,j)[2]=200;
			}
		}
	}
	
	cvtColor(rgb,hsv,CV_RGB2HSV);

	namedWindow("rgbwindow",WINDOW_AUTOSIZE);
	namedWindow("hsvwindow",WINDOW_AUTOSIZE);
	imshow("rgbwindow",rgb);
	imshow("hsvwindow",hsv);
	printf("%d",rgb.at<Vec3b>(50,100)[0]);

	//namedWindow("trackbarhsv",WINDOW_AUTOSIZE);
	// createTrackbar("huehigh","hsvwindow",&hh,255,change);
	// createTrackbar("huelow","hsvwindow",&hl,255,change);
	// createTrackbar("satutatin high","hsvwindow",&sh,255,change);
	// createTrackbar("satutatin low","hsvwindow",&sl,255,change);
	// createTrackbar("Value high","hsvwindow",&vh,255,change);
	// createTrackbar("Value low","hsvwindow",&vl,255,change);
	// change(0,0);
	waitKey(0);

	//change(0,0,0);
	return 0;
}
*/



// Mat src; Mat src_gray;
// int thresh = 100;
// int max_thresh = 255;
// RNG rng(12345);

// /// Function header
// void thresh_callback(int, void* );

// /** @function main */
// int main()
// {
//   /// Load source image and convert it to gray
//   src = imread( "pic.jpg", 1 );

//   /// Convert image to gray and blur it
//   cvtColor( src, src_gray, CV_BGR2GRAY );
//   blur( src_gray, src_gray, Size(3,3) );

//   /// Create Window
//   //char* source_window = "Source";
//   namedWindow( "Source", CV_WINDOW_AUTOSIZE );
//   imshow( "Source", src );

//   createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
//   thresh_callback( 0, 0 );

//   waitKey(0);
//   return(0);
// }

// /** @function thresh_callback */
// void thresh_callback(int, void* )
// {
//   Mat canny_output;
//   vector<vector<Point> > contours;=
//   vector<Vec4i> hierarchy;

//   /// Detect edges using canny
//   Canny( src_gray, canny_output, thresh, thresh*2, 3 );
//   /// Find contours
//   findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//   /// Draw contours
//   Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//   for( int i = 0; i< contours.size(); i++)
//      {
//        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
//      }

//   /// Show in a window
//   namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//   imshow( "Contours", drawing );
// }



/** Global variables */
//  FACE DETECTOR FOR given situation


//String face_cascade_name = "haarcascade_frontalface_alt.xml";
//  String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
//  CascadeClassifier face_cascade;
//  CascadeClassifier eyes_cascade;
//  string window_name = "Capture - Face detection";
//  RNG rng(12345);
 
//  queue<Point2d> q;
//  int flags=0;
//  void detectAndDisplay( Mat frame );

//  /** @function main */
//  int main( int argc, const char** argv )
//  {
//    // CvCapture* capture;

//  	VideoCapture capture("video.mp4");
//   	Mat frame;

//    //-- 1. Load the cascades
//    if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
//    if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

//    //-- 2. Read the video stream
//    //capture = cvCaptureFromCAM( -1 );
//    if( capture.isOpened() )
//    {
//      while( true )
//      {	
     	
//    		capture>>frame;
//    		resize(frame,frame,Size(500,500),0,0,INTER_CUBIC);
   		
//    //-- 3. Apply the classifier to te frame
//        if( !frame.empty() )
//        { detectAndDisplay( frame ); }
//        else
//        { printf(" --(!) No captured frame -- Break!"); break; }

//        // int c = waitKey(10);
//        // if( (char)c == 'c' ) { break; }
//    waitKey(20);
//    //Sleep(1000);
//       }
//    }

//    return 0;
//  }

// /** @function detectAndDisplay */
// void detectAndDisplay( Mat frame )
// {	int count=0;
//   vector<Rect> faces;
//   Mat frame_gray;

//   cvtColor( frame, frame_gray, CV_BGR2GRAY);
//   //equalizeHist( frame_gray, frame_gray );

//   //-- Detect faces
//   face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
//   if(faces.size()==0 && !q.empty())
//   {
  	
//    Point2d started  = q.back();
//    Point2d end =q.front();
//    // cout<<"started" << started.x<< endl;
//    // cout<< "end"<< end.x<< endl;
//    	if(q.size()>5)
//    	{
//    		if(started.x<end.x)
//    		{
//    			cout<<"exit"<<endl;
//    		}
//    		else
//    		{
//    			cout<<"Entered"<<endl;
//    		}
//    	}
//    while(!q.empty())
//    {
//    	q.pop();
//    }

//    }
//   //cout<<"face n. ="<<faces.size()<<endl;
 

//   for( size_t i = 0; i < faces.size(); i++)
//   {

//     Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
//     q.push(Point2d(center.x,center.y));
//     ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

//     Mat faceROI = frame_gray( faces[i] );
//     std::vector<Rect> eyes;

//     //-- In each face, detect eyes
//     eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

//     for( size_t j = 0; j < eyes.size(); j++ )
//      {	
//        Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
//        int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
//        circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
//      }
//   }
//   //-- Show what you got
//   imshow( window_name, frame);
//   //cout<<"N. of people"<<count<<endl;
//  }


