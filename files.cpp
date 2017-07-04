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
using namespace std;
using namespace cv;

int main()
{	
	char val;
	int val1;
	Mat map1_x(img1.rows,img1.cols,CV_16SC2,Scalar(0));
	ifstream file;
	file.open("map1_1");
	if(file.good())
	{
		for(int i=0;i<960;i++)
		{
			for(int j=0;j<640;j++)

		}
	}
	else
	{
		cout<<"prob"<<endl;
	}
	return 0;
}