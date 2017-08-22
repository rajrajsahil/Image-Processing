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

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
// #include <opencv2/viz/viz3d.hpp>
// #include <opencv2/viz/vizcore.hpp>
// #include <opencv2/viz/widget_accessor.hpp>
// #include <opencv2/viz/types.hpp>
// #include <opencv2/viz/widgets.hpp>
using namespace std;
using namespace cv;

int main()
{
	//int fd;
	//char *myfifo = myfifo;
	//char buf[MAX_BUF];
	//fd = open(myfifo,O_RDWR);
	//read(fd,buf,MAX_BUF);
	Mat frame;
	namedWindow("windows",WINDOW_AUTOSIZE);
	VideoCapture raspicam("mypipe");
	cout<<"hii"<<endl;
	//waitKey(1000);
	while(raspicam.isOpened())
	{	
		raspicam >> frame;
		imshow("windows",frame);
		waitKey(25);
		
	}
	return 0;
}

/*int main()
{
	VideoCapture cap("mypipe"); 
    if( !cap.isOpened()){
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
    cap.set(CV_CAP_PROP_POS_FRAMES,count-1); //Set index to last frame
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);

    while(1)
    {
        Mat frame;
        bool success = cap.read(frame); 
        if (!success){
          cout << "Cannot read  frame " << endl;
          break;
        }
        imshow("MyVideo", frame);
        //if(waitKey(0) == 27) break;
        waitKey(25);
    }
    return 0;
}*/

