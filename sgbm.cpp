Mat g1, g2;
Mat disp, disp8;
Mat img1 = imread("left.ppm");
Mat img2 = imread("right.ppm");
int SADWindowSize1=5;
int numberOfDisparities1=112;
int preFilterCap1=61;
int minDisparity1 =64;
int uniquenessRatio1=5;
int speckleWindowSize1=50;
int speckleRange1=1;
int disp12MaxDiff1=30;
int p11 = 600;
int p21 =2400;
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
    sbm.fullDP = true;
    sbm.P1 = p11;
    sbm.P2 = p21;
    sbm(img1,img2, disp);
// }


	normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
	imshow("disp8", disp8);
}
int main()
{
	
	// Mat disp, disp8;

	//char* method = argv[3];
	//char* method = "SGBM";

	//img1 = imread(argv[1]);
	//img2 = imread(argv[2]);
	// img1 = imread("left.ppm");
	// img2 = imread("right.ppm");

	// cvtColor(img1, g1, CV_BGR2GRAY);
	// cvtColor(img2, g2, CV_BGR2GRAY);
	namedWindow("disp8",WINDOW_NORMAL);
	namedWindow("disp",WINDOW_NORMAL);

	createTrackbar("SADWindowSize","disp",&SADWindowSize1,27,therosole);
	createTrackbar("numberOfDisparities","disp",&numberOfDisparities1,500,therosole);
	createTrackbar("preFilterCap","disp",&preFilterCap1,27,therosole);
	createTrackbar("minDisparity","disp",&minDisparity1,100,therosole);
	createTrackbar("uniquenessRatio","disp",&uniquenessRatio1,15,therosole);
	createTrackbar("speckleWindowSize","disp",&speckleWindowSize1,200,therosole);
	createTrackbar("speckleRange","disp",&speckleRange1,2,therosole);
	createTrackbar("disp12MaxDiff","disp",&disp12MaxDiff1,250,therosole);
	createTrackbar("p1","disp",&p11,250,therosole);
	createTrackbar("p2","disp",&p21,3200,therosole);
	therosole(0,0);

	// namedWindow("left",WINDOW_NORMAL);
	// namedWindow("right",WINDOW_NORMAL);
	// namedWindow("disp",WINDOW_NORMAL);
	// imshow("left", img1);
	// imshow("right", img2);
	
	waitKey(0);

	return(0);
}
