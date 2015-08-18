#include <iostream>
#include <armadillo>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <fstream>
#include <math.h>
#include<ctime>

using namespace cv;
using namespace std;
using namespace arma;

class vanishingPt
{
	public:
	cv::Mat image, img, gray;
	cv::Mat frame;
	vector< vector<int> > points;
	mat A,b, prevRes;
	mat Atemp, btemp, res, aug, error, soln;
	float epsilon;
	float m,c;
	int minlength;
	vector<int> temp;
	vector<cv::Vec4i> lines_std;
	cv::VideoCapture cap;
	double temperr;
	vanishingPt()
	{
		cv::namedWindow("win", 2);
		cv::namedWindow("Lines", 2);

		clock_t begin, end;

		cap = VideoCapture("road.m4v");
		if( cap.isOpened() )
		{
			cap.read(frame);
        	image= cv::Mat(cv::Size(frame.rows,frame.cols), CV_8UC1, 0.0);
		}
		minlength = image.cols * image.cols * 0.001 ; // sqr of 20% of image width is minimum requirement for any line

	    int flag=0;
		while( cap.isOpened() )   // check if we succeeded
		{
		    if ( ! cap.grab() )
		        continue;

			if(!cap.retrieve(img))
				continue;

			//it's advised not to modify image stored in the buffer structure of the opencv.
			frame  = img.clone();

			begin = clock();
			
			cv::cvtColor(frame,image , cv::COLOR_BGR2GRAY);
			cv::resize(image, image, cv::Size(480,320));
			cv::equalizeHist(image, image);
			init(image, prevRes);
			makeLines(flag);
			eval();

			end = clock();
			cout<<"fps: "<<1/(double(end-begin)/CLOCKS_PER_SEC)<<endl;


		    int k = cv::waitKey(1);
		    if ( k==27 )
		        break;
		}

	}
	void init(cv::Mat image, mat prevRes)
	{
		cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);

    	lines_std.clear();// Detect the lines
    	ls->detect(image, lines_std);

    	// Show found lines
    	cv::Mat drawnLines (image);
    	

		for(int i=0; i<lines_std.size(); i++)
		{
            /*remove vertical lines and
				short lines*/

			if ( abs(lines_std[i][0]-lines_std[i][2]) < 10 || abs(lines_std[i][1]-lines_std[i][3]) < 10) //check if almost vertical
				continue;
			if( ((lines_std[i][0]-lines_std[i][2])*(lines_std[i][0]-lines_std[i][2]) +(lines_std[i][1]-lines_std[i][3])*(lines_std[i][1]-lines_std[i][3])) < minlength)
				continue;

			for(int j=0; j<4; j++)
			{
				if(j%2 == 1)
				temp.push_back(lines_std[i][j]);
				else
				temp.push_back(lines_std[i][j]);

			}

			points.push_back(temp);
			temp.clear();
		}
		ls->drawSegments(drawnLines, lines_std);
		cv::imshow("Lines", drawnLines);
	}
	void makeLines(int flag)
	{
	    A = zeros<mat>(points.size(), 2);
	    b = zeros<mat>(points.size(), 1);

	    for(int i=0; i<points.size(); i++)
	    {
            A(i,1)=(points[i][2]-points[i][0]);				//x2-x1
			A(i,0)=-(points[i][3]-points[i][1]);			//-(y2-y1)
			b(i,0)=A(i,0)*points[i][0]+A(i,1)*points[i][1];	//-(y2-y1)*x1 + (x2-x1)*y1

	    }
	}
	void eval()
	{
		soln= zeros<mat>(2,1);
		double err = 999999999991;
		for(int i=0; i<points.size(); i++)
		{
			for(int j=0; j<points.size(); j++)
			{
				if(i >= j)
				continue;

				uvec indices;
				indices << i << j;
				Atemp = A.rows(indices);
				btemp = b.rows(indices);

				if(rank(Atemp) != 2)
				continue;

				res = calc(Atemp, btemp);

				if(res.n_rows==0 || res.n_cols==0)
					continue;

				error = A*res - b;
				error = error/1000;

				temperr = 0;
				for(int i=0; i<error.n_rows ; i++)
                    temperr+=(error(i,0)*error(i,0))/1000;

				temperr/=1000000;
				if(err > temperr)
				{
					soln = res;
					err = temperr;
				}
			}
		}


		if(soln(0,0) > 0 && soln(0,0) < image.cols && soln(1,0) > 0 && soln(1,0) < image.rows)
			cv::circle(image, Point(soln(0,0), soln(1,0)), 25, cv::Scalar(0,0,255), 10);
		cv::imshow("win", image);
		points.clear();
		prevRes = soln;
	}

	mat calc(mat A, mat b)
	{
	    mat x = zeros<mat>(2,1);
		solve(x,A,b);
	    return x;
	}
};


int main()
{
    vanishingPt obj;
    cv::destroyAllWindows();
    return 0;
}
