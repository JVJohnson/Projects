#include <iostream>
#include <cstdlib>		//random for node choices
#include <ctime>		//time for the random seed
#include <math.h>
//#include <vector>	


//Opencv2 for displaying stuff cause i like it
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace std;
using namespace cv;



const int mapW = 1000;
const int mapH = 1000;


const int startX = 500;
const int startY = 500;

const int nodeNum = 5;


// void draw(Point* nodes)
// {

// }


int dist();


int main()
{

	struct Point
	{
		int x;
		int y;
		Point* parent;
	}

	Mat I;

	I = Mat(mapH, mapW, CV_8UC3, Scalar(255,255,255) );
	namedWindow("CaR-RT", 1);
	imshow("CaR-RT", I);
	waitKey(0);



	Point* nodes[nodeNum];


	Point start;
	start.x = startX;
	start.y = startY;
	start.parent = &start;


	nodes[0] = &start;
	
	srand(std::time(0));

	for(size_t i = 0; i < nodeNum; i++)
	{
		Point newP;

		newP.x = rand() % mapW;
		newP.y = rand() % mapH;
		newP.parent = nodes[0];




		

	}

	return 0;
}