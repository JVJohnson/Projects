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

//width & height of map
const int mapW = 1000;
const int mapH = 1000;

//initial node
const int startX = 500;
const int startY = 500;
const double startTh = 0;

const double stDeltX = 0;



const double carLength = 3;
const double maxTheta = atan(1)/8;
const double maxSpeed = 5;

const int DEF = 100;


//max number of nodes
const int nodeNum = 500;


const int nodeRad = 2;
const int lineThickness = 2;

//Epsilon is a fraction of average of w and h
const double EPSILON =  ( (mapW + mapH)/2 ) /7 ;

int currNodeNum = 0;


//node leaf. contains current state and derivative
struct NodeLeaf
	{
		Point self;
		double angle;

		double speed;
		double deltaAng;



		NodeLeaf* parent;
	};

//draws the nodes on an image and connects the parents
void draw( NodeLeaf *nodes, Mat img)
{

	for(int i = 0; i<currNodeNum; i++ ) //iterates through each node
	{
		// cout << "Iteration: " << i <<  endl;
		
		// cout << "	suceeded" << endl;
		circle(img, nodes[i].self, nodeRad, CV_RGB(0,0,0), nodeRad);
		// cout << "	drew: " << nodes[i].self << endl;
		// cout << "	node parent: ";
		// cout << (nodes[i].parent) << endl;
		
		//line(img, nodes[i].self, nodes[i].parent->self, CV_RGB(0,0,0), lineThickness );
		// cout << "	drew line" << endl;
	}


	imshow("CaR-RT", img);
	waitKey(4);
};



NodeLeaf drive(NodeLeaf init, Mat img)
{
	double us = init.speed;

	double x = init.self.x;
	double y = init.self.y;
	double th = init.angle;
	

	for(int step = 1; step < 20; step++)
	{
		x = x + us*cos(th);
		y = y + us*sin(th);
		th = th + us/carLength * tan(init.deltaAng);
		circle(img, Point(x,y), 1, CV_RGB(0,0,255), 1);
		
		imshow("CaR-RT", img);
		waitKey(4);
		// cout << "drew: " << x << ", " << y << endl;

	};
	NodeLeaf end;
	end.self.x = x;
	end.self.y = y;
	end.angle  = th;
	return end;

}



int main()
{

	
	//matrix of the image space
	Mat img(mapH, mapW, CV_8UC3, Scalar(255,255,255));
	


	//array containing all nodes
	NodeLeaf nodes[nodeNum];

	//setting up beginnning node
	NodeLeaf start;
	start.self = Point(startX, startY);
	start.parent = &start;


	//add start node
	nodes[currNodeNum] = start;
	currNodeNum++;
	srand(std::time(0));

	NodeLeaf newN;

	namedWindow("CaR-RT", CV_WINDOW_NORMAL);





	//iterates through node-making until desired number of nodes is reached
	while(currNodeNum < nodeNum)
	{
		draw(nodes, img);
		
		//creates new node with random position
		
		//____________TEST________________
		//just a test version to see if the kinematic model works without having to implement non-holonmic regression for it
		
		int randIndex = rand() % currNodeNum;

		NodeLeaf nextNode(nodes[randIndex]); 

		//gets within range of (-DEF,DEF), then multilies by max theta to get (-maxTheta , maxTheta)
		double dTheta = (    ( rand()%(2*DEF) )- DEF   )*1.0/DEF * maxTheta;
				
		//uses same  thing as above to get range (-maxSpeed, maxSpeed)
		double dSpeed = (    ( rand()%(2*DEF) )- DEF   )*1.0/DEF * maxSpeed;
		cout << "rand Th,S: " << dTheta << ", " << dSpeed << endl;

		nextNode.speed = dSpeed;
		nextNode.deltaAng = dTheta;


		nextNode = drive(nextNode, img);



		//____________TEST___________________
/*
		newN.self = Point( rand() % mapW , rand() % mapH);
		newN.parent = &nodes[0];

		//distance to the initial node
		double minDist = sqrt(   (newN.self.x-newN.parent->self.x)*(newN.self.x-newN.parent->self.x) + (newN.self.y-newN.parent->self.y)*(newN.self.y-newN.parent->self.y)   );


		//iterates through the other nodes to see if any are closer
		
		for(size_t j = 0; j < currNodeNum; j++)
		{
			NodeLeaf n = nodes[j];
			NodeLeaf* point = &nodes[j];
			
			//distance to next node
			double dist = sqrt(   (newN.self.x-n.self.x)*(newN.self.x-n.self.x) + (newN.self.y-n.self.y)*(newN.self.y-n.self.y)   );
			if(dist < minDist)
			{
				minDist = dist;
				newN.parent = point;	//new parent is closest node
			}
		}
		// cout << "Node:	" << newN.self << endl;
		// cout << "	Node Parent:	" << newN.parent->self << endl;
		// cout << "node dist:	" << minDist << endl;
		



		//if the node is too far away, throw it out
		if(minDist > EPSILON)
		{
			// cout << "threw out" << endl;
			continue;
			//atan2(newN.self.y-newN.parent->y )
		}*/

		// cout << "Created node: " << newN.self; 
		// cout << "	with parent: " << newN.parent->self << endl;
		
		//add new node into the array
		nodes[currNodeNum] = nextNode;
		currNodeNum++;

	}

	// cout << endl << " All Nodes: " << endl;
	// for(size_t i = 0; i < currNodeNum; i++)
	// {
	// 	cout << "	" << nodes[i].self << endl;
	// }


		//final image display
	draw(nodes, img);
	circle(img, start.self, nodeRad, CV_RGB(0,255,0), nodeRad);
	imshow("CaR-RT", img);
	waitKey(0);

	return 0;
}