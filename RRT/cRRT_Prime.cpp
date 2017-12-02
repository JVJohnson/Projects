#include <iostream>
#include <cstdlib>		//random for node choices
#include <ctime>		//time for the random seed
#include <math.h>
#include <iomanip>
#include <fstream>


//Opencv2 for displaying stuff cause i like it
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace std;
using namespace cv;

//mathematical constants
const double PI = atan(1)*4;


//width & height of map
int mapW; 
int mapH;

//initial node
int startX;
int startY;
double startTh;





//basic car restraints
double carLength;
double maxTheta;
double turnRad = carLength/tan(maxTheta);





//max number of nodes
int nodeNum;

//Drawing Constants
int nodeRad;
int lineThickness;
int lineLen;


//Epsilon is a fraction of average of w and h
double EPSILON =  ( (mapW + mapH)/2 ) /7 ;

int currNodeNum = 0;
void readINI(string fileName)
{
	ifstream File(fileName, ios::in);

	if(!File)
	{
		cerr << "File Not Found:" << fileName << endl;
	};


	//dump allows me to dump all the headings of the numbers
	string dump;
	
	//width & height of map
	File>>dump;
	File>>mapW;
	File>>dump;
	File>>mapH;
	File>>dump;

	//Beginning node restraints
	File>>dump;
	File>>startX;
	File>>dump;
	File>>startY;
	File>>dump;
	File>>startTh;
	File>>dump;

	//basic car restraints
	File>>dump;
	File>>carLength;
	File>>dump;
	double turnDenom;
	File>>turnDenom;
	maxTheta = PI/turnDenom;
	turnRad = carLength/tan(maxTheta);
	File>>dump;

	//max number of nodes
	File>>dump;
	File>>nodeNum;
	File>>dump;

	//Drawing Constants
	File>>dump;
	File>>nodeRad;
	File>>dump;
	File>>lineThickness;
	File>>dump;
	File>>lineLen;
	File>>dump;


	//Epsilon is a fraction of average of w and h
	File>>dump;
	double EpsilonDenom;
	File>>EpsilonDenom;
	EPSILON =  ( (mapW + mapH)/2 ) /EpsilonDenom ;
	File>>dump;

	return;
}

//node leaf. contains current state and parent
struct NodeLeaf
	{
		Point self;
		double angle;
		double cost;

		NodeLeaf* parent;
	};

//draws the nodes on an image and connects the parents
void draw( NodeLeaf *nodes, Mat img)
{

	for(int i = 0; i<currNodeNum; i++ ) //iterates through each node and draws the pose direction
	{

		circle(img, nodes[i].self, nodeRad, CV_RGB(0,0,0), nodeRad);
		arrowedLine(   img, nodes[i].self,    Point( nodes[i].self.x + lineLen*cos(nodes[i].angle) , nodes[i].self.y + lineLen*sin(nodes[i].angle) )   , CV_RGB(0,0,0), lineThickness, 8 );
	}


	imshow("CaR-RT", img);
	waitKey(4);
};






//Right, straight, Right Dubins curve, returns length of the path

//struct containing all the information needed to redraw the line
struct DubinsResult
{
	int id = 0;
	double dist;
	Point Origin;
	Point Final;
	double alph;
	double gamm;
	double OriginLeave;
	double FinalLeave;
};

DubinsResult RSR(NodeLeaf PoseOrigin, NodeLeaf PoseFinal)
{
	//creates the center of the turning radius circle on the right side of the origin pose
	Point Origin;
	Origin.x 	= PoseOrigin.self.x + turnRad*cos(PoseOrigin.angle + PI/2);
	Origin.y 	= PoseOrigin.self.y + turnRad*sin(PoseOrigin.angle + PI/2);


	//creates the center of the turning radius circle on the right side of the final pose
	Point Final;
	Final.x 	= PoseFinal.self.x  + turnRad*cos(PoseFinal.angle  + PI/2);
	Final.y 	= PoseFinal.self.y  + turnRad*sin(PoseFinal.angle  + PI/2);

	double deltX = Final.x-Origin.x;
	double deltY = Final.y-Origin.y;

	//Travel angle around the first circle
	double OriginLeave = atan2(deltY, deltX);
	double alph	= OriginLeave - PoseOrigin.angle + 2*PI;
	while(alph > 2*PI) alph -= 2*PI;
	while(alph < 0) alph += 2*PI;

	//straight distance to travel
	double dist = sqrt(   (deltX)*(deltX) + (deltY)*(deltY)   );
	
	//Travel angle around the first circle
	double FinalLeave = atan2(deltY, deltX);
	double gamm	= FinalLeave - PoseFinal.angle -2*PI;
	while (gamm > 2*PI) gamm-= 2*PI;
	while (gamm < -2*PI) gamm+= 2*PI;
	
	// cout << gamm << endl;

	DubinsResult result;
	result.id 			= 0;
	result.dist 		= abs(alph*turnRad)  +  abs(dist)  +  abs(gamm*turnRad);
	result.Origin 		= Origin;
	result.Final 		= Final;
	result.alph 		= alph;
	result.gamm 		= gamm;
	result.OriginLeave 	= OriginLeave;
	result.FinalLeave 	= FinalLeave;

	return result;
	

}

//draws an RSR dubin's curve
void drawRSR(DubinsResult r, NodeLeaf PoseOrigin, NodeLeaf PoseFinal,  Mat img)
{
	ellipse(img, Point(r.Origin.x, r.Origin.y), Size(turnRad, turnRad), (PoseOrigin.angle-PI/2) *180/PI, (r.alph) *180/PI, 0, Scalar(255,0,0), lineThickness, 8 );
	ellipse(img, Point(r.Final.x, r.Final.y), Size(turnRad, turnRad), (PoseFinal.angle-PI/2)*180/PI, 0, (r.gamm)*180/PI, Scalar(255,0,0), lineThickness, 8 );

	//circle(img, Point(Origin.x,Origin.y), turnRad , CV_RGB(0,0,255), lineThickness);
	//circle(img, Point(Final.x,Final.y),turnRad , CV_RGB(0,0,255), lineThickness);

	double OxLeft = -turnRad*cos(r.OriginLeave + PI/2) + r.Origin.x;
	double OyLeft = -turnRad*sin(r.OriginLeave + PI/2) + r.Origin.y;
	double FxLeft = -turnRad*cos(r.FinalLeave  + PI/2)  +r. Final.x ;
	double FyLeft = -turnRad*sin(r.FinalLeave  + PI/2)  + r.Final.y ;

	// cout << Point(OxLeft, OyLeft) << " to " << Point(FxLeft, FyLeft) << endl;
	line(  img, Point(OxLeft, OyLeft), Point(FxLeft, FyLeft), CV_RGB(0,0,255), lineThickness );


}



DubinsResult LSL(NodeLeaf PoseOrigin, NodeLeaf PoseFinal)
{
	//creates the center of the turning radius circle on the right side of the origin pose
	Point Origin;
	Origin.x 	= PoseOrigin.self.x + turnRad*cos(PoseOrigin.angle - PI/2);
	Origin.y 	= PoseOrigin.self.y + turnRad*sin(PoseOrigin.angle - PI/2);


	//creates the center of the turning radius circle on the right side of the final pose
	Point Final;
	Final.x 	= PoseFinal.self.x  + turnRad*cos(PoseFinal.angle  - PI/2);
	Final.y 	= PoseFinal.self.y  + turnRad*sin(PoseFinal.angle  - PI/2);

	double deltX = Final.x-Origin.x;
	double deltY = Final.y-Origin.y;

	//Travel angle around the first circle
	double OriginLeave = atan2(deltY, deltX);
	double alph	= OriginLeave - PoseOrigin.angle - 2*PI;
	while(alph > 0) alph -= 2*PI;
	while(alph < -2*PI) alph += 2*PI;

	//straight distance to travel
	double dist = sqrt(   (deltX)*(deltX) + (deltY)*(deltY)   );
	
	//Travel angle around the first circle
	double FinalLeave = atan2(deltY, deltX);
	double gamm	= FinalLeave - PoseFinal.angle + 2*PI;
	while (gamm > 2*PI) gamm-= 2*PI;
	while (gamm < 0) gamm+= 2*PI;
	
	// cout << gamm << endl;

	DubinsResult result;
	result.id 			= 1;
	result.dist 		= abs(alph*turnRad)  +  abs(dist)  +  abs(gamm*turnRad);
	result.Origin 		= Origin;
	result.Final 		= Final;
	result.alph 		= alph;
	result.gamm 		= gamm;
	result.OriginLeave 	= OriginLeave;
	result.FinalLeave 	= FinalLeave;

	return result;
	

}

//draws an LSL dubin's curve
void drawLSL(DubinsResult r, NodeLeaf PoseOrigin, NodeLeaf PoseFinal,  Mat img)
{
	ellipse(img, Point(r.Origin.x, r.Origin.y), Size(turnRad, turnRad), (PoseOrigin.angle+PI/2) *180/PI,0, (r.alph) *180/PI,  Scalar(255,0,0), lineThickness, 8 );
	ellipse(img, Point(r.Final.x, r.Final.y), Size(turnRad, turnRad), (PoseFinal.angle+PI/2)*180/PI, (r.gamm)*180/PI,0, Scalar(255,0,0), lineThickness, 8 );

	//circle(img, Point(Origin.x,Origin.y), turnRad , CV_RGB(0,0,255), lineThickness);
	//circle(img, Point(Final.x,Final.y),turnRad , CV_RGB(0,0,255), lineThickness);

	double OxLeft = -turnRad*cos(r.OriginLeave - PI/2) + r.Origin.x;
	double OyLeft = -turnRad*sin(r.OriginLeave - PI/2) + r.Origin.y;
	double FxLeft = -turnRad*cos(r.FinalLeave  - PI/2)  +r. Final.x ;
	double FyLeft = -turnRad*sin(r.FinalLeave  - PI/2)  + r.Final.y ;

	// cout << Point(OxLeft, OyLeft) << " to " << Point(FxLeft, FyLeft) << endl;
	line(  img, Point(OxLeft, OyLeft), Point(FxLeft, FyLeft), CV_RGB(0,0,255), lineThickness );


}

//calls a certain draw function based on the id of the dubins result
void drawDubins(DubinsResult r, NodeLeaf PoseOrigin, NodeLeaf PoseFinal, Mat img)
{
	switch(r.id){
		case 0:
			drawRSR(r, PoseOrigin, PoseFinal, img);
			break;

		case 1:
			drawLSL(r,PoseOrigin, PoseFinal, img);
			break;

	}
}


int main()
{

	readINI("cRRT_Prime.ini");
	//matrix of the image space
	Mat img(mapH, mapW, CV_8UC3, Scalar(255,255,255));
	


	//array containing all nodes
	NodeLeaf nodes[nodeNum];

	//setting up beginnning node
	NodeLeaf start;
	start.self = Point(startX, startY);
	start.angle = startTh;
	start.parent = &start;
	start.cost = 0;

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
		//just a test version to see if the dubins curves works without having to implement non-holonmic regression for it
		
		NodeLeaf newNode;
		newNode.self.x = rand() % (mapW - 2*(int)(turnRad*2)) + 2*turnRad;
		newNode.self.y = rand() % (mapH - 2*(int)(turnRad*2)) + 2*turnRad;
		//returns random angle 
		newNode.angle  = fmod(rand(), 2*PI);

		DubinsResult final = LSL(nodes[0], newNode);
		DubinsResult next;
		NodeLeaf chosen =nodes[0];


		for(size_t i = 0; i < currNodeNum; i++)
		{

			next = LSL(nodes[i], newNode);

			if(next.dist /*+ nodes[i].cost*/ < final.dist /*+ chosen.cost*/)
			{
				final = next;
				chosen = nodes[i];
			};

			next = RSR(nodes[i], newNode);

			if(next.dist /*+ nodes[i].cost*/ < final.dist /*+ chosen.cost*/)
			{
				final = next;
				chosen = nodes[i];
			};

		}

		newNode.cost = final.dist + chosen.cost;
		drawDubins(final, chosen, newNode, img);

		// cout << "Length of path: " << len << endl;


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
		nodes[currNodeNum] = newNode;
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