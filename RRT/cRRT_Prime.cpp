#include <iostream>
#include <cstdlib>		//random for node choices
#include <ctime>		//time for the random seed
#include <math.h>
#include <iomanip>
#include <fstream>
#include <vector>

//Opencv2 for displaying stuff cause i like it
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

/* TO RECOMPILE CODE 
g++ -std=c++11 -o  Prime cRRT_Prime.cpp `pkg-config opencv --cflags --libs`

*/



using namespace std;
using namespace cv;

//mathematical constants
const double PI = atan(1)*4;
const double MAX = numeric_limits<double>::max();






//width & height of map
int mapW; 
int mapH;

//initial node
int startX;
int startY;
double startTh;


//final node
int endX;
int endY;
double endTh; 



//basic car restraints
double carLength;
double maxTheta;
double turnRad;




//max number of nodes
int nodeNum;
//rate to check final point
int RATE;


//Drawing Constants
int nodeRad;
int lineThickness;
int lineLen;
int obsThickness;


//Epsilon is a fraction of average of w and h
double EPSILON =  ( (mapW + mapH)/2 ) /7 ;

int currNodeNum = 0;
int nodeTry = 0;



class Line
{
public:
	//creates the line with a beginning and end point
	Line(Point m, Point n)
	{
		if(m.x-n.x == 0)		//ignore vertical lines becaus etheir not functions, I'll try to deal with it later
			a=Point(m.x+1, m.y);
		else
			a=m;

		
		b = n;


	}
	//literally the equation
	double equation(int x)
	{
		if(a.y-b.y == 0)	//no change in slope
		{
			return a.y;
		}
		else
		{
			return (1.0*(a.y-b.y)/(a.x-b.x))*(x-a.x ) + a.y;
		}
	}

	double getSlope() //returns slope
	{
		return (1.0*(a.y-b.y)/(a.x-b.x));
	}
	double getIntersect() // returns intersect
	{		
		return a.y - this->getSlope()*a.x;
	}

	Point getA() const
	{
		return a;
	}
	Point getB() const
	{
		return b;
	}
private:
	Point a;
	Point b;

};

vector<Line> *obstacles;
int numLines;



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
	double mult;
	File>>mult;
	startTh = PI*mult;
	File>>dump;
	File>>endX;
	File>>dump;
	File>>endY;
	File>>dump;
	File>>mult;
	endTh = mult*PI;
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
	File>>obsThickness;
	File>>dump;


	//Rate is how often the algorithm tries to get to the goal point
	File>>dump;

	File>>RATE;

	File>>dump;


	//reading in my lines
	File>>dump;
	File>>dump;
	int x1, y1, x2, y2;
	obstacles = new vector<Line>;
	numLines = 0;
	while(dump=="n")
	{
		File>>x1;
		File>>y1;
		File>>x2;
		File>>y2;

		obstacles->push_back(   Line( Point(x1,y1), Point(x2,y2) )   );
		numLines++;
		File>>dump;
	}

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
	for(int i = 0; i < numLines; i++)
	{
		line(  img, (*obstacles)[i].getA(), (*obstacles)[i].getB(), CV_RGB(0,0,0), obsThickness );

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


//checks if three points are counter clock-wise
bool ccw(Point a, Point b, Point c)
{

	double f = (c.y - a.y) * (b.x - a.x);
	double s = (b.y - a.y) * (c.x - a.x);
	// cout << "	" << (f > s) << endl;
	return (f > s);
}

//checks if two lines intersect using the ccw algotihm
bool intersect(Line one, Line two)
{
	return(    ccw(one.getA(),two.getA(),two.getB()) != ccw(one.getB(), two.getA(),two.getB())    )&&(    ccw(one.getA(),one.getB(),two.getA()) != ccw(one.getA(),one.getB(),two.getB())    );

}
//checks if a certain line collides wih the obstacles
bool collidesLine(Line dude)
{
	// cout << "  " << "on:" << dude.getA() << "--" << dude.getB() << endl;
	bool collides = false;
	for(size_t i = 0; i < numLines; i++)
	{
		if( intersect(dude, (*obstacles)[i]))
		{
			collides = true;
			break;
		}
	}
	return collides;



}

//checks if an arc collides with a single line
//uses quadratic formula
bool intersectArc(Point cent, double rad, double ang1, double ang2, Line line)
{


	Point v = line.getA() - line.getB();
	Point q = cent;
	double r = rad;
	Point p1 = line.getB();

	double a = v.dot(v);
	double b = 2.0 * v.dot(p1 - q);
	double c = p1.dot(p1) + q.dot(q) - 2.0*p1.dot(q) - r*r;

	double discriminant = 1.0*b*b - 4.0*a*c;

	double x1, x2, y1, y2;	//for future refrence

	if(discriminant < 0)	//no intersection
	{
		return false;
	}
	
	else 
	{

		x1 = (-b + sqrt(discriminant))/(2*a);
		//if x isnt on the line segment
		if( (x1 < 0 || x1 > 1) 	)
		{}
		else{
			y1 = line.equation(x1);

			double angOfIntersect = atan2(y1-cent.y, x1-cent.x);

			//intersect angle is bewteen the two angles
			if( (angOfIntersect < ang2 		&& angOfIntersect > ang1 	 ) || 		//regular check
				(angOfIntersect < ang2 		&& angOfIntersect > ang1-2*PI) ||		//in case the angles straddle 0/2PI and angOfIntersect is < pi
				(angOfIntersect < ang2+2*PI	&& angOfIntersect > ang1 	 ) )		//same as above, but angOfIntersect > pi
			{
				return true;
			}
		
		}
		x2 = (-b + sqrt(discriminant))/(2*a);
		//if x isnt on the line segment
		if( (x2 < 0 || x2 > 1 ) )
		{}
		else{
			y2 = line.equation(x2);

			double angOfIntersect = atan2(y2-cent.y, x2-cent.x);

			//intersect angle is bewteen the two angles
			if( (angOfIntersect < ang2 		&& angOfIntersect > ang1 	 ) || 		//regular check
				(angOfIntersect < ang2 		&& angOfIntersect > ang1-2*PI) ||		//in case the angles straddle 0/2PI and angOfIntersect is < pi
				(angOfIntersect < ang2+2*PI	&& angOfIntersect > ang1 	 ) )		//same as above, but angOfIntersect > pi
			{
				return true;
			}
		
		}

	}
	


	return true;



}
//checks if an arc collides with any obstacles
bool collidesArc(Point cent, double rad, double ang1, double ang2)
{
	bool collides = false;
	for(size_t i = 0; i < numLines; i++)
	{
		if( intersectArc(cent, rad, ang1, ang2, (*obstacles)[i]))
		{

			collides = true;
			break;
		}
	}
	return collides;
}

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

	double OxLeft = -turnRad*cos(OriginLeave + PI/2) + Origin.x;
	double OyLeft = -turnRad*sin(OriginLeave + PI/2) + Origin.y;
	double FxLeft = -turnRad*cos(FinalLeave  + PI/2)  +Final.x ;
	double FyLeft = -turnRad*sin(FinalLeave  + PI/2)  +Final.y ;


	Line straightLine(Point(OxLeft, OyLeft), Point(FxLeft, FyLeft));
	if(	collidesLine(straightLine) || 
		collidesArc(Origin, turnRad, OriginLeave     - PI/2,  PoseOrigin.angle 	- PI/2	) ||
		collidesArc(Final , turnRad, PoseFinal.angle - PI/2,  FinalLeave		- PI/2	)

		 )
	{
		result.dist = MAX;
	}
	return result;
	

}

//draws an RSR dubin's curve
void drawRSR(DubinsResult r, NodeLeaf PoseOrigin, NodeLeaf PoseFinal,  Mat img, bool final)
{
	if(!final){

		ellipse(img, Point(r.Origin.x, r.Origin.y), Size(turnRad, turnRad), (PoseOrigin.angle-PI/2) *180/PI, (r.alph) *180/PI, 0, Scalar(255,0,0), lineThickness, 8 );
		ellipse(img, Point(r.Final.x, r.Final.y), Size(turnRad, turnRad), (PoseFinal.angle-PI/2)*180/PI, 0, (r.gamm)*180/PI, Scalar(255,0,0), lineThickness, 8 );
	
	}
	else{
		ellipse(img, Point(r.Origin.x, r.Origin.y), Size(turnRad, turnRad), (PoseOrigin.angle-PI/2) *180/PI, (r.alph) *180/PI, 0, Scalar(255,0,255), lineThickness, 8 );
		ellipse(img, Point(r.Final.x, r.Final.y), Size(turnRad, turnRad), (PoseFinal.angle-PI/2)*180/PI, 0, (r.gamm)*180/PI, Scalar(255,0,255), lineThickness, 8 );
		
	}


	//circle(img, Point(Origin.x,Origin.y), turnRad , CV_RGB(0,0,255), lineThickness);
	//circle(img, Point(Final.x,Final.y),turnRad , CV_RGB(0,0,255), lineThickness);

	double OxLeft = -turnRad*cos(r.OriginLeave + PI/2) + r.Origin.x;
	double OyLeft = -turnRad*sin(r.OriginLeave + PI/2) + r.Origin.y;
	double FxLeft = -turnRad*cos(r.FinalLeave  + PI/2)  +r. Final.x ;
	double FyLeft = -turnRad*sin(r.FinalLeave  + PI/2)  + r.Final.y ;

	// cout << Point(OxLeft, OyLeft) << " to " << Point(FxLeft, FyLeft) << endl;
	if(!final)
	{
		line(  img, Point(OxLeft, OyLeft), Point(FxLeft, FyLeft), CV_RGB(0,0,255), lineThickness );
	}
	else
	{
		line(  img, Point(OxLeft, OyLeft), Point(FxLeft, FyLeft), CV_RGB(255,0,255), lineThickness );

	}

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

	double OxLeft = -turnRad*cos(OriginLeave - PI/2) + Origin.x;
	double OyLeft = -turnRad*sin(OriginLeave - PI/2) + Origin.y;
	double FxLeft = -turnRad*cos(FinalLeave  - PI/2)  +Final.x ;
	double FyLeft = -turnRad*sin(FinalLeave  - PI/2)  +Final.y ;


	Line straightLine(Point(OxLeft, OyLeft), Point(FxLeft, FyLeft));
	if(	collidesLine(straightLine) || 
		collidesArc(Origin, turnRad, PoseOrigin.angle 	- PI/2, OriginLeave 	- PI/2 )  ||
		collidesArc(Final , turnRad, FinalLeave  		- PI/2, PoseFinal.angle - PI/2 )
		 )
	{
		result.dist = MAX;
	}

	return result;
	return result;
	

}

//draws an LSL dubin's curve
void drawLSL(DubinsResult r, NodeLeaf PoseOrigin, NodeLeaf PoseFinal,  Mat img, bool final)
{
	if(!final)
	{
		ellipse(img, Point(r.Origin.x, r.Origin.y), Size(turnRad, turnRad), (PoseOrigin.angle+PI/2) *180/PI,0, (r.alph) *180/PI,  Scalar(255,0,0), lineThickness, 8 );
		ellipse(img, Point(r.Final.x, r.Final.y), Size(turnRad, turnRad), (PoseFinal.angle+PI/2)*180/PI, (r.gamm)*180/PI,0, Scalar(255,0,0), lineThickness, 8 );
	}
	else
	{
		cout << "drawing final path" << endl;

		ellipse(img, Point(r.Origin.x, r.Origin.y), Size(turnRad, turnRad), (PoseOrigin.angle+PI/2) *180/PI,0, (r.alph) *180/PI,  Scalar(255,0,255), lineThickness, 8 );
		ellipse(img, Point(r.Final.x, r.Final.y), Size(turnRad, turnRad), (PoseFinal.angle+PI/2)*180/PI, (r.gamm)*180/PI,0, Scalar(255,0,255), lineThickness, 8 );

	}
	//circle(img, Point(Origin.x,Origin.y), turnRad , CV_RGB(0,0,255), lineThickness);
	//circle(img, Point(Final.x,Final.y),turnRad , CV_RGB(0,0,255), lineThickness);

	double OxLeft = -turnRad*cos(r.OriginLeave - PI/2) + r.Origin.x;
	double OyLeft = -turnRad*sin(r.OriginLeave - PI/2) + r.Origin.y;
	double FxLeft = -turnRad*cos(r.FinalLeave  - PI/2)  +r. Final.x ;
	double FyLeft = -turnRad*sin(r.FinalLeave  - PI/2)  + r.Final.y ;

	// cout << Point(OxLeft, OyLeft) << " to " << Point(FxLeft, FyLeft) << endl;
	if(!final)
	{
		line(  img, Point(OxLeft, OyLeft), Point(FxLeft, FyLeft), CV_RGB(0,0,255), lineThickness );
	}
	else
	{
		line(  img, Point(OxLeft, OyLeft), Point(FxLeft, FyLeft), CV_RGB(255,0,255), lineThickness );
	}

}

//calls a certain draw function based on the id of the dubins result
void drawDubins(DubinsResult r, NodeLeaf PoseOrigin, NodeLeaf PoseFinal, Mat img, bool final)
{
	switch(r.id){
		case 0:
			drawRSR(r, PoseOrigin, PoseFinal, img, final);
			break;

		case 1:
			drawLSL(r,PoseOrigin, PoseFinal, img, final);
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
		if(nodeTry % RATE - 1 == 0)//try every rate-th time
		{			//attempt to connect to ending node
			newNode.self.x = endX;
			newNode.self.y = endY;
			newNode.angle  = endTh;
		}

		DubinsResult final = LSL(nodes[0], newNode);
		DubinsResult next;
		NodeLeaf chosen =nodes[0];
		newNode.parent = &nodes[0];

		 // cout << "Checking:" << newNode.self << endl;

		for(size_t i = 0; i < currNodeNum; i++)
		{

			next = LSL(nodes[i], newNode);//checks an rsr

			if(next.dist + nodes[i].cost < final.dist + chosen.cost)
			{
				final = next;
				chosen = nodes[i];
				newNode.parent = &nodes[i];
			};

			next = RSR(nodes[i], newNode);//checks an lsl

			if(next.dist + nodes[i].cost < final.dist + chosen.cost)
			{
				final = next;
				chosen = nodes[i];
				newNode.parent = &nodes[i];
			};

		}

		
		nodeTry++;
		if(next.dist == MAX){	//if the path is too long or intersects
			 // cout << "Collided" << endl;
			continue;
		}
		// cout << "Good" << endl;
		// newNode.cost = final.dist + chosen.cost;
		// for(size_t i = 0; i < currNodeNum; i++)
		// {

		// 	next = LSL(newNode, nodes[i]);//checks an rsr

		// 	if(next.dist + newNode.cost < )
		// 	{
		// 		final = next;
		// 		chosen = nodes[i];
		// 	};

		// 	next = RSR(nodes[i], newNode);//checks an lsl

		// 	if(next.dist + nodes[i].cost < final.dist + chosen.cost)
		// 	{
		// 		final = next;
		// 		chosen = nodes[i];
		// 	};

		// }
		drawDubins(final, chosen, newNode, img, false);

		
		
		nodes[currNodeNum] = newNode;
		
		currNodeNum++;

	}

	NodeLeaf selectedFinal;
	double finalCost = MAX;
	//iterate through list 
	for(size_t i = 0; i < currNodeNum; i++)
	{
		if(nodes[i].self.x == endX && nodes[i].self.y == endY && nodes[i].angle == endTh)
		{
			if(nodes[i].cost < finalCost )
			{
				selectedFinal = nodes[i];
				finalCost = nodes[i].cost;
			}
		}
	}
	if(finalCost == MAX)
	{
		cout << "Result not found" << endl;
		draw(nodes, img);
		circle(img, start.self, nodeRad+1, CV_RGB(0,0,0), nodeRad);
		arrowedLine(   img, start.self,    Point( start.self.x + lineLen*cos(start.angle) , start.self.y + lineLen*sin(start.angle) )   , CV_RGB(0,0,0), lineThickness+1, 8 );

		circle(img, start.self, nodeRad, CV_RGB(255,0,0), nodeRad);
		arrowedLine(   img, start.self,    Point( start.self.x + lineLen*cos(start.angle) , start.self.y + lineLen*sin(start.angle) )   , CV_RGB(255,0,0), lineThickness, 8 );

	}
	else{
		draw(nodes, img);

		circle(img, start.self, nodeRad+1, CV_RGB(0,0,0), nodeRad);
		arrowedLine(   img, start.self,    Point( start.self.x + lineLen*cos(start.angle) , start.self.y + lineLen*sin(start.angle) )   , CV_RGB(0,0,0), lineThickness+1, 8 );

		circle(img, start.self, nodeRad, CV_RGB(0,255,0), nodeRad);
		arrowedLine(   img, start.self,    Point( start.self.x + lineLen*cos(start.angle) , start.self.y + lineLen*sin(start.angle) )   , CV_RGB(0,255,0), lineThickness, 8 );
	


		circle(img, selectedFinal.self, nodeRad+1, CV_RGB(0,0,0), nodeRad);
		arrowedLine(   img, selectedFinal.self,    Point( selectedFinal.self.x + lineLen*cos(selectedFinal.angle) , selectedFinal.self.y + lineLen*sin(selectedFinal.angle) )   , CV_RGB(0,0,0), lineThickness+1, 8 );

		circle(img, selectedFinal.self, nodeRad, CV_RGB(0,255,0), nodeRad);
		arrowedLine(   img, selectedFinal.self,    Point( selectedFinal.self.x + lineLen*cos(selectedFinal.angle) , selectedFinal.self.y + lineLen*sin(selectedFinal.angle) )   , CV_RGB(0,255,0), lineThickness, 8 );
		


		NodeLeaf current = selectedFinal;
		DubinsResult prev;
		DubinsResult prevR;
		DubinsResult prevL;
		// cout << "parent: " << (*(current.parent)).self << endl;

		//while parent is not the start

		while(  (*(current.parent)).self != current.self)
		{
			cout << "drawing final: " << current.self << ", " << (*(current.parent)).self << endl;
			prevL = LSL(*(current.parent), current);//checks an rsr
			prevR = RSR(*(current.parent), current);//checks an lsl

			if(prevL.dist < prevR.dist)
			{
				prev = prevL;
			}
			else
			{
				prev = prevR;
			}
			drawDubins(prev, *(current.parent), current, img, true);
			cout << "parent: " << (*(current.parent)).self << endl;

			current = *(current.parent);

		}
	}	



		//final image display
	
	imshow("CaR-RT", img);
	waitKey(0);


	delete obstacles;
	return 0;
}
