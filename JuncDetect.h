#ifndef JUNCDETECT_H_
#define JUNCDETECT_H_

#include<vector>
#include<string>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>
#include <sstream>

const double PI=3.141592653589;

using namespace std;
using namespace cv;


struct BoundingBox
{
    float top;
    float bottom;
    float left;
    float right;
};

struct Ljunct
{
  Point2f location;
  int class_id;
  float   branch[2];
  float strength[2];
  int tag;
  
  string to_str()
  {
      string out_string;
      char out_str[128];
      sprintf(out_str, "%f, %f; %d; %f, %f; %f, %f",
              location.x, location.y, class_id, branch[0], branch[1], strength[0], strength[1]);
      out_string.assign(out_str, strlen(out_str));
      return out_string;
  }
};

 struct Rectangle
{
  Ljunct junct1;
  Ljunct junct2;
  Ljunct junct3;
  Ljunct junct4;
  int tag;
};

 struct Triangle
{
  Ljunct junct1;
  Ljunct junct2;
  Ljunct junct3;
  int tag;
};

 struct Crosswalk
{
  vector<Rectangle> rectangle;
};

struct Rhombus
{
    Ljunct acuteJunct1;
    Ljunct acuteJunct2;
    Ljunct obtuseJunct1;
    Ljunct obtuseJunct2;
	int tag;
};

struct QuasiRhombus
{
    Ljunct rightJunct1;
    Ljunct rightJunct2;
    Ljunct obtuseJunct1;
    Ljunct obtuseJunct2;
    Ljunct obtuseJunct3;
    Ljunct acuteJunct;
	int tag;
};

struct Forward
{
  Triangle triangle;
  Rectangle rectangle;
};
 
struct ForwardRight
{  
  Triangle acuteTriangle;
  Triangle obtuseTriangle;
  Rectangle rectangle;
  Rhombus rhombus;
};

struct ForwardLeft
{
    Triangle acuteTriangle;
    Triangle obtuseTriangle;
    Rectangle rectangle;
    Rhombus rhombus;
};

struct ForwardLeftRight
{
    Rectangle rectangle;
    Triangle acuteTriangle;
    Triangle obtuseTriangle1;
    Triangle obtuseTriangle2;
    Rhombus rhombus1;
    Rhombus rhombus2;
};

struct Left
{
    Triangle obtuseTriangle;
    QuasiRhombus quasiRhombus;
};

struct Right
{
    Triangle obtuseTriangle;
    QuasiRhombus quasiRhombus;
};


class JunctDec
{
private:
    /*
  vector<Ljunct> m_Jlist;
  vector<Ljunct> m_Jlist_acute;
  vector<Ljunct> m_Jlist_obtuse;
  vector<Ljunct> m_Jlist_right;
  vector<Rectangle> m_Rec;
  vector<Triangle> m_Tri;
  vector<Rectangle> m_Rhombus;
  */
public:
//  JunctDec();
//  ~JunctDec();
  void classifyJlist(vector<Ljunct>& Jlist, vector<Ljunct>& acuteJunct, vector<Ljunct>& obtuseJunct,
          vector<Ljunct>& rightJunct);

  vector<Rectangle> findRectangle(vector<Ljunct>& rightJlist);

  vector<Rectangle> findQuadrangle(vector<Ljunct>& allJlist);
  vector<Triangle> findAcuteTriangle(vector<Ljunct>& acuteJlist);

  vector<Triangle> findObtuseTriangle(vector<Ljunct>& acuteJlist, vector<Ljunct>& obtuseJlist);

  vector<Rhombus> findRhombus(vector<Ljunct>& acuteJlist, vector<Ljunct>& obtuseJlist);

  vector<QuasiRhombus> findQuasiRhombus(vector<Ljunct>& acuteJlist, vector<Ljunct>& obtuseJlist,vector<Ljunct>& rightJlist);

  vector<Forward> findForward(vector<Triangle>& acuteTriangle, vector<Rectangle>& rectangle, vector<Rhombus>& rhombus);

  vector<ForwardLeft> findForwardLeft(vector<Rectangle>& rectangle, vector<Rhombus>& rhombus,
          vector<Triangle>& acuteTriangle, vector<Triangle>& obtuseTriangle);

  vector<ForwardRight> findForwardRight(vector<Triangle>& acuteTriangle, vector<Rectangle>& rectangle,
          vector<Triangle>& obtuseTriangle, vector<Rhombus>& rhombus);

  vector<ForwardLeftRight> findForwardLeftRight(vector<Rectangle>& rectangle, vector<Rhombus>& rhombus,
          vector<Triangle>& acuteTriangle, vector<Triangle>& obtuseTriangle);

  vector<Left> findLeft(vector<Triangle>& obtuseTriangle, vector<QuasiRhombus>& quasiRhombus) ;
 
  vector<Right> findRight(vector<Triangle>& obtuseTriangle, vector<QuasiRhombus>& quasiRhombus) ;

  void changeDirection(vector<Triangle>& obtuseTriangle, vector<Triangle>& acuteTriangle, vector<Ljunct>& rightJlist, vector<Ljunct>& acuteJlist, vector<Ljunct>& obtuseJlist);

  bool isLiesOnTriangle(Ljunct junction, Triangle& triangle);

  bool isLiesOnLine(Point2f& p1, Point2f& p2, Point2f& p_test);

  bool isTriangleRectangleConnect(Triangle& triangle,Rectangle& rectangle);

  bool isRectangleRhombusConnect(Rectangle& rectangle,Rhombus& rhombus);

  bool isTriangleRhombusConnect(Triangle& triangle, Rhombus& rhombus);

  bool isTriangleQuasiRhombusConnect(Triangle& triangle, QuasiRhombus& quasiRhombus);

  int findBranchTag(Ljunct& junction, Triangle& triangle);

  bool classifyRoadMarking(vector<Ljunct>& Jlist,vector<Forward>& forward, vector<ForwardLeft>& forwardLeft,
          vector<ForwardRight>& forwardRight, vector<ForwardLeftRight>& fowardLeftRight, 
		  vector<Left>& left, vector<Right>& right,vector<BoundingBox>& boundingBox, Mat& img,vector<Triangle>& acuteTriangle,
		  vector<Triangle>& obtuseTriangle,vector<Rectangle>& rectangle,vector<Ljunct>& acuteJlist,vector<Ljunct>& obtuseJlist,
		  vector<Ljunct>& rightJlist,vector<Rectangle>& quaDrangle,vector<Rhombus> rhombus, vector<QuasiRhombus> quasiRhombus);

//  bool triCompensate();
//  bool recCompensate();
  bool findNextJunction(vector<Ljunct>& Jlist, Ljunct& junction,vector<Ljunct>& rstJunction);
  bool findJlist(vector<Ljunct> Jlist,Point2f location, double branch,vector<Ljunct>& resultJlist);
  bool sortLlist(vector<Ljunct>& Jlist, const Point2f location);

  bool findNextJunforOneBranch(vector<Ljunct>& Jlist,Point2f location,double orientation, Ljunct& rstJunct);
  bool checkParallelism(double orientation, Ljunct& junction);

  void findConnectTriRec(Triangle& tri, float* x_min, float* x_max, float* y_max);

  vector<Forward>  findForwardWithoutRhombus(vector<Triangle>& acuteTriangle, vector<Rectangle>& rectangle);

  template <typename T>
      BoundingBox& findBoundingBox(const T& roadSign);

  BoundingBox& findBoundingBoxForwardLeft(const ForwardLeft& forwardLeft);

  BoundingBox& findBoundingBoxForwardRight(const ForwardRight& forwardRight);

  BoundingBox& findBoundingBoxForwardLeftRight(const ForwardLeftRight& forwardLeftRight);

  BoundingBox& findBoundingBoxLeft(const Left& left);

  BoundingBox& findBoundingBoxRight(const Right& right);

  bool isJunctCovered(vector<Ljunct>& Jlist, Ljunct& junction);

  void changeDirectionAfterRecExtraction(vector<Ljunct>& obtuseJlist, vector<Ljunct>& acuteJlist,
	  vector<Rectangle>& rectangle);

	  bool isLiesOnRectangle(Rectangle& rectangle, Ljunct junction);

	  int findBranchTagForRectangle(Ljunct& junction, Rectangle& rectangle);

	  bool findNextJunforOneBranchWithReturn(vector<Ljunct>& Jlist, Point2f location, double orientation, Ljunct& rstJunct, int* branch);
	  vector<QuasiRhombus> findHexagon(vector<Ljunct> allJlist);

	  vector<string> encodingLjunct(vector<Ljunct>& Jlist);
};



#endif
