#ifndef JUNCTGENERATOR_H_
#define JUNCTGENERATOR_H_
#include<stack>
#include<fstream>
#include<iostream>
#include<vector>
#include<string>
#include<list>
#include<map>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<opencv2/ml/ml.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "EditDistance.h"
//#include"JuncDetect.h"
//#include "display.h"


extern float consecuMerg_distanceThrd;
extern double piAngleThrd;
extern double zeroAngleThrd;
extern float  topo_distanceThrd;
extern float  merg_ditanceThrd;
extern float search_distance;
extern float leftRight_ditanceThrd;
using namespace std;
using namespace cv;
struct LSDline
{
    Point2f lineBegin;
    Point2f lineEnd;
    float width;
    float p;
    float log_nfa;
    int tagBegin;
    int tagEnd;
};

struct LsdJunction
{
    LSDline lsdLine_begin;
    LSDline lsdLine_end;
    Ljunct junct;
    int tag;
};

enum RoadSignClassifier
{
    r_NonRoadSign,
    r_Forward,
    r_Left,
    r_Right,
    r_ForwardLeft,
    r_ForwardRight,
    r_ForwardLeftRight
};

struct codeStringBoundingBox
{
    string str;

    string location_str;

    float top;
    float bottom;
    float left;
    float right;

    RoadSignClassifier classifyRst;
};

float calcBranch(const Point2f point, const LSDline& line);

bool checkParamllelsim(const LSDline& l1,const LSDline& l2);

Point2f calcIntersect(const LSDline& l1, const LSDline& l2, Point2f& closePoint, Point2f& farPoint, Point2f& secondClosePoint,Point2f& secondFarPoint, bool isMedianValue);

bool findNextJunction(vector<LsdJunction>& lsdJunction, int search_begin, LSDline&  lsdLine_begin, Ljunct& rstJunct, int* location, LSDline& lsdLine_NextBegin);
float findMinBranch(float branch1, float branch2, float branch3, float branch4);

void DrawBoundingBox(vector<codeStringBoundingBox>& code_string, Mat& img);

bool LSD2Junct(vector<LSDline>& lsdLine, vector<Ljunct>& lJunct, vector<LsdJunction>& lsdJunction, float radius, Mat& imgSize)
{
    for(int i=0; i<lsdLine.size(); i++)
    {
        if(lsdLine[i].tagBegin!=0)
        {
            int x_searchBegin=int(lsdLine[i].lineBegin.x-radius)<0?0:int(lsdLine[i].lineBegin.x-radius);
            int x_searchEnd=int(lsdLine[i].lineBegin.x+radius)>imgSize.cols?imgSize.cols:
                            int(lsdLine[i].lineBegin.x+radius);
            int y_searchBegin=int(lsdLine[i].lineBegin.y-radius)<0?0:int(lsdLine[i].lineBegin.y-radius);
            int y_searchEnd=int(lsdLine[i].lineBegin.y+radius)>imgSize.rows?imgSize.rows:
                            int(lsdLine[i].lineBegin.y+radius);
            vector<int> candidate_begin;
            for (int j=0; j<lsdLine.size(); j++)
            {
                if (i == j)
                {
                    continue;
                }
                if ((lsdLine[j].lineBegin.x>x_searchBegin&&lsdLine[j].lineBegin.x<x_searchEnd&&
                        lsdLine[j].lineBegin.y>y_searchBegin&&lsdLine[j].lineBegin.y<y_searchEnd&&
                        !checkParamllelsim(lsdLine[i],lsdLine[j])&&lsdLine[j].tagBegin!=0 &&
                        abs(lsdLine[j].lineBegin.x-lsdLine[i].lineBegin.x)+abs(lsdLine[j].lineBegin.y-lsdLine[i].lineBegin.y)<
                        abs(lsdLine[j].lineBegin.x-lsdLine[i].lineEnd.x)+abs(lsdLine[j].lineBegin.y-lsdLine[i].lineEnd.y)) ||
                        (lsdLine[j].lineEnd.x>x_searchBegin&&lsdLine[j].lineEnd.x<x_searchEnd&&
                         lsdLine[j].lineEnd.y>y_searchBegin&&lsdLine[j].lineEnd.y<y_searchEnd&&
                         !checkParamllelsim(lsdLine[i],lsdLine[j])&&lsdLine[j].tagEnd!=0 &&
                         abs(lsdLine[j].lineEnd.x-lsdLine[i].lineBegin.x)+abs(lsdLine[j].lineEnd.y-lsdLine[i].lineBegin.y)<
                         abs(lsdLine[j].lineEnd.x-lsdLine[i].lineEnd.x)+abs(lsdLine[j].lineEnd.y-lsdLine[i].lineEnd.y)))
                {
                    candidate_begin.push_back(j);
                }
            }
            if(candidate_begin.size()>0)
            {
                LSDline line_begin=lsdLine[candidate_begin[0]];
                float d_begin = sqrt((line_begin.lineBegin.x - lsdLine[i].lineBegin.x)*(line_begin.lineBegin.x-lsdLine[i].lineBegin.x)+
                                     (line_begin.lineBegin.y-lsdLine[i].lineBegin.y)*(line_begin.lineBegin.y-lsdLine[i].lineBegin.y));
                float d_end = sqrt((line_begin.lineEnd.x - lsdLine[i].lineBegin.x)*(line_begin.lineEnd.x-lsdLine[i].lineBegin.x)+
                                   (line_begin.lineEnd.y-lsdLine[i].lineBegin.y)*(line_begin.lineEnd.y-lsdLine[i].lineBegin.y));
                float d=d_begin<d_end?d_begin:d_end;
                //float d=findMinBranch(d_begin,d_begin_end,d_end,d_end_begin);
                int close_index=candidate_begin[0];
                for (int j=1; j<candidate_begin.size(); j++)
                {
                    d_begin = sqrt((lsdLine[candidate_begin[j]].lineBegin.x - lsdLine[i].lineBegin.x)*(lsdLine[candidate_begin[j]].lineBegin.x-lsdLine[i].lineBegin.x)+
                                   (lsdLine[candidate_begin[j]].lineBegin.y-lsdLine[i].lineBegin.y)*(lsdLine[candidate_begin[j]].lineBegin.y-lsdLine[i].lineBegin.y));
                    d_end = sqrt((lsdLine[candidate_begin[j]].lineEnd.x - lsdLine[i].lineBegin.x)*(lsdLine[candidate_begin[j]].lineEnd.x-lsdLine[i].lineBegin.x)+
                                 (lsdLine[candidate_begin[j]].lineEnd.y-lsdLine[i].lineBegin.y)*(lsdLine[candidate_begin[j]].lineEnd.y-lsdLine[i].lineBegin.y));

                    float d_tmp = d_begin>d_end?d_end:d_begin;
                    if (d>d_tmp)
                    {
                        close_index = candidate_begin[j];
                        d = d_tmp;
                    }
                }



                Ljunct lJunct_tmp;
                Point2f secondClosePoint=abs(lsdLine[i].lineBegin.x-lsdLine[close_index].lineBegin.x)+abs(lsdLine[i].lineBegin.y-lsdLine[close_index].lineBegin.y)<
                                         abs(lsdLine[i].lineBegin.x-lsdLine[close_index].lineEnd.x)+abs(lsdLine[i].lineBegin.y-lsdLine[close_index].lineEnd.y)?
                                         lsdLine[close_index].lineBegin:lsdLine[close_index].lineEnd;
                Point2f secondFarPoint=abs(lsdLine[i].lineBegin.x-lsdLine[close_index].lineBegin.x)+abs(lsdLine[i].lineBegin.y-lsdLine[close_index].lineBegin.y)>
                                       abs(lsdLine[i].lineBegin.x-lsdLine[close_index].lineEnd.x)+abs(lsdLine[i].lineBegin.y-lsdLine[close_index].lineEnd.y)?
                                       lsdLine[close_index].lineBegin:lsdLine[close_index].lineEnd;
                bool isMedianValue=false;
                Point2f location_tmp = calcIntersect(lsdLine[i], lsdLine[close_index], lsdLine[i].lineBegin,lsdLine[i].lineEnd,secondClosePoint,secondFarPoint,isMedianValue);
                //check the whether the location_tmp lies too far from lsdLine[i];
                if(abs(location_tmp.x-lsdLine[i].lineBegin.x) >= 20 ||
                        abs(location_tmp.y-lsdLine[i].lineBegin.y)>=20)
                {
                    location_tmp.x=(lsdLine[i].lineBegin.x+secondClosePoint.x)/2;
                    location_tmp.y=(lsdLine[i].lineBegin.y+secondClosePoint.y)/2;
                }
                int class_id=2;
                float branch1=calcBranch(location_tmp,lsdLine[i]);
                float branch2=calcBranch(location_tmp,lsdLine[close_index]);
                lJunct_tmp.location=location_tmp;
                lJunct_tmp.class_id=class_id;
                lJunct_tmp.branch[0]=branch1;
                lJunct_tmp.branch[1]=branch2;
                lJunct_tmp.strength[0]=0;
                lJunct_tmp.strength[1]=0;
                lJunct_tmp.tag=1;

                //assign value for lsdJunction;
                //if ((abs(location_tmp.x-lsdLine[i].lineBegin.x)<20 && abs(location_tmp.y-lsdLine[i].lineBegin.y)<20) ||
                //	(abs(location_tmp.x-lsdLine[i].lineEnd.x)<20 && abs(location_tmp.y-lsdLine[i].lineEnd.y)<20))
                //{
                LsdJunction lsdJunction_tmp;
                lsdJunction_tmp.lsdLine_begin=lsdLine[i];
                lsdJunction_tmp.lsdLine_end=lsdLine[close_index];
                lsdJunction_tmp.junct=lJunct_tmp;
                lsdJunction_tmp.tag=1;
                if(location_tmp.x>=0 && location_tmp.x <=imgSize.cols && location_tmp.y >=0 && location_tmp.y <= imgSize.rows)
                {
                    lsdJunction.push_back(lsdJunction_tmp);
                }
                //}



                if(!(abs(abs(branch1-branch2)-PI)<0.1) &&
                        abs(branch1-branch2)>0.1)
                {
                    lJunct.push_back(lJunct_tmp);
                }
                if (abs(lsdLine[i].lineBegin.x-lsdLine[close_index].lineBegin.x)+abs(lsdLine[i].lineBegin.y-lsdLine[close_index].lineBegin.y) <
                        abs(lsdLine[i].lineBegin.x-lsdLine[close_index].lineEnd.x)+abs(lsdLine[i].lineBegin.y-lsdLine[close_index].lineEnd.y))
                {
                    lsdLine[close_index].tagBegin=0;
                }
                else
                {
                    lsdLine[close_index].tagEnd=0;
                }

            }
            lsdLine[i].tagBegin=0;
        }

        //processing the endPoint of lsdLine[i];
        if(lsdLine[i].tagEnd!=0)
        {
            int x_searchBegin=int(lsdLine[i].lineEnd.x-radius)<0?0:int(lsdLine[i].lineEnd.x-radius);
            int x_searchEnd=int(lsdLine[i].lineEnd.x+radius)>imgSize.cols?imgSize.cols:
                            int(lsdLine[i].lineEnd.x+radius);
            int y_searchBegin=int(lsdLine[i].lineEnd.y-radius)<0?0:int(lsdLine[i].lineEnd.y-radius);
            int y_searchEnd=int(lsdLine[i].lineEnd.y+radius)>imgSize.rows?imgSize.rows:
                            int(lsdLine[i].lineEnd.y+radius);
            vector<int> candidate_end;
            for (int j=0; j<lsdLine.size(); j++)
            {
                if (i == j)
                {
                    continue;
                }
                if ((lsdLine[j].lineBegin.x>x_searchBegin&&lsdLine[j].lineBegin.x<x_searchEnd&&
                        lsdLine[j].lineBegin.y>y_searchBegin&&lsdLine[j].lineBegin.y<y_searchEnd&&
                        !checkParamllelsim(lsdLine[i],lsdLine[j])&&lsdLine[j].tagBegin!=0 &&
                        abs(lsdLine[j].lineBegin.x-lsdLine[i].lineEnd.x)+abs(lsdLine[j].lineBegin.y-lsdLine[i].lineEnd.y)<
                        abs(lsdLine[j].lineBegin.x-lsdLine[i].lineBegin.x)+abs(lsdLine[j].lineBegin.y-lsdLine[i].lineBegin.y)) ||
                        (lsdLine[j].lineEnd.x>x_searchBegin&&lsdLine[j].lineEnd.x<x_searchEnd&&
                         lsdLine[j].lineEnd.y>y_searchBegin&&lsdLine[j].lineEnd.y<y_searchEnd&&
                         !checkParamllelsim(lsdLine[i],lsdLine[j])&&lsdLine[j].tagEnd!=0 &&
                         abs(lsdLine[j].lineEnd.x-lsdLine[i].lineEnd.x)+abs(lsdLine[j].lineEnd.y-lsdLine[i].lineEnd.y)<
                         abs(lsdLine[j].lineEnd.x-lsdLine[i].lineBegin.x)+abs(lsdLine[j].lineEnd.y-lsdLine[i].lineBegin.y)))
                {
                    candidate_end.push_back(j);
                }
            }
            if (candidate_end.size()>0)
            {
                LSDline line_end=lsdLine[candidate_end[0]];
                float d_begin = sqrt((line_end.lineBegin.x - lsdLine[i].lineEnd.x)*(line_end.lineBegin.x-lsdLine[i].lineEnd.x)+
                                     (line_end.lineBegin.y-lsdLine[i].lineEnd.y)*(line_end.lineBegin.y-lsdLine[i].lineEnd.y));
                float d_end = sqrt((line_end.lineEnd.x - lsdLine[i].lineEnd.x)*(line_end.lineEnd.x-lsdLine[i].lineEnd.x)+
                                   (line_end.lineEnd.y-lsdLine[i].lineEnd.y)*(line_end.lineEnd.y-lsdLine[i].lineEnd.y));
                float d=d_begin<d_end? d_begin:d_end;
                int close_index=candidate_end[0];
                for (int j=1; j<candidate_end.size(); j++)
                {
                    d_begin = sqrt((lsdLine[candidate_end[j]].lineBegin.x - lsdLine[i].lineEnd.x)*(lsdLine[candidate_end[j]].lineBegin.x-lsdLine[i].lineEnd.x)+
                                   (lsdLine[candidate_end[j]].lineBegin.y-lsdLine[i].lineEnd.y)*(lsdLine[candidate_end[j]].lineBegin.y-lsdLine[i].lineEnd.y));
                    d_end = sqrt((lsdLine[candidate_end[j]].lineEnd.x - lsdLine[i].lineEnd.x)*(lsdLine[candidate_end[j]].lineEnd.x-lsdLine[i].lineEnd.x)+
                                 (lsdLine[candidate_end[j]].lineEnd.y-lsdLine[i].lineEnd.y)*(lsdLine[candidate_end[j]].lineEnd.y-lsdLine[i].lineEnd.y));
                    float d_tmp = d_begin<d_end? d_begin:d_end;
                    if (d>d_tmp)
                    {
                        close_index = candidate_end[j];
                        d = d_tmp;
                    }
                }

                Ljunct lJunct_tmp;

                Point2f secondClosePoint=abs(lsdLine[i].lineEnd.x-lsdLine[close_index].lineBegin.x)+abs(lsdLine[i].lineEnd.y-lsdLine[close_index].lineBegin.y)<
                                         abs(lsdLine[i].lineEnd.x-lsdLine[close_index].lineEnd.x)+abs(lsdLine[i].lineEnd.y-lsdLine[close_index].lineEnd.y)?
                                         lsdLine[close_index].lineBegin:lsdLine[close_index].lineEnd;
                Point2f secondFarPoint=abs(lsdLine[i].lineEnd.x-lsdLine[close_index].lineBegin.x)+abs(lsdLine[i].lineEnd.y-lsdLine[close_index].lineBegin.y)>
                                       abs(lsdLine[i].lineEnd.x-lsdLine[close_index].lineEnd.x)+abs(lsdLine[i].lineEnd.y-lsdLine[close_index].lineEnd.y)?
                                       lsdLine[close_index].lineBegin:lsdLine[close_index].lineEnd;
                bool isMedianValue=false;
                Point2f location_tmp = calcIntersect(lsdLine[i], lsdLine[close_index], lsdLine[i].lineEnd,lsdLine[i].lineBegin,secondClosePoint,secondFarPoint,isMedianValue);

                if(abs(location_tmp.x-lsdLine[i].lineEnd.x) >= 20 ||
                        abs(location_tmp.y-lsdLine[i].lineEnd.y)>=20)
                {
                    location_tmp.x=(lsdLine[i].lineEnd.x+secondClosePoint.x)/2;
                    location_tmp.y=(lsdLine[i].lineEnd.y+secondClosePoint.y)/2;
                }
                //Point2f location_tmp = calcIntersect(lsdLine[i], lsdLine[close_index]);
                int class_id=2;
                float branch1=calcBranch(location_tmp,lsdLine[i]);
                float branch2=calcBranch(location_tmp,lsdLine[close_index]);
                lJunct_tmp.location=location_tmp;
                lJunct_tmp.class_id=class_id;
                lJunct_tmp.branch[0]=branch1;
                lJunct_tmp.branch[1]=branch2;
                lJunct_tmp.strength[0]=0;
                lJunct_tmp.strength[1]=0;
                lJunct_tmp.tag=1;

                //assign value for lsdJunction;
                LsdJunction lsdJunction_tmp;
                lsdJunction_tmp.lsdLine_begin=lsdLine[i];
                lsdJunction_tmp.lsdLine_end=lsdLine[close_index];
                lsdJunction_tmp.junct=lJunct_tmp;
                lsdJunction_tmp.tag=1;
                if(location_tmp.x >=0 && location_tmp.x <= imgSize.cols && location_tmp.y >=0 && location_tmp.y <= imgSize.rows)
                {
                    lsdJunction.push_back(lsdJunction_tmp);
                }

                if(!(abs(abs(branch1-branch2)-PI)<0.1) &&
                        abs(branch1-branch2)>0.1)
                {
                    lJunct.push_back(lJunct_tmp);
                }
                if (abs(lsdLine[i].lineEnd.x-lsdLine[close_index].lineBegin.x)+abs(lsdLine[i].lineEnd.y-lsdLine[close_index].lineBegin.y) <
                        abs(lsdLine[i].lineEnd.x-lsdLine[close_index].lineEnd.x)+abs(lsdLine[i].lineEnd.y-lsdLine[close_index].lineEnd.y))
                {
                    lsdLine[close_index].tagBegin=0;
                }
                else
                {
                    lsdLine[close_index].tagEnd=0;
                }
            }
            lsdLine[i].tagEnd=0;
        }

        /*
        for(int j=0;j<lsdLine.size();j++)
        {
        if(i==j)
        continue;
        if(!(lsdLine[j].lineBegin.x==lsdLine[i].lineBegin.x&&lsdLine[j].lineBegin.y==lsdLine[i].lineBegin.y)
        &&lsdLine[j].lineBegin.x>x_searchBegin&&lsdLine[j].lineBegin.x<x_searchEnd&&
        lsdLine[j].lineBegin.y>y_searchBegin&&lsdLine[j].lineBegin.y<y_searchEnd&&
        !checkParamllelsim(lsdLine[i],lsdLine[j])&&
        lsdLine[j].tagBegin!=0)
        {
        Ljunct lJunct_tmp;
        Point2f location_tmp=calcIntersect(lsdLine[i],lsdLine[j]);
        int class_id=2;
        float branch1=calcBranch(location_tmp,lsdLine[i]);
        float branch2=calcBranch(location_tmp,lsdLine[j]);
        lJunct_tmp.location=location_tmp;
        lJunct_tmp.class_id=class_id;
        lJunct_tmp.branch[0]=branch1;
        lJunct_tmp.branch[1]=branch2;
        lJunct_tmp.strength[0]=0;
        lJunct_tmp.strength[1]=0;
        lJunct_tmp.tag=1;

        if(!(abs(abs(branch1-branch2)-PI)<0.01) &&
        abs(branch1-branch2)>0.1)
        {
        lJunct.push_back(lJunct_tmp);
        lsdLine[i].tagBegin=0;
        lsdLine[j].tagBegin=0;

        break;
        }
        }
        if(!(lsdLine[j].lineEnd.x==lsdLine[i].lineBegin.x&&lsdLine[j].lineEnd.y==lsdLine[i].lineBegin.y)
        &&lsdLine[j].lineEnd.x>x_searchBegin&&lsdLine[j].lineEnd.x<x_searchEnd&&
        lsdLine[j].lineEnd.y>y_searchBegin&&lsdLine[j].lineEnd.y<y_searchEnd&&
        !checkParamllelsim(lsdLine[i],lsdLine[j])&&
        lsdLine[j].tagEnd!=0)
        {
        Ljunct lJunct_tmp;
        Point2f location_tmp=calcIntersect(lsdLine[i],lsdLine[j]);
        int class_id=2;
        float branch1=calcBranch(location_tmp,lsdLine[i]);
        float branch2=calcBranch(location_tmp,lsdLine[j]);
        lJunct_tmp.location=location_tmp;
        lJunct_tmp.class_id=class_id;
        lJunct_tmp.branch[0]=branch1;
        lJunct_tmp.branch[1]=branch2;
        lJunct_tmp.strength[0]=0;
        lJunct_tmp.strength[1]=0;
        lJunct_tmp.tag=1;

        if(!(abs(abs(branch1-branch2))<0.01) &&
        abs(branch1-branch2)>0.1)
        {
        lJunct.push_back(lJunct_tmp);
        lsdLine[i].tagBegin=0;
        lsdLine[j].tagEnd=0;

        break;
        }
        }
        }

        }
        if(lsdLine[i].tagEnd!=0)
        {
        int x_searchBegin=int(lsdLine[i].lineEnd.x-radius)<0?0:int(lsdLine[i].lineEnd.x-radius);
        int x_searchEnd=int(lsdLine[i].lineEnd.x+radius)>imgSize.cols?imgSize.cols:
        int(lsdLine[i].lineEnd.x+radius);
        int y_searchBegin=int(lsdLine[i].lineEnd.y-radius)<0?0:int(lsdLine[i].lineEnd.y-radius);
        int y_searchEnd=int(lsdLine[i].lineEnd.y+radius)>imgSize.rows?imgSize.rows:
        int(lsdLine[i].lineEnd.y+radius);
        for(int j=0;j<lsdLine.size();j++)
        {
        if(i==j)
        continue;
        if(!(lsdLine[j].lineBegin.x==lsdLine[i].lineEnd.x&&lsdLine[j].lineBegin.y==lsdLine[i].lineEnd.y)
        &&lsdLine[j].lineBegin.x>x_searchBegin&&lsdLine[j].lineBegin.x<x_searchEnd&&
        lsdLine[j].lineBegin.y>y_searchBegin&&lsdLine[j].lineBegin.y<y_searchEnd&&
        !checkParamllelsim(lsdLine[i],lsdLine[j])&&
        lsdLine[j].tagBegin!=0)
        {
        Ljunct lJunct_tmp;
        Point2f location_tmp=calcIntersect(lsdLine[i],lsdLine[j]);
        int class_id=2;
        float branch1=calcBranch(location_tmp,lsdLine[i]);
        float branch2=calcBranch(location_tmp,lsdLine[j]);
        lJunct_tmp.location=location_tmp;
        lJunct_tmp.class_id=class_id;
        lJunct_tmp.branch[0]=branch1;
        lJunct_tmp.branch[1]=branch2;
        lJunct_tmp.strength[0]=0;
        lJunct_tmp.strength[1]=0;
        lJunct_tmp.tag=1;

        if(!(abs(abs(branch1-branch2)-PI)<0.01) &&
        abs(branch1-branch2)>0.1)
        {
        lJunct.push_back(lJunct_tmp);
        lsdLine[i].tagEnd=0;
        lsdLine[j].tagBegin=0;

        break;
        }
        }
        if(!(lsdLine[j].lineEnd.x==lsdLine[i].lineEnd.x&&lsdLine[j].lineEnd.y==lsdLine[i].lineEnd.y)
        &&lsdLine[j].lineEnd.x>x_searchBegin&&lsdLine[j].lineEnd.x<x_searchEnd&&
        lsdLine[j].lineEnd.y>y_searchBegin&&lsdLine[j].lineEnd.y<y_searchEnd&&
        !checkParamllelsim(lsdLine[i],lsdLine[j])&&
        lsdLine[j].tagEnd!=0)
        {
        Ljunct lJunct_tmp;
        Point2f location_tmp=calcIntersect(lsdLine[i],lsdLine[j]);
        int class_id=2;
        float branch1=calcBranch(location_tmp,lsdLine[i]);
        float branch2=calcBranch(location_tmp,lsdLine[j]);
        lJunct_tmp.location=location_tmp;
        lJunct_tmp.class_id=class_id;
        lJunct_tmp.branch[0]=branch1;
        lJunct_tmp.branch[1]=branch2;
        lJunct_tmp.strength[0]=0;
        lJunct_tmp.strength[1]=0;
        lJunct_tmp.tag=1;

        if(!(abs(abs(branch1-branch2)-PI)<0.01) &&
        abs(branch1-branch2)>0.1)
        {
        lJunct.push_back(lJunct_tmp);
        lsdLine[i].tagEnd=0;
        lsdLine[j].tagEnd=0;

        break;
        }
        }
        }

        }

        }
        */

    }
    if(lJunct.size()>0)
        return true;
    else
        return false;

}

float calcBranch(const Point2f point, const LSDline& line)
{
    //if line lies on x axis;
    if(line.lineBegin.y==line.lineEnd.y)
    {
        if(point.x>=line.lineBegin.x&&point.x>=line.lineEnd.x)
            return PI;
        else
            return 0;
    }
    //if line lies on y axis;
    if(line.lineBegin.x==line.lineEnd.x)
    {
        if(point.y>=line.lineBegin.y&&point.y>=line.lineEnd.y)
            return -PI/2;
        else
            return PI/2;
    }

    else
    {
        double deltaY=line.lineBegin.y-line.lineEnd.y;
        double deltaX=line.lineBegin.x-line.lineEnd.x;
        if(abs(point.x-line.lineBegin.x)<abs(point.x-line.lineEnd.x))//point is more close to lineBegin point;
        {
            return atan2(-deltaY,-deltaX);
        }
        else
            return atan2(deltaY,deltaX);
    }
}

Point2f calcIntersect(const LSDline& l1, const LSDline& l2, Point2f& closePoint, Point2f& farPoint, Point2f& secondClosePoint,Point2f& secondFarPoint, bool isMedianValue)
{
    //cout<<"begin to calculate the intersection point:\n";
    //cout<<"first line:  "<<l1.lineBegin.x<<"  "<<l1.lineBegin.y<<" , "<<l1.lineEnd.x<<"  "<<l1.lineEnd.y<<endl;
    //cout<<"second line:  "<<l2.lineBegin.x<<"  "<<l2.lineBegin.y<<" , "<<l2.lineEnd.x<<"  "<<l2.lineEnd.y<<endl;
    Point2f point;
    //if l1 lies on the x axis and l2 lines on the y axis;
    if(l1.lineBegin.y==l1.lineEnd.y&&l2.lineBegin.x==l2.lineEnd.x)
    {
        point.x=l2.lineEnd.x;
        point.y=l1.lineEnd.y;

        //cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;

        if (abs(point.x-closePoint.x)+abs(point.y-closePoint.y) < abs(point.x-farPoint.x)+abs(point.y-farPoint.y) &&
                abs(point.x-secondClosePoint.x)+abs(point.y-secondClosePoint.y) < abs(point.x-secondFarPoint.x)+abs(point.y-secondFarPoint.y))
        {
            isMedianValue=false;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
        else
        {
            isMedianValue=true;
            point.x=(closePoint.x + secondClosePoint.x)/2.0;
            point.y=(closePoint.y + secondClosePoint.y)/2.0;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
    }

    //if l2 lins on the x axis and l1 lines on the y axis;
    if(l1.lineBegin.x==l1.lineEnd.x&&l2.lineBegin.y==l2.lineEnd.y)
    {
        point.x=l1.lineEnd.x;
        point.y=l2.lineEnd.y;
        // cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;

        if (abs(point.x-closePoint.x)+abs(point.y-closePoint.y) < abs(point.x-farPoint.x)+abs(point.y-farPoint.y) &&
                abs(point.x-secondClosePoint.x)+abs(point.y-secondClosePoint.y) < abs(point.x-secondFarPoint.x)+abs(point.y-secondFarPoint.y))
        {
            isMedianValue=false;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
        else
        {
            isMedianValue=true;
            point.x=(closePoint.x + secondClosePoint.x)/2.0;
            point.y=(closePoint.y + secondClosePoint.y)/2.0;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
    }

    //if l1 lies on the x axis but l2 is skew;
    if(l1.lineBegin.y==l1.lineEnd.y&&l2.lineBegin.x!=l2.lineEnd.x&&l2.lineBegin.y!=l2.lineEnd.y)
    {
        float k2=(l2.lineBegin.y-l2.lineEnd.y)/(l2.lineBegin.x-l2.lineEnd.x);
        point.y=l1.lineEnd.y;
        point.x=(point.y-l2.lineBegin.y)/k2+l2.lineBegin.x;
        // cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;

        if (abs(point.x-closePoint.x)+abs(point.y-closePoint.y) < abs(point.x-farPoint.x)+abs(point.y-farPoint.y) &&
                abs(point.x-secondClosePoint.x)+abs(point.y-secondClosePoint.y) < abs(point.x-secondFarPoint.x)+abs(point.y-secondFarPoint.y))
        {
            isMedianValue=false;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
        else
        {
            isMedianValue=true;
            point.x=(closePoint.x + secondClosePoint.x)/2.0;
            point.y=(closePoint.y + secondClosePoint.y)/2.0;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
    }

    //if l1 lies on the y axis but l2 is skew;
    if(l1.lineBegin.x==l1.lineEnd.x&&l2.lineBegin.x!=l2.lineEnd.x&&l2.lineBegin.y!=l2.lineEnd.y)
    {
        float k2=(l2.lineBegin.y-l2.lineEnd.y)/(l2.lineBegin.x-l2.lineEnd.x);
        point.x=l1.lineEnd.x;
        point.x=(point.x-l2.lineBegin.x)*k2+l2.lineBegin.y;
        //cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
        if (abs(point.x-closePoint.x)+abs(point.y-closePoint.y) < abs(point.x-farPoint.x)+abs(point.y-farPoint.y) &&
                abs(point.x-secondClosePoint.x)+abs(point.y-secondClosePoint.y) < abs(point.x-secondFarPoint.x)+abs(point.y-secondFarPoint.y))
        {
            isMedianValue=false;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
        else
        {
            isMedianValue=true;
            point.x=(closePoint.x + secondClosePoint.x)/2.0;
            point.y=(closePoint.y + secondClosePoint.y)/2.0;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
    }

    //if l2 lies on the x axis but l1 is skew;
    if(l2.lineBegin.y==l2.lineEnd.y&&l1.lineBegin.x!=l1.lineEnd.x&&l1.lineBegin.y!=l1.lineEnd.y)
    {
        float k1=(l1.lineBegin.y-l1.lineEnd.y)/(l1.lineBegin.x-l1.lineEnd.x);
        point.y=l2.lineEnd.y;
        point.x=(point.y-l1.lineBegin.y)/k1+l1.lineBegin.x;

        //cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
        if (abs(point.x-closePoint.x)+abs(point.y-closePoint.y) < abs(point.x-farPoint.x)+abs(point.y-farPoint.y) &&
                abs(point.x-secondClosePoint.x)+abs(point.y-secondClosePoint.y) < abs(point.x-secondFarPoint.x)+abs(point.y-secondFarPoint.y))
        {
            isMedianValue=false;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
        else
        {
            isMedianValue=true;
            point.x=(closePoint.x + secondClosePoint.x)/2.0;
            point.y=(closePoint.y + secondClosePoint.y)/2.0;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
    }

    //if l2 lies on the y axis but l1 is skew;
    if(l2.lineBegin.x==l2.lineEnd.x&&l1.lineBegin.x!=l1.lineEnd.x&&l1.lineBegin.y!=l1.lineEnd.y)
    {
        float k1=(l1.lineBegin.y-l1.lineEnd.y)/(l1.lineBegin.x-l1.lineEnd.x);
        point.x=l2.lineEnd.x;
        point.x=(point.x-l1.lineBegin.x)*k1+l1.lineBegin.y;
        //cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
        if (abs(point.x-closePoint.x)+abs(point.y-closePoint.y) < abs(point.x-farPoint.x)+abs(point.y-farPoint.y) &&
                abs(point.x-secondClosePoint.x)+abs(point.y-secondClosePoint.y) < abs(point.x-secondFarPoint.x)+abs(point.y-secondFarPoint.y))
        {
            isMedianValue=false;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
        else
        {
            isMedianValue=true;
            point.x=(closePoint.x + secondClosePoint.x)/2.0;
            point.y=(closePoint.y + secondClosePoint.y)/2.0;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
    }

    else
    {
        float k1=(l1.lineBegin.y-l1.lineEnd.y)/(l1.lineBegin.x-l1.lineEnd.x);
        float k2=(l2.lineBegin.y-l2.lineEnd.y)/(l2.lineBegin.x-l2.lineEnd.x);
        float b1=l1.lineBegin.y-l1.lineBegin.x*k1;
        float b2=l2.lineBegin.y-l2.lineBegin.x*k2;

        point.x=(b2-b1)/(k1-k2);
        point.y=k1*point.x+b1;



        if (abs(point.x-closePoint.x)+abs(point.y-closePoint.y) < abs(point.x-farPoint.x)+abs(point.y-farPoint.y) &&
                abs(point.x-secondClosePoint.x)+abs(point.y-secondClosePoint.y) < abs(point.x-secondFarPoint.x)+abs(point.y-secondFarPoint.y))
        {
            isMedianValue=false;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
        else
        {
            isMedianValue=true;
            point.x=(closePoint.x + secondClosePoint.x)/2.0;
            point.y=(closePoint.y + secondClosePoint.y)/2.0;
            cout<<"the calculated point is:  "<<point.x<<"  "<<point.y<<endl;
            return point;
        }
    }
}

bool checkParamllelsim(const LSDline& l1,const LSDline& l2)
{
    //both l1 and l2 lie on x axis;
    if(l1.lineBegin.y==l1.lineEnd.y&&l2.lineBegin.y==l2.lineEnd.y)
    {
        return true;
    }
    //both l1 and l2 lie on y axis;
    if(l1.lineBegin.x==l1.lineEnd.x&&l2.lineBegin.x==l2.lineEnd.x)
    {
        return true;
    }
    if(l1.lineBegin.x!=l1.lineEnd.x&&l1.lineBegin.y!=l1.lineEnd.y&&
            l2.lineBegin.x!=l2.lineEnd.x&& l2.lineBegin.y!=l2.lineEnd.y)
    {
        float k1=(l1.lineBegin.y-l1.lineEnd.y)/(l1.lineBegin.x-l1.lineEnd.x);
        float k2=(l2.lineBegin.y-l2.lineEnd.y)/(l2.lineBegin.x-l2.lineEnd.x);
        if(abs(k1-k2)<0.0001)
            return true;
        else
            return false;
    }
    else
        return false;
}


float findMaxBranch(float branch1, float branch2, float branch3, float branch4)
{
    float branch_max;
    branch_max=branch1>branch2? branch1:branch2;
    branch_max=branch_max>branch3?branch_max:branch3;
    branch_max=branch_max>branch4?branch_max:branch4;

    return branch_max;

}

float findMinBranch(float branch1, float branch2, float branch3, float branch4)
{
    float branch_min;
    branch_min=branch1<branch2? branch1:branch2;
    branch_min=branch_min<branch3?branch_min:branch3;
    branch_min=branch_min<branch4?branch_min:branch4;

    return branch_min;
}

vector<codeStringBoundingBox> encodingFromLsdJunction(vector<LsdJunction>& lsdJunction, vector<Ljunct>& Jlist_coding,Mat& img)
{
    vector<codeStringBoundingBox> code_string;
    if (lsdJunction.size()<1)
    {
        cerr<<"cannot calculate the encode string without lsdJunction!\n";
    }
    for (int i=0; i<lsdJunction.size(); i++)
    {
        if (lsdJunction[i].tag == 0)
        {
            continue;
        }
        vector<Ljunct> lJunct_tmp;
        lJunct_tmp.push_back(lsdJunction[i].junct);
        int search_begin=i;
        LSDline lsdLine_begin=lsdJunction[i].lsdLine_begin;
        Ljunct rstJunct;
        int location;
        LSDline lsdLine_NextBegin;
        while (findNextJunction(lsdJunction, search_begin, lsdLine_begin, rstJunct, &location, lsdLine_NextBegin) &&
                rstJunct.location.x != lsdJunction[i].junct.location.x &&
                rstJunct.location.y != lsdJunction[i].junct.location.y)
        {
            lJunct_tmp.push_back(rstJunct);
            search_begin=location;
            location=0;
            lsdLine_begin=lsdLine_NextBegin;

            rstJunct.location.x=0;
            rstJunct.location.y=0;
        }



        lsdJunction[i].tag=0;
        //to ensure the detected Junctions in the lJunct_tmp can form a circle, we constrain the first and the last element in
        //lJunct_tmp lies close;

        //to ensure all the detected L-junction can form a circle we have to search from the lsdLine_end;

        //then we have found the circle Ljunct and now have to transfer them into code_string in anticlockwise order;
        if (lJunct_tmp.size()>=7 && (lsdJunction[i].lsdLine_begin.lineBegin.x == lsdLine_NextBegin.lineBegin.x &&
                                     lsdJunction[i].lsdLine_begin.lineBegin.y == lsdLine_NextBegin.lineBegin.y))
        {
            //after find the connected junction, we need to delete those junctions whose angle are PI and closed junctions;

            vector<Ljunct> lJunct_tmp_afterRemoval;
            for (vector<Ljunct>::iterator t=lJunct_tmp.begin(); t!=lJunct_tmp.end();)
            {
                if (abs(abs(t->branch[0] - t->branch[1])-PI)<piAngleThrd ||
                        abs(t->branch[0] - t->branch[1])<=zeroAngleThrd)
                {
                    t=lJunct_tmp.erase(t);
                }
                else
                    t++;
            }

            //now we have to find those close junction which are not neighboring junctions;
            vector<Ljunct> lJunct_tmp_new=lJunct_tmp;
            vector<Ljunct> lJunct_tmp_merge;
            for (int j=0; j<lJunct_tmp_new.size(); j++)
            {
                bool isInJunct_tmp=false;
                int location_begin=0;
                for (int k=0; k<lJunct_tmp.size(); k++)
                {
                    if (lJunct_tmp_new[j].location.x == lJunct_tmp[k].location.x &&
                            lJunct_tmp_new[j].location.y == lJunct_tmp[k].location.y)
                    {
                        cout<<"have found the location in lJunct_tmp.\n";
                        isInJunct_tmp=true;
                        location_begin=k;
                    }
                }
                if (isInJunct_tmp)
                {
                    cout<<"we have entered here!\n";
                    for (int k=location_begin+2; k<lJunct_tmp.size(); k++)
                    {
                        if (abs(lJunct_tmp[location_begin].location.x-lJunct_tmp[k].location.x)<merg_ditanceThrd&&
                                abs(lJunct_tmp[location_begin].location.y-lJunct_tmp[k].location.y)<merg_ditanceThrd&&
                                !(location_begin==0&&k==lJunct_tmp.size()-1))
                        {
                            cout<<"\n \n  we have to merge two discontinuous L- junctions!!!!!!\n";
                            vector<Ljunct> candidate1;
                            vector<Ljunct> candidate2;
                            for (int k1=location_begin; k1<=k; k1++)
                            {
                                candidate1.push_back(lJunct_tmp[k1]);
                            }
                            for (int k1=0; k1<=location_begin; k1++)
                            {
                                candidate2.push_back(lJunct_tmp[k1]);
                            }
                            for (int k1=k; k1<lJunct_tmp.size(); k1++)
                            {
                                candidate2.push_back(lJunct_tmp[k1]);
                            }
                            float d1_x=0;
                            float d1_y=0;
                            float d2_x=0;
                            float d2_y=0;
                            for (int k1=0; k1<candidate1.size()-1; k1++)
                            {
                                d1_x += abs(candidate1[k1+1].location.x-candidate1[k1].location.x);
                                d1_y += abs(candidate1[k1+1].location.y-candidate1[k1].location.y);
                            }
                            for (int k1=0; k1<candidate2.size()-1; k1++)
                            {
                                d2_x += abs(candidate2[k1+1].location.x-candidate2[k1].location.x);
                                d2_y += abs(candidate2[k1+1].location.y-candidate2[k1].location.y);
                            }
                            d1_x /= (candidate1.size()-1);
                            d2_x /= (candidate2.size()-1);
                            d1_y /= (candidate1.size()-1);
                            d2_y /= (candidate2.size()-1);
                            if(d1_x+d1_y < d2_x+d2_y)
                            {
                                cout<<"now we have entered in d1_x + d2_y < d2_x + d2_y";
                                cout<<"the size of candidate1 is: "<<candidate1.size()<<endl;

                                cout<<"the size of candidate2 is: "<<candidate2.size()<<endl;
                                Ljunct lJunct_tmp1;
                                float location_x=(lJunct_tmp[j].location.x+lJunct_tmp[k].location.x)/2;
                                float location_y=(lJunct_tmp[j].location.y+lJunct_tmp[k].location.y)/2;
                                float branch_min=0;
                                float branch_max=0;
                                branch_min=lJunct_tmp[j].branch[0]>lJunct_tmp[j].branch[1]?lJunct_tmp[j].branch[1]:lJunct_tmp[j].branch[0];
                                branch_min=branch_min<lJunct_tmp[j].branch[0]?branch_min:lJunct_tmp[j].branch[0];
                                branch_min=branch_min<lJunct_tmp[j].branch[1]?branch_min:lJunct_tmp[j].branch[1];
                                lJunct_tmp1.branch[0]=branch_max;
                                lJunct_tmp1.branch[1]=branch_min;
                                lJunct_tmp1.class_id=2;
                                lJunct_tmp1.strength[0]=0;
                                lJunct_tmp1.strength[1]=0;
                                lJunct_tmp1.tag=0;
                                lJunct_tmp1.location.x = location_x;
                                lJunct_tmp1.location.y = location_y;

                                lJunct_tmp_merge.push_back(lJunct_tmp1);

                                Ljunct Ljunct_tag=lJunct_tmp[k];
                                int whereInsert=0;
                                for(int k1=0; k1<lJunct_tmp.size(); k1++)
                                {
                                    if(candidate1[0].location.x == lJunct_tmp[k1].location.x &&
                                            candidate1[0].location.y == lJunct_tmp[k1].location.y)
                                    {
                                        whereInsert = k1;
                                    }
                                }
                                for (int k1=0; k1<candidate1.size(); k1++)
                                {
                                    for (vector<Ljunct>::iterator t=lJunct_tmp.begin(); t!=lJunct_tmp.end();)
                                    {
                                        if (candidate1[k1].location.x == t->location.x &&
                                                candidate1[k1].location.y == t->location.y)
                                        {
                                            t=lJunct_tmp.erase(t);
                                        }
                                        else
                                            t++;
                                    }
                                }
                                int tag=-1;
                                for (vector<Ljunct>::iterator t=lJunct_tmp.begin(); t<lJunct_tmp.end(); t++)
                                {
                                    tag++;
                                    if (tag == whereInsert)
                                    {
                                        lJunct_tmp.insert(t,lJunct_tmp1);
                                        break;
                                    }
                                }
                            }
                            else
                            {
                                cout<<"now we have entered in d1_x + d2_y > d2_x + d2_y";
                                cout<<"the size of candidate1 is: "<<candidate1.size()<<endl;

                                cout<<"the size of candidate2 is: "<<candidate2.size()<<endl;
                                Ljunct lJunct_tmp1;
                                float location_x=(lJunct_tmp[j].location.x+lJunct_tmp[k].location.x)/2;
                                float location_y=(lJunct_tmp[j].location.y+lJunct_tmp[k].location.y)/2;
                                float branch_min=0;
                                float branch_max=0;
                                branch_min=lJunct_tmp[j].branch[0]>lJunct_tmp[j].branch[1]?lJunct_tmp[j].branch[1]:lJunct_tmp[j].branch[0];
                                branch_min=branch_min<lJunct_tmp[j].branch[0]?branch_min:lJunct_tmp[j].branch[0];
                                branch_min=branch_min<lJunct_tmp[j].branch[1]?branch_min:lJunct_tmp[j].branch[1];
                                lJunct_tmp1.branch[0]=branch_max;
                                lJunct_tmp1.branch[1]=branch_min;
                                lJunct_tmp1.class_id=2;
                                lJunct_tmp1.strength[0]=0;
                                lJunct_tmp1.strength[1]=0;
                                lJunct_tmp1.tag=0;
                                lJunct_tmp1.location.x = location_x;
                                lJunct_tmp1.location.y = location_y;

                                lJunct_tmp_merge.push_back(lJunct_tmp1);

                                Ljunct Ljunct_tag=lJunct_tmp[j];
                                int whereInsert=0;
                                for(int k1=0; k1<lJunct_tmp.size(); k1++)
                                {
                                    if(candidate1[0].location.x == lJunct_tmp[k1].location.x &&
                                            candidate1[0].location.y == lJunct_tmp[k1].location.y)
                                    {
                                        whereInsert = k1;
                                    }
                                }
                                for (int k1=0; k1<candidate2.size(); k1++)
                                {
                                    for (vector<Ljunct>::iterator t=lJunct_tmp.begin(); t != lJunct_tmp.end();)
                                    {
                                        if (candidate2[k1].location.x == t->location.x &&
                                                candidate2[k1].location.y == t->location.y)
                                        {
                                            t=lJunct_tmp.erase(t);
                                        }
                                        else
                                            t++;
                                    }
                                }

                                int tag = -1;
                                for (vector<Ljunct>::iterator t=lJunct_tmp.begin(); t<lJunct_tmp.end(); t++)
                                {
                                    tag++;
                                    if (tag == whereInsert)
                                    {
                                        lJunct_tmp.insert(t,lJunct_tmp1);
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
            }

            //now we have merge those neighboring junctions;
            vector<int> whereMerge;

            for (int j=0; j<lJunct_tmp.size(); j++)
            {
                /*
                if((j+1)<lJunct_tmp.size()&& abs(lJunct_tmp[j].location.x-lJunct_tmp[j+1].location.x) < consecuMerg_distanceThrd &&
                        abs(lJunct_tmp[j].location.y-lJunct_tmp[j+1].location.y) < consecuMerg_distanceThrd)
                {

                    cout<<"we have to merge the two neighboring !!!!!\n";
                    Ljunct lJunct_tmp1;
                    float location_x=(lJunct_tmp[j].location.x + lJunct_tmp[j+1].location.x)/2;
                    float location_y=(lJunct_tmp[j].location.y + lJunct_tmp[j+1].location.y)/2;
                    float branch_min=0;
                    float branch_max=0;
                    cout<<"before mergin the two points are: j:\n";
                    cout<<lJunct_tmp[j].location.x<<" "<<lJunct_tmp[j].location.y<<endl;
                    cout<<"the j+1 is:\n";
                    cout<<lJunct_tmp[j+1].location.x<<" "<<lJunct_tmp[j+1].location.y<<endl;
                    cout<<"after merging the result point is:\n";
                    cout<<location_x<<"  "<<location_y<<endl;
                    branch_min=lJunct_tmp[j].branch[0]>lJunct_tmp[j].branch[1]?lJunct_tmp[j].branch[1]:lJunct_tmp[j].branch[0];
                    branch_min=branch_min<lJunct_tmp[j+1].branch[0]?branch_min:lJunct_tmp[j+1].branch[0];
                    branch_min=branch_min<lJunct_tmp[j+1].branch[1]?branch_min:lJunct_tmp[j+1].branch[1];

                    branch_max=lJunct_tmp[j].branch[0]<lJunct_tmp[j].branch[1]?lJunct_tmp[j].branch[1]:lJunct_tmp[j].branch[0];
                    branch_max=branch_max>lJunct_tmp[j+1].branch[0]?branch_max:lJunct_tmp[j+1].branch[0];
                    branch_max=branch_max>lJunct_tmp[j+1].branch[1]?branch_max:lJunct_tmp[j+1].branch[1];
                    lJunct_tmp1.branch[0]=branch_max;
                    lJunct_tmp1.branch[1]=branch_min;
                    lJunct_tmp1.class_id=2;
                    lJunct_tmp1.strength[0]=0;
                    lJunct_tmp1.strength[1]=0;
                    lJunct_tmp1.tag=0;
                    lJunct_tmp1.location.x = location_x;
                    lJunct_tmp1.location.y = location_y;

                    // lJunct_tmp_afterRemoval.push_back(lJunct_tmp1);
                    lJunct_tmp_afterRemoval.push_back(lJunct_tmp1);
                    whereMerge.push_back(lJunct_tmp_afterRemoval.size()-1);

                    j=j+1;
                }
                else
                {
                    lJunct_tmp_afterRemoval.push_back(lJunct_tmp[j]);
                }
                */

                if (j+1<lJunct_tmp.size() && abs(lJunct_tmp[j].location.x-lJunct_tmp[j+1].location.x) < consecuMerg_distanceThrd &&
                        abs(lJunct_tmp[j].location.y-lJunct_tmp[j+1].location.y) <consecuMerg_distanceThrd)
                {
                    int begin=j;
                    int end=j+1;
                    while (end+1 < lJunct_tmp.size() && abs(lJunct_tmp[end+1].location.x-lJunct_tmp[end].location.x) < consecuMerg_distanceThrd &&
                            abs(lJunct_tmp[end].location.y-lJunct_tmp[end+1].location.y) < consecuMerg_distanceThrd)
                    {
                        end++;
                    }
                    Ljunct lJunct_tmp1;
                    float location_x=0;
                    float location_y=0;
                    float branch_min=0;
                    float branch_max=0;
                    for (int k=j; k<end+1; k++)
                    {
                        if (k==j)
                        {
                            branch_max=lJunct_tmp[k].branch[0]>lJunct_tmp[k].branch[1]?lJunct_tmp[k].branch[0]:lJunct_tmp[k].branch[1];
                            branch_min=lJunct_tmp[k].branch[0]>lJunct_tmp[k].branch[1]?lJunct_tmp[k].branch[1]:lJunct_tmp[k].branch[0];
                        }
                        branch_max=lJunct_tmp[k].branch[0]>branch_max?lJunct_tmp[k].branch[0]:branch_max;
                        branch_max=lJunct_tmp[k].branch[1]>branch_max?lJunct_tmp[k].branch[1]:branch_max;

                        branch_min=lJunct_tmp[k].branch[0]<branch_min?lJunct_tmp[k].branch[0]:branch_min;
                        branch_min=lJunct_tmp[k].branch[1]<branch_min?lJunct_tmp[k].branch[1]:branch_min;

                        location_x+=lJunct_tmp[k].location.x;
                        location_y+=lJunct_tmp[k].location.y;
                    }
                    lJunct_tmp1.location.x=location_x/(end+1-begin);
                    lJunct_tmp1.location.y=location_y/(end+1-begin);
                    lJunct_tmp1.branch[0]=branch_max;
                    lJunct_tmp1.branch[1]=branch_min;
                    lJunct_tmp1.class_id=2;
                    lJunct_tmp1.strength[0]=0;
                    lJunct_tmp1.strength[1]=0;
                    lJunct_tmp1.tag=0;

                    lJunct_tmp_afterRemoval.push_back(lJunct_tmp1);
                    whereMerge.push_back(lJunct_tmp_afterRemoval.size()-1);
                    j=end;
                }
                else
                    lJunct_tmp_afterRemoval.push_back(lJunct_tmp[j]);
            }

            /*
            if(lJunct_tmp_merge.size() > 0)
            {
                for(int j=0; j<lJunct_tmp_merge.size(); j++)
                {
                    bool isStillIn=false;
                    int location=0;
                    for(int k=0; k<lJunct_tmp_afterRemoval.size(); k++)
                    {
                        if(lJunct_tmp_merge[j].location.x == lJunct_tmp_afterRemoval[k].location.x &&
                                lJunct_tmp_merge[j].location.y == lJunct_tmp_afterRemoval[k].location.y)
                        {
                            isStillIn=true;
                            location=k;
                            break;
                        }
                    }

                    if(isStillIn)
                    {
                        whereMerge.push_back(location);
                    }
                }
            }
            */

            //check the first and the last element in lJunct_tmp_afterRemoval;
            if (lJunct_tmp_afterRemoval.size()>4)
            {
                if (abs(lJunct_tmp_afterRemoval[0].location.x-lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)<merg_ditanceThrd &&
                        abs(lJunct_tmp_afterRemoval[0].location.y-lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.y)<merg_ditanceThrd)
                {
                    Ljunct lJunct_tmp2;
                    lJunct_tmp2.location.x=(lJunct_tmp_afterRemoval[0].location.x+lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)/2;
                    lJunct_tmp2.location.y=(lJunct_tmp_afterRemoval[0].location.y+lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.y)/2;
                    lJunct_tmp2.branch[0]=findMaxBranch(lJunct_tmp_afterRemoval[0].branch[0],lJunct_tmp_afterRemoval[0].branch[1],
                                                        lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].branch[0],
                                                        lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].branch[1]);
                    lJunct_tmp2.branch[1]=findMinBranch(lJunct_tmp_afterRemoval[0].branch[0],lJunct_tmp_afterRemoval[0].branch[1],
                                                        lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].branch[0],
                                                        lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].branch[1]);
                    lJunct_tmp2.class_id=2;
                    lJunct_tmp2.strength[0]=0;
                    lJunct_tmp2.strength[1]=0;
                    lJunct_tmp2.tag=0;
                    lJunct_tmp_afterRemoval[0]=lJunct_tmp2;
                    whereMerge.push_back(0);
                    vector<Ljunct>::iterator t=lJunct_tmp_afterRemoval.end()-1;
                    lJunct_tmp_afterRemoval.erase(t);
                }
            }

            if (lJunct_tmp_afterRemoval.size()>4)
            { 
                //find the BoundingBox for the lJunct_tmp_afterRemoval;
                float top;
                float bottom;
                float left;
                float right;
                for (int j=0; j<lJunct_tmp_afterRemoval.size(); j++)
                {
                    if (j==0)
                    {
                        top=lJunct_tmp_afterRemoval[j].location.y;
                        bottom=lJunct_tmp_afterRemoval[j].location.y;
                        left=lJunct_tmp_afterRemoval[j].location.x;
                        right=lJunct_tmp_afterRemoval[j].location.x;
                    }
                    else
                    {
                        top=lJunct_tmp_afterRemoval[j].location.y<top?lJunct_tmp_afterRemoval[j].location.y:top;
                        bottom=lJunct_tmp_afterRemoval[j].location.y>bottom?lJunct_tmp_afterRemoval[j].location.y:bottom;
                        left=lJunct_tmp_afterRemoval[j].location.x<left?lJunct_tmp_afterRemoval[j].location.x:left;
                        right=lJunct_tmp_afterRemoval[j].location.x>right?lJunct_tmp_afterRemoval[j].location.x:right;
                    }
                }
                //now calculate the number of pixels in the bounding box;
                int foregroundPixel_num=0;
                for(int r=(int)top; r<= (int)bottom; r++)
                {
                    for(int c=(int)left; c<= (int)right; c++)
                    {
                        if(img.at<uchar>(r,c)>0)
                        {
                            foregroundPixel_num++;
                        }
                    }
                }
                //these connected string whose width and height are not too large or too small are further processed;
                if( !(right-left >= 600 || bottom-top >= 600 || right-left <=30 || bottom-top <= 120 || bottom-top>=7.0*(right-left) || (bottom-top)<=1.4*(right-left) || foregroundPixel_num<7500))
                {
                    cout<<"the size of lJunct_tmp_afterRemoval is:   "<<lJunct_tmp_afterRemoval.size()<<endl;
                    for(int j=0; j<lJunct_tmp_afterRemoval.size(); j++)
                    {
                        Jlist_coding.push_back(lJunct_tmp_afterRemoval[j]);
                    }

                    //find the top junction's location;
                    Ljunct top_Ljunct;
                    int top_location;
                    for (int j=0; j<lJunct_tmp_afterRemoval.size(); j++)
                    {
                        if (j ==0 )
                        {
                            top_Ljunct=lJunct_tmp_afterRemoval[j];
                            top_location=j;
                        }
                        if (lJunct_tmp_afterRemoval[j].location.y < top_Ljunct.location.y)
                        {
                            top_Ljunct=lJunct_tmp_afterRemoval[j];
                            top_location=j;
                        }
                    }

                    bool isAntiClockwise=true;
                    if (top_location<lJunct_tmp_afterRemoval.size()-1)
                    {
                        if (lJunct_tmp_afterRemoval[top_location].location.x > lJunct_tmp_afterRemoval[top_location+1].location.x)
                        {
                            isAntiClockwise=true;
                        }
                        else
                            isAntiClockwise=false;
                    }
                    else
                    {
                        if (lJunct_tmp_afterRemoval[top_location].location.x > lJunct_tmp_afterRemoval[0].location.x)
                        {
                            isAntiClockwise=true;
                        }
                        else
                            isAntiClockwise=false;
                    }

                    //encoding
                    string str;
                    string location_str;
                    int location_str_threshold=leftRight_ditanceThrd;
                    float location_str_check=topo_distanceThrd;
                    float distance=0;
                    if(isAntiClockwise == true)
                    {
                        for(int j=top_location; j<lJunct_tmp_afterRemoval.size(); j++)
                        {
                            if(j == top_location && top_location==0)
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x -
                                        lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x) < location_str_threshold)
                                {
                                    stringstream s;
                                    distance=sqrt((lJunct_tmp_afterRemoval[j].location.x-lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)*
                                                  (lJunct_tmp_afterRemoval[j].location.x-lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x) +
                                                  (lJunct_tmp_afterRemoval[j].location.y-lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.y)*
                                                  (lJunct_tmp_afterRemoval[j].location.y-lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.y));

                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;
                                    location_str += s.str();
                                }
                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)
                                    {
                                        stringstream  s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)
                                    {
                                        stringstream  s;
                                        s<<1;
                                        location_str += s.str();
                                    }
                                }
                            }
                            else
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x -
                                        lJunct_tmp_afterRemoval[j-1].location.x) < location_str_threshold)
                                {
                                    stringstream s;
                                    distance=sqrt((lJunct_tmp_afterRemoval[j].location.x-lJunct_tmp_afterRemoval[j-1].location.x)*
                                                  (lJunct_tmp_afterRemoval[j].location.x-lJunct_tmp_afterRemoval[j-1].location.x) +
                                                  (lJunct_tmp_afterRemoval[j].location.y-lJunct_tmp_afterRemoval[j-1].location.y)*
                                                  (lJunct_tmp_afterRemoval[j].location.y-lJunct_tmp_afterRemoval[j-1].location.y));

                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;
                                    location_str += s.str();
                                }
                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[j-1].location.x)
                                    {
                                        stringstream  s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[j-1].location.x)
                                    {
                                        stringstream  s;
                                        s<<1;
                                        location_str += s.str();
                                    }
                                }

                            }
                        }


                        for(int j=0; j<top_location; j++)
                        {
                            if(j==0)
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x-lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)
                                        <location_str_threshold)
                                {
                                    stringstream s;
                                    distance=sqrt((lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)*
                                                  (lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x) +
                                                  (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.y)*
                                                  (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.y));

                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;
                                    location_str += s.str();
                                }

                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)
                                    {
                                        stringstream s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[lJunct_tmp_afterRemoval.size()-1].location.x)
                                    {
                                        stringstream s;
                                        s<<1;
                                        location_str += s.str();
                                    }

                                }
                            }
                            else
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x -
                                        lJunct_tmp_afterRemoval[j-1].location.x) < location_str_threshold)
                                {
                                    stringstream s;
                                    distance=sqrt((lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[j-1].location.x)*
                                                  (lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[j-1].location.x) +
                                                  (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[j-1].location.y)*
                                                  (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[j-1].location.y));

                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;

                                    location_str += s.str();
                                }
                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[j-1].location.x)
                                    {
                                        stringstream  s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[j-1].location.x)
                                    {
                                        stringstream  s;
                                        s<<1;
                                        location_str += s.str();
                                    }
                                }

                            }
                        }
                    }

                    if(isAntiClockwise == false)
                    {
                        for(int j=top_location; j<lJunct_tmp_afterRemoval.size(); j++)
                        {
                            if(j == lJunct_tmp_afterRemoval.size()-1)
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x -
                                        lJunct_tmp_afterRemoval[0].location.x) < location_str_threshold)
                                {
                                    stringstream s;
                                    distance=sqrt((lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[0].location.x)*
                                                  (lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[0].location.x) +
                                                  (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[0].location.y)*
                                                  (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[0].location.y));

                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;

                                    location_str += s.str();
                                }
                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[0].location.x)
                                    {
                                        stringstream  s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[0].location.x)
                                    {
                                        stringstream  s;
                                        s<<1;
                                        location_str += s.str();
                                    }
                                }
                            }
                            else
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x -
                                        lJunct_tmp_afterRemoval[j+1].location.x) < location_str_threshold)
                                {
                                    stringstream s;
                                    distance = sqrt((lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[j+1].location.x)*
                                                    (lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[j+1].location.x) +
                                                    (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[j+1].location.y)*
                                                    (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[j+1].location.y));

                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;
                                    location_str += s.str();
                                }
                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[j+1].location.x)
                                    {
                                        stringstream  s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[j+1].location.x)
                                    {
                                        stringstream  s;
                                        s<<1;
                                        location_str += s.str();
                                    }
                                }

                            }
                        }

                        for(int j=0; j<top_location; j++)
                        {
                            if(j==top_location-1)
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x-lJunct_tmp_afterRemoval[top_location].location.x)
                                        <location_str_threshold)
                                {
                                    stringstream s;
                                    distance = sqrt((lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[top_location].location.x)*
                                                    (lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[top_location].location.x) +
                                                    (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[top_location].location.y)*
                                                    (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[top_location].location.y));
                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;
                                    location_str += s.str();
                                }

                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[top_location].location.x)
                                    {
                                        stringstream s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[top_location].location.x)
                                    {
                                        stringstream s;
                                        s<<1;
                                        location_str += s.str();
                                    }

                                }
                            }
                            else
                            {
                                if(abs(lJunct_tmp_afterRemoval[j].location.x -
                                        lJunct_tmp_afterRemoval[j+1].location.x) < location_str_threshold)
                                {
                                    stringstream s;
                                    distance = sqrt((lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[j+1].location.x)*
                                                    (lJunct_tmp_afterRemoval[j].location.x - lJunct_tmp_afterRemoval[j+1].location.x) +
                                                    (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[j+1].location.y)*
                                                    (lJunct_tmp_afterRemoval[j].location.y - lJunct_tmp_afterRemoval[j+1].location.y));
                                    if(distance > location_str_check)
                                        s<<2;
                                    else
                                        s<<3;
                                    location_str += s.str();
                                }
                                else
                                {
                                    if(lJunct_tmp_afterRemoval[j].location.x+location_str_threshold < lJunct_tmp_afterRemoval[j+1].location.x)
                                    {
                                        stringstream  s;
                                        s<<0;
                                        location_str += s.str();
                                    }
                                    if(lJunct_tmp_afterRemoval[j].location.x-location_str_threshold > lJunct_tmp_afterRemoval[j+1].location.x)
                                    {
                                        stringstream  s;
                                        s<<1;
                                        location_str += s.str();
                                    }
                                }
                            }
                        }
                    }

                    //encoding for the angle_string;
                    bool isHaveAcuteAngle=false;
                    for (int j=top_location; j<lJunct_tmp_afterRemoval.size(); j++)
                    {
                        vector<int>::iterator result=find(whereMerge.begin(),whereMerge.end(),j);
                        if (result != whereMerge.end()) //if we found the junction, then we directly assign it with 3 value;
                        {
                            stringstream s;
                            s<<3;
                            str=str+s.str();

                            continue;
                        }
                        else
                        {
                            float angle=abs(lJunct_tmp_afterRemoval[j].branch[0]-lJunct_tmp_afterRemoval[j].branch[1])>PI?
                                        2*PI-abs(lJunct_tmp_afterRemoval[j].branch[0]-lJunct_tmp_afterRemoval[j].branch[1]):
                                        abs(lJunct_tmp_afterRemoval[j].branch[0]-lJunct_tmp_afterRemoval[j].branch[1]);
                            if (angle<=(70.0/180.0)*PI)
                            {
                                isHaveAcuteAngle=true;
                                stringstream s;
                                s<<0;
                                str=str+s.str();
                            }
                            if (angle > (70.0/180.0)*PI && angle <= PI*(80.0/180.0))
                            {
                                stringstream s;
                                s<<3;
                                str=str+s.str();
                            }
                            if (angle > PI*(80.0/180.0) && angle <= PI*(95.0/180.0))
                            {
                                stringstream s;
                                s<<1;
                                str=str+s.str();
                            }
                            if( angle > PI*(95.0/180.0) && angle <= PI*(120.0/180.0))
                            {
                                stringstream s;
                                s<<3;
                                str=str+s.str();
                            }
                            if( angle > PI*(120.0/180.0) )
                            {
                                stringstream s;
                                s<<2;
                                str=str+s.str();
                            }
                        }
                    }

                    for (int j=0; j<top_location; j++)
                    {
                        vector<int>::iterator result=find(whereMerge.begin(),whereMerge.end(),j);
                        if (result != whereMerge.end()) //if we found the junction, then we directly assign it with 3 value;
                        {
                            stringstream s;
                            s<<3;
                            str=str+s.str();

                            continue;
                        }
                        else
                        {
                            float angle=abs(lJunct_tmp_afterRemoval[j].branch[0]-lJunct_tmp_afterRemoval[j].branch[1])>PI?
                                        2*PI-abs(lJunct_tmp_afterRemoval[j].branch[0]-lJunct_tmp_afterRemoval[j].branch[1]):
                                        abs(lJunct_tmp_afterRemoval[j].branch[0]-lJunct_tmp_afterRemoval[j].branch[1]);

                            if (angle<=(70.0/180.0)*PI)
                            {
                                isHaveAcuteAngle=true;
                                stringstream s;
                                s<<0;
                                str=str+s.str();
                            }
                            if (angle > (70.0/180.0)*PI && angle <= PI*(80.0/180.0))
                            {
                                stringstream s;
                                s<<3;
                                str=str+s.str();
                            }
                            if (angle > PI*(80.0/180.0) && angle <= PI*(95.0/180.0))
                            {
                                stringstream s;
                                s<<1;
                                str=str+s.str();
                            }
                            if( angle > PI*(95.0/180.0) && angle <= PI*(120.0/180.0))
                            {
                                stringstream s;
                                s<<3;
                                str=str+s.str();
                            }
                            if( angle > PI*(120.0/180.0) )
                            {
                                stringstream s;
                                s<<2;
                                str=str+s.str();
                            }

                        }
                    }

                    //the size of the final string must be bigger than 5;
                    if ((str.size()<7&&isHaveAcuteAngle==true)||str.size()>=7)
                    {
                        codeStringBoundingBox code_string_tmp;
                        if (isAntiClockwise)
                        {
                            code_string_tmp.str=str;
                            code_string_tmp.location_str=location_str;
                        }
                        else
                        {
                            string str_angle;
                            string str_location;
                            str_angle.push_back(str[0]);
                            str_location.push_back(location_str[0]);

                            for (int k=0; k<str.size()-1; k++)
                            {
                                str_angle.push_back(str[str.size()-k-1]);
                            }
                            for(int k=0; k<location_str.size()-1; k++)
                            {
                                str_location.push_back(location_str[location_str.size()-k-1]);
                            }
                            code_string_tmp.str=str_angle;
                            code_string_tmp.location_str=str_location;
                        }
                        code_string_tmp.top=top;
                        code_string_tmp.bottom=bottom;
                        code_string_tmp.left=left;
                        code_string_tmp.right=right;
                        code_string_tmp.classifyRst=r_NonRoadSign;

                        code_string.push_back(code_string_tmp);
                    }
                }
            }
        }
    }
    return code_string;
}


bool findNextJunction(vector<LsdJunction>& lsdJunction, int search_begin, LSDline&  lsdLine_begin, Ljunct& rstJunct, int* location, LSDline& lsdLine_NextBegin)
{
    for (int i=0; i<lsdJunction.size(); i++)
    {
        if (i==search_begin || lsdJunction[i].tag==0)
        {
            continue;
        }
        if (lsdLine_begin.lineBegin.x == lsdJunction[i].lsdLine_begin.lineBegin.x && lsdLine_begin.lineBegin.y == lsdJunction[i].lsdLine_begin.lineBegin.y &&
                lsdLine_begin.lineEnd.x == lsdJunction[i].lsdLine_begin.lineEnd.x && lsdLine_begin.lineEnd.y == lsdJunction[i].lsdLine_begin.lineEnd.y)
        {
            *location=i;
            rstJunct = lsdJunction[i].junct;
            lsdLine_NextBegin=lsdJunction[i].lsdLine_end;

            lsdJunction[i].tag=0;

            break;
        }

        if (lsdLine_begin.lineBegin.x == lsdJunction[i].lsdLine_end.lineBegin.x && lsdLine_begin.lineBegin.y == lsdJunction[i].lsdLine_end.lineBegin.y &&
                lsdLine_begin.lineEnd.x == lsdJunction[i].lsdLine_end.lineEnd.x && lsdLine_begin.lineEnd.y == lsdJunction[i].lsdLine_end.lineEnd.y)
        {
            *location=i;
            rstJunct = lsdJunction[i].junct;
            lsdLine_NextBegin=lsdJunction[i].lsdLine_begin;

            lsdJunction[i].tag=0;

            break;
        }
    }

    if (rstJunct.location.x !=0 && rstJunct.location.y !=0)
    {
        return true;
    }
    else
        return false;
}


void classifyRoadMarking(vector<codeStringBoundingBox>& code_string, const Mat& srcImg)
{
    //the Ground Truth code_string for various RoadMarking;
    string str_Forward="0311113";
    string str_Left="020021122";
    string str_Right="022112002";
    string str_ForwardLeft="03102020021113";
    string str_ForwardRight="03111200202013";
    string str_ForwardLeftRight="020021120020222";


    string loc_Forward="0012121";
    string loc_Left="201212120";
    string loc_Right="020212121";
    string loc_ForwardLeft="00120201212121";
    string loc_ForwardRight="00121212102021";
    string loc_ForwardLeftRight="001202012121212102021";

    //the intergral coding;
    string integral_Forward = "0364648";
    string integral_Left = "425474642";
    string integral_Right = "042464547";
    string integral_FLeft = "03642425474648";
    string integral_FRight = "03646454704048";

    for (int i=0; i<code_string.size(); i++)
    {

        // we first find the number of connect component in the bounding box;
        // if the number>1 then we should delete it;

        Mat binImg=srcImg(Range(code_string[i].top,code_string[i].bottom+1),Range(code_string[i].left,code_string[i].right+1));
        Mat tagImg(binImg.size(),CV_8UC1,Scalar::all(0));

        int connectNum=0;
        for(int r=1; r<binImg.rows-1; r++)
        {
            unsigned char * data=binImg.ptr<unsigned char>(r);
            for(int j=1; j<binImg.cols-1; j++)
            {
                stack< pair<int,int> > findBlob;
                if(binImg.at<uchar>(r,j) >0 && tagImg.at<uchar>(r,j) != 1)
                {
                    stack< pair<int, int> > neighborPixels;
                    neighborPixels.push(pair<int,int>(r,j));
                    tagImg.at<uchar>(r,j)=1;
                    findBlob.push(pair<int,int> (r,j));
                    while(!neighborPixels.empty())
                    {
                        //cout<<"finding the blob......\n";
                        pair<int,int> curPixel=neighborPixels.top();
                        int curX=curPixel.first;
                        int curY=curPixel.second;
                        tagImg.at<uchar>(curX,curY)=1;
                        findBlob.push(pair<int,int>(curX,curY));
                        neighborPixels.pop();


                        //pop the top pixel;
                        if(curY-1>=0&&binImg.at<uchar>(curX,curY-1)>0&&tagImg.at<uchar>(curX,curY-1)!=1)
                        {
                            neighborPixels.push(pair<int,int>(curX,curY-1));
                        }

                        if(curY+1<binImg.cols&&binImg.at<uchar>(curX,curY+1)>0&&tagImg.at<uchar>(curX,curY+1)!=1)
                        {
                            neighborPixels.push(pair<int,int>(curX,curY+1));
                        }
                        if(curX-1>=0&&binImg.at<uchar>(curX-1,curY)>0&&tagImg.at<uchar>(curX-1,curY)!=1)
                        {
                            neighborPixels.push(pair<int,int>(curX-1,curY));
                        }
                        if(curX+1<binImg.rows&&binImg.at<uchar>(curX+1,curY)>0&&tagImg.at<uchar>(curX+1,curY)!=1)
                        {
                            neighborPixels.push(pair<int,int>(curX+1,curY));
                        }
                    }
                }
                if(findBlob.size() > 150)
                {
                    connectNum++;
                }
            }
        }
        if(connectNum >=2)
            continue;
        //we first if there is overlap, if so we treate it as false code_string;
        /*
        bool isOverlap=false;
        Point topLeft_can=Point(code_string[i].top,code_string[i].left);
        Point topRight_can=Point(code_string[i].top,code_string[i].right);
        Point bottomLeft_can=Point(code_string[i].bottom,code_string[i].left);
        Point bottomRight_can=Point(code_string[i].bottom,code_string[i].right);

        for(int j=0; j<code_string.size(); j++)
        {
            if(i==j)
                continue;

            else
            {
                Point topLeft_test=Point(code_string[j].top,code_string[j].left);
                Point topRight_test=Point(code_string[j].top,code_string[j].right);
                Point bottomLeft_test=Point(code_string[j].bottom,code_string[j].left);
                Point bottomRight_test=Point(code_string[j].bottom,code_string[j].right);

                if((topLeft_can.x >= code_string[j].left && topLeft_can.x<=code_string[j].right &&
                        topLeft_can.y >= code_string[j].top && topLeft_can.y <= code_string[j].bottom)||
                        (topRight_can.x >= code_string[j].left && topRight_can.x<=code_string[j].right &&
                         topRight_can.y >= code_string[j].top && topRight_can.y <= code_string[j].bottom) ||
                        (bottomLeft_can.x >= code_string[j].left && bottomLeft_can.x<=code_string[j].right &&
                         bottomLeft_can.y >= code_string[j].top && bottomLeft_can.y <= code_string[j].bottom)||
                        (bottomRight_can.x >= code_string[j].left && bottomRight_can.x<=code_string[j].right &&
                         bottomRight_can.y >= code_string[j].top && bottomRight_can.y <= code_string[j].bottom) ||
                        (topLeft_test.x >= code_string[i].left && topLeft_test.x<=code_string[i].right &&
                         topLeft_test.y >= code_string[i].top && topLeft_test.y <= code_string[i].bottom)||
                        (topRight_test.x >= code_string[i].left && topRight_test.x<=code_string[i].right &&
                         topRight_test.y >= code_string[i].top && topRight_test.y <= code_string[i].bottom) ||
                        (bottomLeft_test.x >= code_string[i].left && bottomLeft_test.x<=code_string[i].right &&
                         bottomLeft_test.y >= code_string[i].top && bottomLeft_test.y <= code_string[i].bottom)||
                        (bottomRight_test.x >= code_string[i].left && bottomRight_test.x<=code_string[i].right &&
                         bottomRight_test.y >= code_string[i].top && bottomRight_test.y <= code_string[i].bottom))
                {
                    isOverlap=true;
                    break;
                }
            }
        }
        if(isOverlap==true)
            continue;
            */
        int num_2=0;
        for(int j=0; j<code_string[i].location_str.size(); j++)
        {
            if(code_string[i].location_str[j] == '3')
            {
                num_2++;
            }
        }
        if(//code_string[i].location_str.size() <= 6 && num_2 >=3 ||
            code_string[i].location_str.size() <=7 && num_2 >=4)
        {
            continue;
        }


        /////////the integral version///////////
        int k1=code_string[i].location_str.size();
        string s=changet(code_string[i].str,code_string[i].location_str,k1);
        cout<<"the returned getChan is: \n"<<s<<endl;;
        if(s == "false")
            continue;

        cout<<"in the code_string "<<i<<endl;
        cout<<"the total_string is: "<<s<<endl;
        cout<<"the angle_string is: "<<code_string[i].str<<endl;
        cout<<"the location_string is: "<<code_string[i].location_str<<endl;
        vector<double> distanceRst;
        distanceRst.push_back(edittotal(s,integral_Forward));
        distanceRst.push_back(edittotal(s,integral_Left));
        distanceRst.push_back(edittotal(s,integral_Right));
        distanceRst.push_back(edittotal(s,integral_FLeft));
        distanceRst.push_back(edittotal(s,integral_FRight));

        cout<<"now, print out the total string distance:\n";
        cout<<"Forward: "<<distanceRst[0]<<endl;
        cout<<"Left: "<<distanceRst[1]<<endl;
        cout<<"Right: "<<distanceRst[2]<<endl;
        cout<<"FLeft: "<<distanceRst[3]<<endl;
        cout<<"FRight: "<<distanceRst[4]<<endl;
        vector<double> distanceRst_angle;
        distanceRst_angle.push_back(editAngle(code_string[i].str,str_Forward));
        distanceRst_angle.push_back(editAngle(code_string[i].str,str_Left));
        distanceRst_angle.push_back(editAngle(code_string[i].str,str_Right));
        distanceRst_angle.push_back(editAngle(code_string[i].str,str_ForwardLeft));
        distanceRst_angle.push_back(editAngle(code_string[i].str,str_ForwardRight));
        distanceRst_angle.push_back(editAngle(code_string[i].str,str_ForwardLeftRight));
        cout<<"now, print out the angle string distance:\n";
        cout<<"Forward: "<<distanceRst_angle[0]<<endl;
        cout<<"Left: "<<distanceRst_angle[1]<<endl;
        cout<<"Right: "<<distanceRst_angle[2]<<endl;
        cout<<"FLeft: "<<distanceRst_angle[3]<<endl;
        cout<<"FRight: "<<distanceRst_angle[4]<<endl;
        cout<<"FLRight: "<<distanceRst_angle[5]<<endl;

        cout<<"the angle_encoding code is:"<<endl;

        cout<<distanceRst_angle[0]<<endl;
        cout<<distanceRst_angle[1]<<endl;
        cout<<distanceRst_angle[2]<<endl;
        cout<<distanceRst_angle[3]<<endl;
        vector<double> distance_location;
        string s1=changel(code_string[i].location_str,code_string[i].location_str.size());
        distance_location.push_back(editLocation(s1,loc_Forward));
        distance_location.push_back(editLocation(s1,loc_Left));
        distance_location.push_back(editLocation(s1,loc_Right));
        distance_location.push_back(editLocation(s1,loc_ForwardLeft));
        distance_location.push_back(editLocation(s1,loc_ForwardRight));
        distance_location.push_back(editLocation(s1,loc_ForwardLeftRight));
        cout<<"now, print out the location string distance:\n";
        cout<<"Forward: "<<distance_location[0]<<endl;
        cout<<"Left: "<<distance_location[1]<<endl;
        cout<<"Right: "<<distance_location[2]<<endl;
        cout<<"FLeft: "<<distance_location[3]<<endl;
        cout<<"FRight: "<<distance_location[4]<<endl;
        cout<<"FLRight: "<<distance_location[5]<<endl;

        cout<<"\n \n the "<< i<<" angle_string is:\n ";
        cout<<code_string[i].str<<endl;

        cout<<"\n \n the location string is:\n  "<<code_string[i].location_str<<endl;

        for(int j=0; j<4; j++)
        {
            cout<<distance_location[j]<<endl;
        }

        //double min_distance=*min_element(distanceRst.begin(),distanceRst.end());
        double min_distance=*min_element(distanceRst.begin(),distanceRst.end());
        if (min_distance<=8)
        {
            int location=-1;
            for (int j=0; j<distanceRst.size(); j++)
            {
                if (distanceRst[j]==min_distance)
                {
                    if(j<3 && min_distance >= 7)
                        continue;
                    location=j;
                    break;
                }
            }

            if (location != -1)
            {
                if (location == 0 && distance_location[location] <= 7.0 && (min_distance+distance_location[location]) < 13)
                {
                    //  if(code_string[i].right-code_string[i].left >= 30 && code_string[i].right-code_string[i].left <= 180 &&
                    //          code_string[i].bottom-code_string[i].top >= 105 && code_string[i].bottom-code_string[i].top <= 370)
                    code_string[i].classifyRst=r_Forward;
                }
                if (location == 1 && distance_location[location] <= 6.0 && (min_distance + distance_location[location]) < 13)
                {
                    // if(code_string[i].right-code_string[i].left >= 70 && code_string[i].right-code_string[i].left <= 160 &&
                    //         code_string[i].bottom-code_string[i].top >= 140 && code_string[i].bottom-code_string[i].top <= 400)
                    code_string[i].classifyRst=r_Left;
                }
                if (location == 2 &&  distance_location[location] <= 6.0 && (min_distance + distance_location[location]) < 13)
                {
                    code_string[i].classifyRst=r_Right;
                }
                if (location == 3 && distance_location[location] <= 9.0)
                {
                    code_string[i].classifyRst=r_ForwardLeft;
                }
                if (location == 4 && distance_location[location] <= 9.0)
                {
                    code_string[i].classifyRst=r_ForwardRight;
                }
                if (location == 5 && distance_location[location] <= 9.0)
                {
                    code_string[i].classifyRst=r_ForwardLeftRight;
                }
            }

        }
    }

}

void DrawBoundingBox(vector<codeStringBoundingBox>& code_string, Mat& img, string& str)
{
    string filestring="./rstFile/"+str.substr(0,str.size()-4)+".jpg.txt";
    ofstream file_out(filestring.c_str());
    if(!file_out.is_open())
    {
        cout<<"cannot open the txt file!\n";
    }
    string imageName=str.substr(0,str.size()-4)+".jpg";
    file_out<<imageName<<"\t"<<img.cols<<"\t"<<img.rows<<endl;
    for (int i=0; i<code_string.size(); i++)
    {
        if (code_string[i].classifyRst == r_NonRoadSign)
        {
            continue;
        }
        else
        {
            Point topLeft=Point(code_string[i].left,code_string[i].top);
            Point topRight=Point(code_string[i].right,code_string[i].top);
            Point bottomLeft=Point(code_string[i].left,code_string[i].bottom);
            Point bottomRight=Point(code_string[i].right,code_string[i].bottom);

            if (code_string[i].classifyRst == r_Forward)
            {
                line(img,topLeft,topRight,Scalar(255,0,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,0,0),2,8);
                putText(img,"Forward",topLeft,2,1.0,Scalar(255,0,0));

                file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<2<<endl;
            }

            if (code_string[i].classifyRst == r_Left)
            {
                line(img,topLeft,topRight,Scalar(200,200,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(200,200,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(200,200,0),2,8);
                line(img,topRight,bottomRight,Scalar(200,200,0),2,8);
                putText(img,"Left",topLeft,2,1.0,Scalar(200,200,0));
                file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<3<<endl;
            }

            if (code_string[i].classifyRst == r_Right)
            {
                line(img,topLeft,topRight,Scalar(200,200,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(200,200,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(200,200,0),2,8);
                line(img,topRight,bottomRight,Scalar(200,200,0),2,8);
                putText(img,"Right",topLeft,2,1.0,Scalar(200,200,0));
                file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<4<<endl;
            }

            if (code_string[i].classifyRst == r_ForwardLeft)
            {
                line(img,topLeft,topRight,Scalar(255,255,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,255,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,255,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,255,0),2,8);
                putText(img,"ForwardLeft",topLeft,2,1.0,Scalar(255,255,0));
                file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<5<<endl;
            }

            if (code_string[i].classifyRst == r_ForwardRight)
            {
                line(img,topLeft,topRight,Scalar(0,255,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(0,255,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(0,255,0),2,8);
                line(img,topRight,bottomRight,Scalar(0,255,0),2,8);
                putText(img,"ForwardRight",topLeft,2,1.0,Scalar(0,255,0));
                file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<6<<endl;
            }

            if (code_string[i].classifyRst == r_ForwardLeftRight)
            {
                line(img,topLeft,topRight,Scalar(0,0,255),2,8);
                line(img,topLeft,bottomLeft,Scalar(0,0,255),2,8);
                line(img,bottomLeft,bottomRight,Scalar(0,0,255),2,8);
                line(img,topRight,bottomRight,Scalar(0,0,255),2,8);
                putText(img,"ForwardLeftRight",topLeft,2,1.0,Scalar(0,0,255));
                file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<7<<endl;
            }



        }
    }

    file_out.close();
    string dst="./rstBinary/"+str+".jpg";

    imwrite(dst,img);
}
/*
void DrawBoundingBox_Ori(vector<codeStringBoundingBox>& code_string, Mat& img, string& str)
{
     	for (int i=0; i<code_string.size(); i++)
	{
		if (code_string[i].classifyRst == r_NonRoadSign)
		{
			continue;
		}
		else
		{



                        Point topLeft=Point(code_string[i].left,code_string[i].top);
			Point topRight=Point(code_string[i].right,code_string[i].top);
			Point bottomLeft=Point(code_string[i].left,code_string[i].bottom);
			Point bottomRight=Point(code_string[i].right,code_string[i].bottom);

			if (code_string[i].classifyRst == r_Forward)
			{
				line(img,topLeft,topRight,Scalar(255,0,0),2,8);
				line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
				line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
				line(img,topRight,bottomRight,Scalar(255,0,0),2,8);
				putText(img,"Forward",topLeft,2,1.0,Scalar(255,0,0));

			}

			if (code_string[i].classifyRst == r_Left)
			{
				line(img,topLeft,topRight,Scalar(200,200,0),2,8);
				line(img,topLeft,bottomLeft,Scalar(200,200,0),2,8);
				line(img,bottomLeft,bottomRight,Scalar(200,200,0),2,8);
				line(img,topRight,bottomRight,Scalar(200,200,0),2,8);
				putText(img,"Left",topLeft,2,1.0,Scalar(200,200,0));
			}

			if (code_string[i].classifyRst == r_Right)
			{
				line(img,topLeft,topRight,Scalar(200,200,0),2,8);
				line(img,topLeft,bottomLeft,Scalar(200,200,0),2,8);
				line(img,bottomLeft,bottomRight,Scalar(200,200,0),2,8);
				line(img,topRight,bottomRight,Scalar(200,200,0),2,8);
				putText(img,"Right",topLeft,2,1.0,Scalar(200,200,0));
			}

			if (code_string[i].classifyRst == r_ForwardLeft)
			{
				line(img,topLeft,topRight,Scalar(255,255,0),2,8);
				line(img,topLeft,bottomLeft,Scalar(255,255,0),2,8);
				line(img,bottomLeft,bottomRight,Scalar(255,255,0),2,8);
				line(img,topRight,bottomRight,Scalar(255,255,0),2,8);
				putText(img,"ForwardLeft",topLeft,2,1.0,Scalar(255,255,0));
			}

			if (code_string[i].classifyRst == r_ForwardRight)
			{
				line(img,topLeft,topRight,Scalar(0,255,0),2,8);
				line(img,topLeft,bottomLeft,Scalar(0,255,0),2,8);
				line(img,bottomLeft,bottomRight,Scalar(0,255,0),2,8);
				line(img,topRight,bottomRight,Scalar(0,255,0),2,8);
				putText(img,"ForwardRight",topLeft,2,1.0,Scalar(0,255,0));
			}

			if (code_string[i].classifyRst == r_ForwardLeftRight)
			{
				line(img,topLeft,topRight,Scalar(0,0,255),2,8);
				line(img,topLeft,bottomLeft,Scalar(0,0,255),2,8);
				line(img,bottomLeft,bottomRight,Scalar(0,0,255),2,8);
				line(img,topRight,bottomRight,Scalar(0,0,255),2,8);
			    putText(img,"ForwardLeftRight",topLeft,2,1.0,Scalar(0,0,255));
			}



		}
	}
        string dst="./DetectResultOri/"+str;

        imwrite(dst,img);



}
*/

//drawing bounding box in original image, not IPM. all the bounding box are drawn in blue color;
void DrawBoundingBox_Ori(vector<codeStringBoundingBox>& code_string, Mat& img, string& str,Mat& H,string& str_tmp)
{
    /*
    string filestring="./FileResult_Ori/"+str_tmp+".jpg.txt";
    ofstream file_out(filestring.c_str());
    if(!file_out.is_open())
    {
        cout<<"cannot open the txt file!\n";
    }
    string imageName=str+".jpg";
    file_out<<imageName<<"\t"<<img.cols<<"\t"<<img.rows<<endl;
    */
    for (int i=0; i<code_string.size(); i++)
    {
        if (code_string[i].classifyRst == r_NonRoadSign)
        {
            // string dst="./DetectResultOri/"+str;
            // imwrite(dst,img);
            continue;
        }
        else
        {
            Point2f topLeft=Point2f(code_string[i].left,code_string[i].top);
            Point2f topRight=Point2f(code_string[i].right,code_string[i].top);
            Point2f bottomLeft=Point2f(code_string[i].left,code_string[i].bottom);
            Point2f bottomRight=Point2f(code_string[i].right,code_string[i].bottom);
            vector<Point2f> IPM_img;
            IPM_img.push_back(topLeft);
            IPM_img.push_back(topRight);
            IPM_img.push_back(bottomLeft);
            IPM_img.push_back(bottomRight);
            vector<Point2f> Ori_img;
            perspectiveTransform(IPM_img,Ori_img,H);
            topLeft=Ori_img[0];
            topRight=Ori_img[1];
            bottomLeft=Ori_img[2];
            bottomRight=Ori_img[3];
            /*
            line(img,topLeft,topRight,Scalar(255,0,0),2,8);
            line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
            line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
            line(img,topRight,bottomRight,Scalar(255,0,0),2,8);
             */
            if (code_string[i].classifyRst == r_Forward)
            {
                putText(img,"2",topLeft,2,1.0,Scalar(255,0,0));
                line(img,topLeft,topRight,Scalar(255,0,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,0,0),2,8);

              //  file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<2<<endl;

            }

            if (code_string[i].classifyRst == r_Left)
            {
                putText(img,"3",topLeft,2,1.0,Scalar(255,0,0));
                line(img,topLeft,topRight,Scalar(255,0,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,0,0),2,8);

               // file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<3<<endl;
            }

            if (code_string[i].classifyRst == r_Right)
            {

                putText(img,"4",topLeft,2,1.0,Scalar(255,0,0));
                line(img,topLeft,topRight,Scalar(255,0,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,0,0),2,8);

               // file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<4<<endl;
            }

            if (code_string[i].classifyRst == r_ForwardLeft)
            {

                putText(img,"5",topLeft,2,1.0,Scalar(255,0,0));
                line(img,topLeft,topRight,Scalar(255,0,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,0,0),2,8);

                //file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<5<<endl;
            }

            if (code_string[i].classifyRst == r_ForwardRight)
            {
                putText(img,"6",topLeft,2,1.0,Scalar(255,0,0));
                line(img,topLeft,topRight,Scalar(255,0,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,0,0),2,8);

                //file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<6<<endl;
            }

            if (code_string[i].classifyRst == r_ForwardLeftRight)
            {
                putText(img,"7",topLeft,2,1.0,Scalar(255,0,0));
                line(img,topLeft,topRight,Scalar(255,0,0),2,8);
                line(img,topLeft,bottomLeft,Scalar(255,0,0),2,8);
                line(img,bottomLeft,bottomRight,Scalar(255,0,0),2,8);
                line(img,topRight,bottomRight,Scalar(255,0,0),2,8);

                //file_out<<topLeft.x<<"\t"<<topLeft.y<<"\t"<<topRight.x-topLeft.x<<"\t"<<bottomLeft.y-topLeft.y<<"\t"<<7<<endl;
            }
        }
    }
    //system("rm -rf DetectResultOri");
    //system("mkdir DetectResultOri");
    string dst="./DetectResultOri/"+str+".jpg";

    imwrite(dst,img);
}


#endif
