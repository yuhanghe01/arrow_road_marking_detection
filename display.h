/***
 *@author: Yuhang He yuhanghe@whu.edu.cn
 *display.h is used for showing various intermidiate results.
 *it could be modified for intermidate result saving or direct intermidate 
 *result display;
***/ 

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "JuncDetect.h"
#include "JunctGenerator.h"
//this file is used to display detected LSD lines and various L-junctions;
extern string srcImgDir;

void displayLSDline(vector<LSDline>& lsdLine, Mat& img)
{
	if (lsdLine.size()<1)
	{
		return;
	}
	RNG rng(0xFFFFFFFF);
	for (int i=0; i<lsdLine.size();i++)
	{
		int color=(unsigned)rng;
		line(img,Point(lsdLine[i].lineBegin.x,lsdLine[i].lineBegin.y),Point(lsdLine[i].lineEnd.x,lsdLine[i].lineEnd.y),
		     Scalar(color&255,(color>>8)&255,(color>>16)&255),2,8);
	}
	char* window="LSDline";
	namedWindow(window,CV_WINDOW_NORMAL);
	imshow(window,img);
	waitKey(0);
}

void displayLjunct(vector<Ljunct>& L_Junct, Mat img,string& str)
{
	if (L_Junct.size()<1)
	{
		return;
	}
	for (int i=0; i<L_Junct.size(); i++)
	{
		Point branch[2];
		for (int j=0; j<2; j++)
		{
			if (abs(L_Junct[i].branch[j])<0.0001)//lines on +x axis;
			{
				branch[j].x=L_Junct[i].location.x+10>img.cols?img.cols:L_Junct[i].location.x+10;
				branch[j].y=L_Junct[i].location.y;
			}
			if (abs(L_Junct[i].branch[j]-PI/2)<0.0001)//lies on +y axis;
			{
				branch[j].x=L_Junct[i].location.x;
				branch[j].y=L_Junct[i].location.y+10>img.rows?img.rows:L_Junct[i].location.y+10;
			}
			if (abs(abs(L_Junct[i].branch[j])-PI)<0.0001)//lies on -x axis;
			{
				branch[j].x=L_Junct[j].location.x-10<0?0:L_Junct[i].location.x-10;
				branch[j].y=L_Junct[i].location.y;
			}
			if (abs(L_Junct[i].branch[j]+PI/2)<0.0001)//lies on -y axis;
			{
				branch[j].x=L_Junct[i].location.x;
				branch[j].y=L_Junct[i].location.y-10<0?0:L_Junct[i].location.y-10;
			}
			else
			{
				if (L_Junct[i].branch[j]>-PI/2 && L_Junct[i].branch[j]< PI/2)
				{
					branch[j].x=L_Junct[i].location.x+10>img.cols?img.cols:L_Junct[i].location.x+10;
					branch[j].y=L_Junct[i].location.y+10*tan(L_Junct[i].branch[j]);
				}
				if ((L_Junct[i].branch[j]>-PI && L_Junct[i].branch[j]< -PI/2) ||
					(L_Junct[i].branch[j]>PI/2 && L_Junct[i].branch[j]< PI))
				{
					branch[j].x=L_Junct[i].location.x-10<0?0:L_Junct[i].location.x-10;
					branch[j].y=L_Junct[i].location.y+10*tan(L_Junct[i].branch[j]);
				}
			}

		}
		Point branch_tmp[2];
		branch_tmp[0].x=branch[0].y;
		branch_tmp[0].y=branch[0].x;
		branch_tmp[1].x=branch[1].y;
		branch_tmp[1].y=branch[1].x;
		circle(img,Point(L_Junct[i].location.x,L_Junct[i].location.y),5,Scalar(255,255,255),2,8,0);
		//line(img,Point(L_Junct[i].location.x,L_Junct[i].location.y),branch_tmp[0],Scalar(255,255,0),2,8);
		//line(img,Point(L_Junct[i].location.x,L_Junct[i].location.y),branch_tmp[1],Scalar(255,255,0),2,8);
	}
        string dst="./rstJunct/"+str.substr(0,str.size()-4)+".jpg";

        //string dst="./rstJunct/1.jpg";
        imwrite(dst,img); 
	char* window="LSDline";
	namedWindow(window,CV_WINDOW_NORMAL);
	imshow(window,img);
	waitKey(0);
}

void displayTriangle(vector<Triangle> triangle, Mat& img)
{
	if (triangle.size()<1)
	{
		return;
	}
	for (int i=0; i<triangle.size(); i++)
	{
		circle(img,Point(triangle[i].junct1.location.x,triangle[i].junct1.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(triangle[i].junct2.location.x,triangle[i].junct2.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(triangle[i].junct3.location.x,triangle[i].junct3.location.y),5,Scalar(255,255,255),2,8,0);
	}
	char* window="Triangle";
	namedWindow(window,CV_WINDOW_NORMAL);
	imshow(window,img);
	waitKey(0);
}



void displayRectangle(vector<Rectangle> rectangle, Mat& img)
{
	if (rectangle.size()<1)
	{
		return;
	}
	for (int i=0; i<rectangle.size(); i++)
	{
		circle(img,Point(rectangle[i].junct1.location.x,rectangle[i].junct1.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(rectangle[i].junct2.location.x,rectangle[i].junct2.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(rectangle[i].junct3.location.x,rectangle[i].junct3.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(rectangle[i].junct4.location.x,rectangle[i].junct4.location.y),5,Scalar(255,255,255),2,8,0);
	}
	char* window="Rectangle";
	namedWindow(window,CV_WINDOW_NORMAL);
	imshow(window,img);
	waitKey(0);
}

void displayRhombus(vector<Rhombus> rhombus, Mat& img)
{
	if (rhombus.size()<1)
	{
		return;
	}
	for (int i=0; i<rhombus.size(); i++)
	{
		circle(img,Point(rhombus[i].acuteJunct1.location.x,rhombus[i].acuteJunct1.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(rhombus[i].acuteJunct2.location.x,rhombus[i].acuteJunct2.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(rhombus[i].obtuseJunct1.location.x,rhombus[i].obtuseJunct1.location.y),5,Scalar(255,255,255),2,8,0);
		circle(img,Point(rhombus[i].obtuseJunct2.location.x,rhombus[i].obtuseJunct2.location.y),5,Scalar(255,255,255),2,8,0);
	}
	char* window="rhombus";
	namedWindow(window,CV_WINDOW_NORMAL);
	imshow(window,img);
	waitKey(0);
}
#endif
