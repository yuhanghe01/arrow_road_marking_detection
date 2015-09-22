/***
 *@author: Yuhang He yuhanghe@whu.edu.cn
 *this is the main function file
 *
 *
***/


#include<omp.h>
#include <cstdio>
#include <cstdlib>
#include<fstream>
#include"JuncDetect.h"
#include"JunctGenerator.h"
#include "display.h"
#include "EditDistance.h"
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d/calib3d.hpp>

extern "C"
{
#include "lsd.h"
};
//define the LSD line detector's angle tolerance value;
double lsdAngle=60;
//define the 0 angle's threshold of a L- junction
double zeroAngleThrd=PI/15;
//define the PI angle's threshold of a L- junction
double piAngleThrd=PI/10;
//define the distance threshold for Topology Coding
float topo_distanceThrd=30.0;
//define the distance threshold for measuring two not consecutive L- junctions' distance in order to merge them
float merg_ditanceThrd=10;
// define the distance threshold for measureing two consecutive L- junctions's distance in order to merge them;
float consecuMerg_distanceThrd=20;
//define the search area when form L- junction by LSD lines;
float search_distance=40;
//define the distance threshold in x axis for determining turn Left or turn Right for Topology Coding;
float leftRight_ditanceThrd=25.0;
//we must display the srcImg Dir;
string srcImgDir="./test/";


int main(int argc, char * argv[])
{
    system("bash dataprep.sh");

    #pragma omp parallel for
    for(int n=1; n<argc; n++)
    {
        Mat srcImg=imread(argv[n],CV_LOAD_IMAGE_GRAYSCALE);
        Mat img=imread(argv[n],CV_LOAD_IMAGE_COLOR);
        string str=argv[n];
        cout<<"\n\n processing the image: "<<str<<endl;
        if (!srcImg.data)
        {
            cout<<"failed to load the image!\n";
            // return 0;
            continue;
        }

        for(int i=0; i< srcImg.rows; i++)
        {
            for(int j=0; j<5; j++)
            {
                srcImg.at<uchar>(i,j)=uchar(0);
                srcImg.at<uchar>(i,srcImg.cols-j-1)=uchar(0);
            }
        }

        for(int i=0; i<srcImg.cols; i++)
        {
            for(int j=0; j<5; j++)
            {
                srcImg.at<uchar>(j,i)=uchar(0);
                srcImg.at<uchar>(srcImg.rows-j-1,i)=uchar(0);
            }
        }

        //detect lsd lines in an input image;
        int cols=srcImg.rows;
        int rows=srcImg.cols;
        double*  lsd_srcImg=new double[cols*rows];
        for (int i=0; i<cols; i++)
        {
            for (int j=0; j<rows; j++)
            {
                lsd_srcImg[i+j*cols]=static_cast<double>(srcImg.at<uchar>(i,j));
            }
        }
        double* lsd_dstImg;
        int n=0;
        lsd_dstImg=lsd(&n,lsd_srcImg,cols,rows);

        cout<<"finished the lsd detection!\n";
        vector<LSDline> lsdLine;
        for (int i=0; i<n; i++)
        {
            LSDline lsdLine_tmp;
            lsdLine_tmp.lineBegin.y=lsd_dstImg[i*7+0];
            lsdLine_tmp.lineBegin.x=lsd_dstImg[i*7+1];
            lsdLine_tmp.lineEnd.y=lsd_dstImg[i*7+2];
            lsdLine_tmp.lineEnd.x=lsd_dstImg[i*7+3];
            lsdLine_tmp.width=lsd_dstImg[i*7+4];
            lsdLine_tmp.p=lsd_dstImg[i*7+5];
            lsdLine_tmp.log_nfa=lsd_dstImg[i*7+6];
            lsdLine_tmp.tagBegin=1;
            lsdLine_tmp.tagEnd=1;
            cout<<lsdLine_tmp.lineBegin.x<<" "<<lsdLine_tmp.lineBegin.y<<" "<<lsdLine_tmp.lineEnd.x<<"  "<<lsdLine_tmp.lineEnd.y<<endl;
            float distThreshold=12;
            if(sqrt((lsdLine_tmp.lineBegin.x-lsdLine_tmp.lineEnd.x)*(lsdLine_tmp.lineBegin.x-lsdLine_tmp.lineEnd.x)+
                    (lsdLine_tmp.lineBegin.y-lsdLine_tmp.lineEnd.y)*(lsdLine_tmp.lineBegin.y-lsdLine_tmp.lineEnd.y))>distThreshold)
            {
                lsdLine.push_back(lsdLine_tmp);
            }
        }

        cout<<"the detected lsd lines' number is: "<<lsdLine.size()<<endl;
        //define the img1 to display the detected LSD lines and junctions;
        Mat img1(img.size(),CV_8UC3,Scalar::all(0));
        delete[] lsd_srcImg;


        displayLSDline(lsdLine,img1);
        //imwrite("img1.bmp",img1);

        vector<Ljunct> Jlist;
        vector<LsdJunction> lsdJunction;



        if(LSD2Junct(lsdLine,Jlist,lsdJunction,search_distance,img))
        {
            cout<<"transform successfully!\n";
        }
        else
        {
            cout<<"cannot form L-junctions from LSD lines!\n";
            //for processing, we also need to write the the detect result;
            char c='_';
            int name_end=str.find(c,0);
            string ori_name_tmp1=str.substr(7,name_end-7);
            // char* ch2=".bmp";
            // int location=str.find(ch2,0);
            // string ori_name_tmp1 = str.substr(7,location-7);
            string dst_tmp = "./DetectResultOri/"+ori_name_tmp1;
            string ori_name_tmp="./OrigImg/"+ori_name_tmp1;

            // Mat oriImg_tmp = imread(ori_name_tmp.c_str(),CV_LOAD_IMAGE_COLOR);
            // imwrite(dst_tmp,oriImg_tmp);
            string filestring_tmp="./dstFile/"+str.substr(srcImgDir.size(),str.size()-4)+".jpg.txt";
            ofstream file_out(filestring_tmp.c_str());
            if(!file_out.is_open())
            {
                cout<<"cannot open the txt file!\n";
            }
            string imageName=str.substr(srcImgDir.size(),str.size()-4)+".jpg";
            file_out<<imageName<<"\t"<<img.cols<<"\t"<<img.rows<<endl;

            continue;
        }
        //vector<string> code_string1;
        vector<Ljunct> Jlist_coding;
        vector<codeStringBoundingBox> code_string;
        code_string=encodingFromLsdJunction(lsdJunction, Jlist_coding,srcImg);
        classifyRoadMarking(code_string,srcImg);

        string str_tmp=str.substr(srcImgDir.size(),str.size());
        cout<<"!!!!!the Jlist_coding size is: "<<Jlist_coding.size()<<endl<<endl;
        
        displayLjunct(Jlist_coding,img1,str_tmp);

        DrawBoundingBox(code_string,img,str_tmp);

        //drawing the bounding box in original image;
        char c='_';
        int name_end=str.find(c,0);
      //  string ori_name=str.substr(7,name_end-7);
        
         char* ch=".bmp";
         int location=str.find(ch,0);
         cout<<"the find .bmp in "<<str<<" is in "<<location<<endl;
         string ori_name=str.substr(7,location-7);
        
         cout<<ori_name<<endl;
         string ori_img="./OrigImg/"+ori_name+".JPG";

         Mat oriImg=imread(ori_img.c_str(),CV_LOAD_IMAGE_COLOR);
         if(!oriImg.data)
         {
             cout<<"cannot load the original image!\n";
             //return 0;
             char ch;
             cin.get(ch);
             continue;
         }
         
        /*
        Point2f imgP1=Point2f(219,668);
        Point2f imgP2=Point2f(452,469);
        Point2f imgP3=Point2f(622,472);
        Point2f imgP4=Point2f(882,681);
        Point2f imgP5=Point2f(388,520);
        Point2f imgP6=Point2f(688,523);
        Point2f imgP7=Point2f(454,538);
        Point2f imgP8=Point2f(645,539);
        Point2f imgP9=Point2f(508,486);
        Point2f imgP10=Point2f(573,509);

        Point2f imgP[10]= {imgP1,imgP2,imgP3,imgP4,imgP5,imgP6,imgP7,imgP8,imgP9,imgP10};
        Point2f objP1=Point2f(250,900);
        Point2f objP2=Point2f(250,100);
        Point2f objP3=Point2f(800,100);
        Point2f objP4=Point2f(800,900);
        Point2f objP5=Point2f(250,550);
        Point2f objP6=Point2f(800,550);
        Point2f objP7=Point2f(400,625);
        Point2f objP8=Point2f(650,625);
        Point2f objP9=Point2f(450,300);
        Point2f objP10=Point2f(600,475);



        Point2f objP[10]= {objP1,objP2,objP3,objP4,objP5,objP6,objP7,objP8,objP9,objP10};
        */
        vector<Point2f> imgP;
        imgP.push_back(Point2f(300,450));
        imgP.push_back(Point2f(700,450));
        imgP.push_back(Point2f(465,450));
        imgP.push_back(Point2f(535,450));
        imgP.push_back(Point2f(260,820));
        imgP.push_back(Point2f(740,820));

        vector<Point2f> objP;
        objP.push_back(Point2f(0,0));
        objP.push_back(Point2f(1000,0));
        objP.push_back(Point2f(400,0));
        objP.push_back(Point2f(600,0));
        objP.push_back(Point2f(400,1000));
        objP.push_back(Point2f(600,1000));

        //Mat H=getPerspectiveTransform(objP,imgP);
        Mat H=findHomography(objP,imgP,CV_RANSAC);
        DrawBoundingBox_Ori(code_string,oriImg,ori_name,H,str_tmp);
    }
    return 0;
}
