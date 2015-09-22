#include<stdlib.h>
#include<stdio.h>
#include<cmath>
#include<typeinfo>
#include "JuncDetect.h"
void JunctDec::classifyJlist(vector<Ljunct>& Jlist,vector<Ljunct>& acuteJunct,vector<Ljunct>& obtuseJunct, vector<Ljunct>& rightJunct)
{
	for(int i=0;i<Jlist.size();i++)
	{
		cout<<Jlist[i].to_str()<<endl;
	}
	double threshold=PI/36;
	for(int i=0; i<Jlist.size();i++)
	{
		float angleDistance=abs(Jlist[i].branch[0]-Jlist[i].branch[1])>PI?
			(2*PI-abs(Jlist[i].branch[0]-Jlist[i].branch[1])):
		abs(Jlist[i].branch[0]-Jlist[i].branch[1]);
		if(abs(abs(angleDistance-PI/2))<2*threshold)//right angle;
		{
			rightJunct.push_back(Jlist[i]);
			cout<<"have found the right junct:\n"<<Jlist[i].branch[0]<<" "<<Jlist[i].branch[1]<<endl;
			cout<<"the location is: x= "<<Jlist[i].location.x<<"  y=  "<<Jlist[i].location.y<<endl<<endl;;
			continue;
		}
		else
		{
			if(angleDistance<PI/2-2*threshold)//acute angle;
			{
				cout<<"have found the acute junct:\n"<<Jlist[i].branch[0]<<" "<<Jlist[i].branch[1]<<endl;
				cout<<"the location is: x= "<<Jlist[i].location.x<<"  y=  "<<Jlist[i].location.y<<endl<<endl;;
				acuteJunct.push_back(Jlist[i]);
				continue;
			}
			else
			{
				if(angleDistance>PI/2+2*threshold)
				{
					cout<<"have found the obtuse junct:\n"<<Jlist[i].branch[0]<<" "<<Jlist[i].branch[1]<<endl;
					cout<<"the location is: x= "<<Jlist[i].location.x<<"  y=  "<<Jlist[i].location.y<<endl<<endl;;
					obtuseJunct.push_back(Jlist[i]);//obtuse angle;        
				}
			}
		}
	}
}

bool JunctDec::findNextJunction(vector<Ljunct>& Jlist, Ljunct& junction,vector<Ljunct>& rstJunction)
{ 
  double angleThreshold=PI/15;
  
  vector<Ljunct> candidateBr1;
  vector<Ljunct> candidateBr2;
  if(findJlist(Jlist,junction.location,junction.branch[0],candidateBr1))
   {
     for(int i=0;i<candidateBr1.size();i++)
     {
         if(abs(atan2(abs(candidateBr1[i].location.y-junction.location.y),
                     abs(candidateBr1[i].location.x-junction.location.x))-
                     junction.branch[0])<angleThreshold &&
                 checkParallelism(junction.branch[0],candidateBr1[i]))
         {
             rstJunction.push_back(candidateBr1[i]);
             break;
         }
     }
   }
  if(findJlist(Jlist,junction.location,junction.branch[1],candidateBr2))
  {
      for(int i=0;i<candidateBr2.size();i++)
      {
          if(abs(atan2(abs(candidateBr2[i].location.y-junction.location.y),
                          abs(candidateBr1[i].location.x-junction.location.x))-
                      junction.branch[1])<angleThreshold &&
                  checkParallelism(junction.branch[1],candidateBr2[i]))
          {
              rstJunction.push_back(candidateBr1[i]);
              break;
          }
      }
  }
  if(rstJunction.size()==2)
      return true;
  else
      return false;
}

bool JunctDec::findNextJunforOneBranch(vector<Ljunct>& Jlist, Point2f location, double orientation, Ljunct& rstJunct)
{
    vector<Ljunct> candidate;
    double angleThreshold=PI/10;
    if(findJlist(Jlist, location, orientation,candidate))
    {
        for(int i=0;i<candidate.size();i++)
        {
			double k=0;
			if(location.x==candidate[i].location.x)
				k=PI/2;
			else
				if(location.y==candidate[i].location.y)
					k=0;
			    else
				k=atan2(candidate[i].location.y-location.y,candidate[i].location.x-location.x);
             if(abs(k-orientation)<angleThreshold &&
                     checkParallelism(orientation,candidate[i]))
             {
              rstJunct=candidate[i];
              break;
             }
        }

    }
    if(rstJunct.location.x>0)
        return true;
    else
        return false;
}


bool JunctDec::checkParallelism(double orientation, Ljunct& junction)
{
    double oriThreshold=PI/10;
    if(abs(orientation-junction.branch[0])<oriThreshold||abs(orientation-junction.branch[1])<oriThreshold||
		abs(abs(orientation-junction.branch[0])-PI)<oriThreshold||abs(abs(orientation-junction.branch[1])-PI)
		<oriThreshold)
        return true;
    else
        return false;
}


bool JunctDec::sortLlist(vector<Ljunct>& Jlist,const Point2f location)
{
    if(Jlist.size()==0)
        return false;
	if (Jlist.size()==1)
	{
		return true;
	}
	
    Ljunct junct_tmp;
    for(int i=0;i<Jlist.size()-1;i++)
        for(int j=0;j<Jlist.size()-1-i;j++)
        {
            if(abs(Jlist[j].location.x-location.x)-abs(Jlist[j+1].location.x-location.x)>0)
			{
                 junct_tmp=Jlist[j];
                 Jlist[j]=Jlist[j+1];
                Jlist[j+1]=junct_tmp;
            }
        }
    return true;
}

bool JunctDec::findJlist(const vector<Ljunct> Jlist, Point2f location, double branch, vector<Ljunct>& resultJlist)
{
  // vector<Ljunct> resultJlist;
	float threshold=0.001;
   if(abs(branch)<threshold)//if lies in the x axes;
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(abs(Jlist[i].location.y-location.y)<1&&Jlist[i].location.x>=location.x&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
		 {
            resultJlist.push_back(Jlist[i]);
		 }
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     }

   if(abs(branch+PI/2)<threshold)//if branch lies in -y axes;
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(abs(Jlist[i].location.x-location.x)<=1&&Jlist[i].location.y<=location.y&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
            resultJlist.push_back(Jlist[i]);
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     } 

   if(abs(abs(branch)-PI)<threshold)//if branch lies in -x axes
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(abs(Jlist[i].location.y-location.y)<=1&&Jlist[i].location.x<=location.x&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
            resultJlist.push_back(Jlist[i]);
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     }
     
   if(abs(branch-PI/2)<threshold)//if branch lies in +y axes;
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(abs(Jlist[i].location.x-location.x)<=1&&Jlist[i].location.y>=location.y&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
            resultJlist.push_back(Jlist[i]);
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     }

   if(branch>0.0001&&branch<PI/2-threshold)//if the branch lies in the forth area;
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(Jlist[i].location.y-location.y>=0&&Jlist[i].location.x>=location.x&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
            resultJlist.push_back(Jlist[i]);
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     }

   if(branch>PI/2+0.0001&&branch<PI-threshold)//if branch lies in the third area;
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(Jlist[i].location.y>=location.y&&Jlist[i].location.x<=location.x&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
            resultJlist.push_back(Jlist[i]);
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     }

   if(branch<-PI/2-0.001&&branch>-PI+threshold)//if branch lies in the second area;
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(Jlist[i].location.y<=location.y&&Jlist[i].location.x<=location.x&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
            resultJlist.push_back(Jlist[i]);
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     }

   if(branch>-PI/2+0.001&&branch<-threshold)//if branch lies in the first area;
     {
       for(int i=0;i<Jlist.size();i++)
       {
         if(Jlist[i].location.y<=location.y&&Jlist[i].location.x>=location.x&&
			 Jlist[i].tag!=0&&!(Jlist[i].location.x==location.x&&Jlist[i].location.y==location.y))
            resultJlist.push_back(Jlist[i]);
       }
       if(resultJlist.size()>0)
       {
           sortLlist(resultJlist,location);
           return true;
       }
       else 
          return false;
     }
  return true;
}
 
vector<Rectangle> JunctDec::findRectangle(vector<Ljunct>& rightJlist)
{
    if (rightJlist.size()<4)
    {
		cerr<<"failed to detect the rectangle!"<<endl;
    }
    
    vector<Rectangle> Lrecs;
    Rectangle rec;
    for(int i=0;i<rightJlist.size();i++)
    {
		if(rightJlist[i].tag==0) 
			continue;

        double branch1=rightJlist[i].branch[0];
        double branch2=rightJlist[i].branch[1];
        Ljunct junction1;
        Ljunct junction2;
        if(findNextJunforOneBranch( rightJlist, rightJlist[i].location,branch1,
                    junction1))
        {
            if(findNextJunforOneBranch(rightJlist, rightJlist[i].location, 
                        branch2,junction2))
            {
                Ljunct junction3;
                Ljunct junction4;
                if(findNextJunforOneBranch(rightJlist, junction1.location,junction1.branch[0],
                            junction3)&&findNextJunforOneBranch( rightJlist, junction1.location,
                                junction1.branch[1],junction4))
                {
					for (vector<Ljunct>::iterator t=rightJlist.begin();t!=rightJlist.end();t++)
					{
						if (t->location.x==junction1.location.x&&t->location.y==junction1.location.y)
						{
							t->tag=0;
						}
						if (t->location.x==junction2.location.x&&t->location.y==junction2.location.y)
						{
							t->tag=0;
						}
						if (t->location.x==junction3.location.x&&t->location.y==junction3.location.y)
						{
							t->tag=0;
						}
						if (t->location.x==junction4.location.x&&t->location.y==junction4.location.y)
						{
							t->tag=0;
						}
						
					}
					
                    rec.junct1=junction1;
                    rec.junct2=junction2;
                    rec.junct3=junction3;
                    rec.junct4=junction4;
					rec.tag=1;
					cout<<"have found the rectangle:\n";
					cout<<"first point is:\n";
					cout<<junction1.location.x<<"  "<<junction1.location.y<<endl;

					cout<<"have found the rectangle:\n";
					cout<<"second point is:\n";
					cout<<junction2.location.x<<"  "<<junction2.location.y<<endl;

					cout<<"have found the rectangle:\n";
					cout<<"third point is:\n";
					cout<<junction3.location.x<<"  "<<junction3.location.y<<endl;

					cout<<"have found the rectangle:\n";
					cout<<"forth point is:\n";
					cout<<junction4.location.x<<"  "<<junction4.location.y<<endl;

					cout<<"\n\n\n\n";
                    Lrecs.push_back(rec);
                }
            }
        }
    }
    return Lrecs;
}

bool JunctDec::findNextJunforOneBranchWithReturn(vector<Ljunct>& Jlist, Point2f location, double orientation, Ljunct& rstJunct, int* branch)
{
	vector<Ljunct> candidate;
	double angleThreshold=PI/10;
	if(findJlist(Jlist, location, orientation,candidate))
	{
		if (candidate.size()<1)
		{
			return false;
		}
		vector<Ljunct> candidate_final;
		for(int i=0;i<candidate.size();i++)
		{
			double k=0;
			if(location.x==candidate[i].location.x)
				k=PI/2;
			else
				if(location.y==candidate[i].location.y)
					k=0;
				else
					k=atan2(candidate[i].location.y-location.y,candidate[i].location.x-location.x);
			if(abs(k-orientation)<angleThreshold &&
				checkParallelism(orientation,candidate[i]))
			{
				candidate_final.push_back(candidate[i]);
				//float branch_tmp=orientation<0?orientation+PI:orientation-PI;
				//rstJunct=candidate[i];
				//if (abs(branch_tmp-rstJunct.branch[0])<0.1)
				//{
				//	*branch=0;
				//}
				//else
				//	*branch=1;
				//
				//break;
			}
		}
		float branch_tmp=orientation<0?orientation+PI:orientation-PI;
		float dist_short=0;
		float dist_tmp=0;
		for (int i=0; i<candidate_final.size(); i++)
		{
			if(i==0)
			{
				dist_tmp=sqrt((location.x-candidate_final[i].location.x)*(location.x-candidate_final[i].location.x)+
					(location.y-candidate_final[i].location.y)*(location.y-candidate_final[i].location.y));
				dist_short=dist_tmp;
				rstJunct=candidate_final[i];
				if (abs(branch_tmp-rstJunct.branch[0])<0.1)
				{
					*branch=0;
				}
				else
					*branch=1;
			}

			dist_tmp=sqrt((location.x-candidate_final[i].location.x)*(location.x-candidate_final[i].location.x)+
				(location.y-candidate_final[i].location.y)*(location.y-candidate_final[i].location.y));
			if (dist_short>dist_tmp)
			{
				dist_short=dist_tmp;
				rstJunct=candidate_final[i];
				if (abs(branch_tmp-rstJunct.branch[0])<0.1)
				{
					*branch=0;
				}
				else
					*branch=1;
			}

		}

	}
	if(rstJunct.location.x>0 && candidate.size()>0)
		return true;
	else
		return false;
}


//bool JunctDec::checkParallelism(double orientation, Ljunct& junction)
//{
//	double oriThreshold=PI/20;
//	if(abs(orientation-junction.branch[0])<oriThreshold||abs(orientation-junction.branch[1])<oriThreshold||
//		abs(abs(orientation-junction.branch[0])-PI)<oriThreshold||abs(abs(orientation-junction.branch[1])-PI)
//		<oriThreshold)
//		return true;
//	else
//		return false;
//}


 vector<Rectangle> JunctDec::findQuadrangle(vector<Ljunct>& allJlist)
 {
	 Ljunct Ljunct_initialize;
	 Ljunct_initialize.class_id=2;
	 Ljunct_initialize.tag=1;
	 Ljunct_initialize.location=Point2f(0,0);
	 Ljunct_initialize.branch[0]=0;
	 Ljunct_initialize.branch[1]=0;
	 Ljunct_initialize.strength[0]=0;
	 Ljunct_initialize.strength[1]=0;
	 vector<Rectangle> quaDrangle;
	 for (int i=0; i<allJlist.size();i++)
	 {
		 Rectangle quaDrangle_tmp;
		 if (allJlist[i].tag==0)
		 {
			 continue;
		 }
		 int branch1=4;
		 Ljunct junction1=Ljunct_initialize;
		 if (findNextJunforOneBranchWithReturn(allJlist,allJlist[i].location,allJlist[i].branch[0],junction1,&branch1)
			 && allJlist[i].location.x != junction1.location.x && allJlist[i].location.y != junction1.location.y)
		 {
			 double orientation1=branch1==0?junction1.branch[1]:junction1.branch[0];
			 int branch2=4;
			 Ljunct junction2=Ljunct_initialize;
			 if (findNextJunforOneBranchWithReturn(allJlist,junction1.location,orientation1,junction2,&branch2)
				 && junction1.location.x != junction2.location.x && junction1.location.y != junction2.location.y)
			 {
				 double orientation2=branch2==0?junction2.branch[1]:junction2.branch[0];
				 int branch3=4;
				 Ljunct junction3=Ljunct_initialize;
				 if (findNextJunforOneBranchWithReturn(allJlist,junction2.location,orientation2,junction3,&branch3)
					 && junction2.location.x != junction3.location.x && junction2.location.y != junction3.location.y)
				 {
					 double orientation3=branch3==0?junction3.branch[1]:junction3.branch[0];
					 int branch4=4;
					 Ljunct junction4=Ljunct_initialize;
					 if (findNextJunforOneBranchWithReturn(allJlist,junction3.location,orientation3,junction4,&branch4)
						 && junction4.location.x == allJlist[i].location.x && junction4.location.y ==allJlist[i].location.y)
					 {
						 quaDrangle_tmp.junct1=junction1;
						 quaDrangle_tmp.junct2=junction2;
						 quaDrangle_tmp.junct3=junction3;
						 quaDrangle_tmp.junct4=junction4;
						 quaDrangle.push_back(quaDrangle_tmp);
						 for (vector<Ljunct>::iterator t=allJlist.begin();t!=allJlist.end();t++)
						 {
							 if (t->location.x==junction1.location.x&&t->location.y==junction1.location.y)
							 {
								 t->tag=0;
							 }
							 if (t->location.x==junction2.location.x&&t->location.y==junction2.location.y)
							 {
								 t->tag=0;
							 }
							 if (t->location.x==junction3.location.x&&t->location.y==junction3.location.y)
							 {
								 t->tag=0;
							 }
							 if (t->location.x==junction4.location.x&&t->location.y==junction4.location.y)
							 {
								 t->tag=0;
							 }

						 }

					 }
				 }
			 }

		 }

	 }

	 return quaDrangle;
 }

bool JunctDec::isJunctCovered(vector<Ljunct>& Jlist, Ljunct& junction)
{
    for(vector<Ljunct>::iterator t=Jlist.begin();t<Jlist.end();t++)
    {
            if(abs(t->location.x-junction.location.x)<0.001 && abs(t->location.y-junction.location.y)<0.001 && t->tag==0)
                    return true;
            else
              return  false;
    }

    return true;

}

vector<Triangle> JunctDec::findAcuteTriangle(vector<Ljunct>& acuteJlist)
{
	if (acuteJlist.size()<3)
	{
		cerr<<"failed to detect the acute triangle because of the limited size of acuteJlist."<<endl;
	}
	
    vector<Triangle> tri;
    cout<<"in the function, the acuteJlist.size() is: "<<acuteJlist.size()<<endl;
   // vector<Ljunct> rstJunction;
   // vector<Ljunct> Jlist_tmp=acuteJlist;

    for(int i=0;i<acuteJlist.size();i++)
    {

		Ljunct junction1;
		Ljunct junction2;
		Ljunct junction3;
        if(acuteJlist[i].tag==0)
                continue;
        Triangle tri_tmp;
        if(findNextJunforOneBranch(acuteJlist,acuteJlist[i].location,acuteJlist[i].branch[0],junction1)&&
                findNextJunforOneBranch(acuteJlist,acuteJlist[i].location,acuteJlist[i].branch[1],junction2)
                &&(checkParallelism(junction1.branch[0],junction2)||checkParallelism(junction1.branch[1], junction2))
                && !isJunctCovered(acuteJlist,junction1) && !isJunctCovered(acuteJlist,junction2)
				&& sqrt((acuteJlist[i].location.x-junction1.location.x)*(acuteJlist[i].location.x-junction1.location.x)+
				(acuteJlist[i].location.y-junction1.location.y)*(acuteJlist[i].location.y-junction1.location.y))>90
				&& sqrt((acuteJlist[i].location.x-junction2.location.x)*(acuteJlist[i].location.x-junction2.location.x)+
				(acuteJlist[i].location.y-junction2.location.y)*(acuteJlist[i].location.y-junction2.location.y))>90
				&& sqrt((junction2.location.x-junction1.location.x)*(junction2.location.x-junction1.location.x)+
				(junction2.location.y-junction1.location.y)*(junction2.location.y-junction1.location.y))>90
				&& sqrt((acuteJlist[i].location.x-junction1.location.x)*(acuteJlist[i].location.x-junction1.location.x)+
				(acuteJlist[i].location.y-junction1.location.y)*(acuteJlist[i].location.y-junction1.location.y))<250
				&& sqrt((acuteJlist[i].location.x-junction2.location.x)*(acuteJlist[i].location.x-junction2.location.x)+
				(acuteJlist[i].location.y-junction2.location.y)*(acuteJlist[i].location.y-junction2.location.y))<250
				&& sqrt((junction2.location.x-junction1.location.x)*(junction2.location.x-junction1.location.x)+
				(junction2.location.y-junction1.location.y)*(junction2.location.y-junction1.location.y))<250)
            {
                for(vector<Ljunct>::iterator t=acuteJlist.begin();t<acuteJlist.end();t++)
                {
                        if(t->location.x==junction1.location.x && t->location.y==junction1.location.y)
                                t->tag=0;
                        if(t->location.x==junction2.location.x && t->location.y==junction2.location.y)
                                t->tag=0;
                }

                acuteJlist[i].tag=0;
                 cout<<"find the appropriate acute Junction.\n";
                 tri_tmp.junct1=acuteJlist[i];
                 tri_tmp.junct2=junction1;
                 tri_tmp.junct3=junction2;
				 tri_tmp.tag=1;
                 cout<<"the detected acutetriangle's three points are:\n";
                 cout<<"point1: "<<tri_tmp.junct1.location.x<<" "<<tri_tmp.junct1.location.y<<endl;
                 cout<<"point2: "<<tri_tmp.junct2.location.x<<" "<<tri_tmp.junct2.location.y<<endl;
                 cout<<"point1: "<<tri_tmp.junct3.location.x<<" "<<tri_tmp.junct3.location.y<<endl;

                 tri.push_back(tri_tmp);
            }
    }
    
    return tri;
}

void JunctDec::findConnectTriRec(Triangle& tri, float* x_min, float* x_max, float* y_max)
{
    *x_min=tri.junct1.location.x;
    *x_max=tri.junct1.location.x;
    *y_max=tri.junct1.location.y;

    *x_min=*x_min>tri.junct2.location.x?tri.junct2.location.x:*x_min;
    *x_max=*x_max<tri.junct2.location.x?tri.junct2.location.x:*x_max;
    *y_max=*y_max<tri.junct2.location.y?tri.junct2.location.y:*y_max;

    *x_min=*x_min>tri.junct3.location.x?tri.junct3.location.x:*x_min;
    *x_max=*x_max<tri.junct3.location.x?tri.junct3.location.x:*x_max;
    *y_max=*y_max<tri.junct3.location.y?tri.junct3.location.y:*y_max;
}

vector<Triangle> JunctDec::findObtuseTriangle(vector<Ljunct>& acuteJlist,vector<Ljunct>& obtuseJlist)
{
	if (obtuseJlist.size()<1)
	{
		cerr<<"failed to detect the obtuse Triangle because of the limited size of obtuseJlist.\n";
	}
	
    vector<Triangle> obtuseTriangle;
    for(int i=0;i<obtuseJlist.size();i++)
    {
		if (obtuseJlist[i].tag==0)
		{
			continue;
		}
		
        Triangle triangle_tmp;
        Ljunct junction1;
        Ljunct junction2;
        if(findNextJunforOneBranch(acuteJlist,obtuseJlist[i].location,obtuseJlist[i].branch[0],junction1)&& 
                findNextJunforOneBranch(acuteJlist,obtuseJlist[i].location,obtuseJlist[i].branch[1],junction2)
                &&(checkParallelism(junction1.branch[0],junction2) || checkParallelism(junction1.branch[0],obtuseJlist[i]))
				&&(checkParallelism(junction1.branch[1],obtuseJlist[i]) || checkParallelism(junction1.branch[1],junction2))
				&&(checkParallelism(junction2.branch[0],junction1) || checkParallelism(junction2.branch[0],obtuseJlist[i]))
				&&(checkParallelism(junction2.branch[1],obtuseJlist[i]) || checkParallelism(junction2.branch[1],junction1))
				&& !isJunctCovered(acuteJlist,junction1) && !isJunctCovered(acuteJlist,junction2)
				&& sqrt((obtuseJlist[i].location.x-junction1.location.x)*(obtuseJlist[i].location.x-junction1.location.x)+
				(obtuseJlist[i].location.y-junction1.location.y)*(obtuseJlist[i].location.y-junction1.location.y))>50
				&& sqrt((obtuseJlist[i].location.x-junction2.location.x)*(obtuseJlist[i].location.x-junction2.location.x)+
				(obtuseJlist[i].location.y-junction2.location.y)*(obtuseJlist[i].location.y-junction2.location.y))>50
				&& sqrt((junction2.location.x-junction1.location.x)*(junction2.location.x-junction1.location.x)+
				(junction2.location.y-junction1.location.y)*(junction2.location.y-junction1.location.y))>50
				&& sqrt((obtuseJlist[i].location.x-junction1.location.x)*(obtuseJlist[i].location.x-junction1.location.x)+
				(obtuseJlist[i].location.y-junction1.location.y)*(obtuseJlist[i].location.y-junction1.location.y))<300
				&& sqrt((obtuseJlist[i].location.x-junction2.location.x)*(obtuseJlist[i].location.x-junction2.location.x)+
				(obtuseJlist[i].location.y-junction2.location.y)*(obtuseJlist[i].location.y-junction2.location.y))<300
				&& sqrt((junction2.location.x-junction1.location.x)*(junction2.location.x-junction1.location.x)+
				(junction2.location.y-junction1.location.y)*(junction2.location.y-junction1.location.y))<300)
        {
			for(vector<Ljunct>::iterator t=acuteJlist.begin();t<acuteJlist.end();t++)
			{
				if(t->location.x==junction1.location.x && t->location.y==junction1.location.y)
					t->tag=0;
				if(t->location.x==junction2.location.x && t->location.y==junction2.location.y)
					t->tag=0;
			}
			obtuseJlist[i].tag=0;
            triangle_tmp.junct1=obtuseJlist[i];
            triangle_tmp.junct2=junction1;
            triangle_tmp.junct3=junction2;
			triangle_tmp.tag=1;
            obtuseTriangle.push_back(triangle_tmp);
            
            
            cout<<"\n\n\n\n  the detected botusetriangle's three points are:\n";
                 cout<<"point1: "<<triangle_tmp.junct1.location.x<<" "<<triangle_tmp.junct1.location.y<<endl;
                 cout<<"point2: "<<triangle_tmp.junct2.location.x<<" "<<triangle_tmp.junct2.location.y<<endl;
                 cout<<"point1: "<<triangle_tmp.junct3.location.x<<" "<<triangle_tmp.junct3.location.y<<endl;

        }
        
    }

    return obtuseTriangle;
}

vector<Rhombus> JunctDec::findRhombus(vector<Ljunct>& acuteJlist, vector<Ljunct>& obtuseJlist)
{
    vector<Rhombus> rhombus;
    int num=acuteJlist.size()>obtuseJlist.size()?obtuseJlist.size():acuteJlist.size();
    printf("%d\n", num);
    for(int i=0;i<obtuseJlist.size();i++)
    {
		if (obtuseJlist[i].tag==0)
			continue;
        Rhombus rhombus_tmp;
        Ljunct junction1;
        Ljunct junction2;
        if(findNextJunforOneBranch(acuteJlist, obtuseJlist[i].location,obtuseJlist[i].branch[0],junction1)
                &&findNextJunforOneBranch(acuteJlist,obtuseJlist[i].location,obtuseJlist[i].branch[1],
                    junction2)&&checkParallelism(junction1.branch[0],junction2)&&
                checkParallelism(junction1.branch[1],junction2))
        {
            Ljunct junction3;
            Ljunct junction4;
            if(findNextJunforOneBranch(obtuseJlist,junction1.location,junction1.branch[0],junction3)
                &&findNextJunforOneBranch(obtuseJlist,junction1.location,junction1.branch[1],
                    junction4)&&checkParallelism(junction3.branch[0],junction4)&&
                checkParallelism(junction3.branch[1],junction4))
            {
                rhombus_tmp.acuteJunct1=junction1;
                rhombus_tmp.acuteJunct2=junction2;
                rhombus_tmp.obtuseJunct1=junction3;
                rhombus_tmp.obtuseJunct2=junction4;
				rhombus_tmp.tag=1;

				for(vector<Ljunct>::iterator t=acuteJlist.begin();t<acuteJlist.end();t++)
				{
					if(t->location.x==junction1.location.x && t->location.y==junction1.location.y)
						t->tag=0;
					if(t->location.x==junction2.location.x && t->location.y==junction2.location.y)
						t->tag=0;
				}
				for(vector<Ljunct>::iterator t=obtuseJlist.begin();t<obtuseJlist.end();t++)
				{
					if(t->location.x==junction3.location.x && t->location.y==junction3.location.y)
						t->tag=0;
					if(t->location.x==junction4.location.x && t->location.y==junction4.location.y)
						t->tag=0;
				}

                rhombus.push_back(rhombus_tmp);
            }

        }
    }
    return rhombus;
}


vector<QuasiRhombus> JunctDec::findQuasiRhombus(vector<Ljunct>& acuteJlist,vector<Ljunct>& obtuseJlist,
        vector<Ljunct>& rightJlist)
{
    vector<QuasiRhombus> quaRhs;
    for(int i=0;i<acuteJlist.size();i++)
    {
		if (acuteJlist[i].tag==0)
			continue;
        QuasiRhombus quaRhs_tmp;
        Ljunct junction1;
        Ljunct junction2;
        if(findNextJunforOneBranch(obtuseJlist,acuteJlist[i].location,acuteJlist[i].branch[0],junction1)
                &&findNextJunforOneBranch(obtuseJlist,acuteJlist[i].location,acuteJlist[i].branch[1],
                    junction2)&&checkParallelism(junction1.branch[0],junction2)&&
                checkParallelism(junction1.branch[1],junction2))
        {
            double b1=junction1.branch[0]>junction1.branch[1]?junction1.branch[0]:junction1.branch[1];
            double b2=junction2.branch[0]>junction2.branch[1]?junction2.branch[0]:junction2.branch[1];
            double b=b1>b2?b1:b2;
            Point2f location;
            if(abs(b-junction1.branch[0])<0.0001||abs(b-junction1.branch[1])<0.0001)
                location=junction1.location;
            else
                location=junction2.location;
            Ljunct junction3;
            if(findNextJunforOneBranch(rightJlist,location,b,junction3)&&
                    checkParallelism(b,junction3))
            {
                double b3=PI-abs(junction3.branch[0])>PI-abs(junction3.branch[1])?junction3.branch[1]:
                    junction3.branch[0];
                Ljunct junction4;
                if(findNextJunforOneBranch(rightJlist,junction3.location,b3,junction4)&&
                        checkParallelism(b3,junction4))
                {
                    double b4=abs(PI/2-junction4.branch[0])<abs(PI/2-junction4.branch[1])?
                        junction4.branch[0]:junction4.branch[1];
                    Ljunct junction5;
                    if(findNextJunforOneBranch(obtuseJlist,junction4.location,b4,junction5)&&
                            checkParallelism(b4,junction5))
                    {
                       quaRhs_tmp.rightJunct1=junction3;
                       quaRhs_tmp.rightJunct2=junction4;
                       
                       quaRhs_tmp.obtuseJunct1=junction1;
                       quaRhs_tmp.obtuseJunct2=junction2;
                       quaRhs_tmp.obtuseJunct3=junction5;
                       quaRhs_tmp.acuteJunct=acuteJlist[i];
					   quaRhs_tmp.tag=1;
                       quaRhs.push_back(quaRhs_tmp);
					   acuteJlist[i].tag=0;
					   for(vector<Ljunct>::iterator t=obtuseJlist.begin();t<obtuseJlist.end();t++)
					   {
						   if(t->location.x==junction1.location.x && t->location.y==junction1.location.y)
						   {
							   t->tag=0;
							   break;
						   }
					   }
					   for(vector<Ljunct>::iterator t=obtuseJlist.begin();t<obtuseJlist.end();t++)
					   {
						   if(t->location.x==junction2.location.x && t->location.y==junction2.location.y)
						   {
							   t->tag=0;
							   break;
						   }
					   }
					   for(vector<Ljunct>::iterator t=rightJlist.begin();t<rightJlist.end();t++)
					   {
						   if(t->location.x==junction3.location.x && t->location.y==junction3.location.y)
						   {
							   t->tag=0;
							   break;
						   }
					   }
					   for(vector<Ljunct>::iterator t=rightJlist.begin();t<rightJlist.end();t++)
					   {
						   if(t->location.x==junction4.location.x && t->location.y==junction4.location.y)
						   {
							   t->tag=0;
							   break;
						   }
					   }
					   for(vector<Ljunct>::iterator t=obtuseJlist.begin();t<obtuseJlist.end();t++)
					   {
						   if(t->location.x==junction5.location.x && t->location.y==junction5.location.y)
						   {
							   t->tag=0;
							   break;
						   }
					   }


                    }

                }
            }
        }
    }
    return quaRhs;
}

bool JunctDec::isLiesOnTriangle(Ljunct junction, Triangle& triangle)
{
    if(isLiesOnLine(triangle.junct1.location,triangle.junct2.location,junction.location)
            &&(checkParallelism(junction.branch[0],triangle.junct1)||
                checkParallelism(junction.branch[1],triangle.junct1)))
        return true;
    else
        if(isLiesOnLine(triangle.junct1.location,triangle.junct3.location,junction.location)
                &&(checkParallelism(junction.branch[0],triangle.junct3)||
                    checkParallelism(junction.branch[1],triangle.junct3)))
            return true;
        else
            if(isLiesOnLine(triangle.junct2.location,triangle.junct3.location,junction.location)
                    &&(checkParallelism(junction.branch[0],triangle.junct2)||
                        checkParallelism(junction.branch[1],triangle.junct2)))
                return true;
            else
                return false;
}

bool JunctDec::isLiesOnLine(Point2f& p1, Point2f& p2, Point2f& p_test)
{
    float threshold=5;
    if(abs(p1.x-p2.x)<threshold)
    {
        if(abs(p_test.x-p1.x)<threshold&&(p_test.y-p1.y)*(p_test.y-p2.y)<0)
            return true;
        else
            return false;
    }
    if(abs(p1.y-p2.y)<threshold)
    {
        if(abs(p_test.y-p1.y)<threshold&&(p_test.x-p1.x)*(p_test.x-p2.x)<0)
            return true;
        else
            return false;
        
    }
    else
    {
        float k=(p1.y-p2.y)/(p1.x-p2.x);
		float b=p1.y-k*p1.x;
        float y_expect=k*p_test.x+b;
        if(abs(y_expect-p_test.y)<30)//30 pixel is OK;
            return true;
        else
            return false;
    }
}

int JunctDec::findBranchTag(Ljunct& junction, Triangle& triangle)
{
    if(isLiesOnLine(triangle.junct1.location,triangle.junct2.location,junction.location))
    {
        if(checkParallelism(junction.branch[0],triangle.junct1))
            return 0;
        else
            return 1;
    }
    if(isLiesOnLine(triangle.junct1.location,triangle.junct3.location,junction.location))
    {
        if(checkParallelism(junction.branch[0],triangle.junct3))
            return 0;
        else
            return 1;
    }
    else
    {
        if(checkParallelism(junction.branch[0],triangle.junct2))
            return 0;
        else
            return 1;
    }
}

int JunctDec::findBranchTagForRectangle(Ljunct& junction, Rectangle& rectangle)
{
	if(isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,junction.location))
	{
		if(checkParallelism(junction.branch[0],rectangle.junct1))
			return 0;
		else
			return 1;
	}
	if(isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,junction.location))
	{
		if(checkParallelism(junction.branch[0],rectangle.junct3))
			return 0;
		else
			return 1;
	}
	if(isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,junction.location))
	{
		if(checkParallelism(junction.branch[0],rectangle.junct4))
			return 0;
		else
			return 1;
	}
	if(isLiesOnLine(rectangle.junct2.location,rectangle.junct3.location,junction.location))
	{
		if(checkParallelism(junction.branch[0],rectangle.junct3))
			return 0;
		else
			return 1;
	}
	if(isLiesOnLine(rectangle.junct2.location,rectangle.junct4.location,junction.location))
	{
		if(checkParallelism(junction.branch[0],rectangle.junct4))
			return 0;
		else
			return 1;
	}
	if(isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,junction.location))
	{
		if(checkParallelism(junction.branch[0],rectangle.junct4))
			return 0;
		else
			return 1;
	}
	else
	{
		cerr<<"cannot find the tag!\n";
		return 3;
	}
}

bool JunctDec::isLiesOnRectangle(Rectangle& rectangle, Ljunct junction)
{
	if(isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,junction.location)
		&&(checkParallelism(junction.branch[0],rectangle.junct1)||
		checkParallelism(junction.branch[1],rectangle.junct1)))
		return true;
	else
		if(isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,junction.location)
			&&(checkParallelism(junction.branch[0],rectangle.junct3)||
			checkParallelism(junction.branch[1],rectangle.junct3)))
			return true;
		else
			if(isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,junction.location)
				&&(checkParallelism(junction.branch[0],rectangle.junct1)||
				checkParallelism(junction.branch[1],rectangle.junct1)))
				return true;
			else
				if(isLiesOnLine(rectangle.junct2.location,rectangle.junct3.location,junction.location)
					&&(checkParallelism(junction.branch[0],rectangle.junct2)||
					checkParallelism(junction.branch[1],rectangle.junct2)))
					return true;
				else
					if(isLiesOnLine(rectangle.junct2.location,rectangle.junct4.location,junction.location)
						&&(checkParallelism(junction.branch[0],rectangle.junct2)||
						checkParallelism(junction.branch[1],rectangle.junct2)))
						return true;
					else
						if(isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,junction.location)
							&&(checkParallelism(junction.branch[0],rectangle.junct3)||
							checkParallelism(junction.branch[1],rectangle.junct3)))
							return true;
						else
							return false;
}

void JunctDec::changeDirectionAfterRecExtraction(vector<Ljunct>& obtuseJlist, vector<Ljunct>& acuteJlist,
	vector<Rectangle>& rectangle)
{
	vector<Ljunct> acuteJlist_tmp;
	vector<Ljunct> obtuseJlist_tmp;
	for (int i=0;i<rectangle.size();i++)
	{
		for(int j1=0;j1<acuteJlist.size();j1++)
		{
			if(isLiesOnRectangle(rectangle[i],acuteJlist[j1])&&acuteJlist[j1].tag!=0)
			{
				Ljunct junction_tmp=acuteJlist[j1]; 
				int b=findBranchTagForRectangle(acuteJlist[j1],rectangle[i]);
				cout<<"\n \n \n Hey! change this direction for acuteJunction! After rectangle extraction.\n \n \n";
				cout<<"the location is: "<<junction_tmp.location.x<<"  "<<junction_tmp.location.y<<endl;
				cout<<"before changing, two branches are: "<<junction_tmp.branch[0]<<" "<<junction_tmp.branch[1]<<endl;
				junction_tmp.branch[b]=junction_tmp.branch[b]<0?junction_tmp.branch[b]+PI:
					junction_tmp.branch[b]-PI;
				cout<<"after changing, two branches are: "<<junction_tmp.branch[0]<<" "<<junction_tmp.branch[1]<<endl;
				//obtuseJlist.push_back(junction_tmp);
				obtuseJlist_tmp.push_back(junction_tmp);
				for (vector<Ljunct>::iterator t=acuteJlist.begin();t!=acuteJlist.end();t++)
				{ 
					if (t->location.x==acuteJlist[j1].location.x&&
						t->location.y==acuteJlist[j1].location.y)
					{
						t->tag=0;
						break;
					}
				}
			}
		}


		for(int j1=0;j1<obtuseJlist.size();j1++)
		{
			if(isLiesOnRectangle(rectangle[i],obtuseJlist[j1])&&obtuseJlist[j1].tag!=0)
			{
				Ljunct junction_tmp=obtuseJlist[j1];

				int b=findBranchTagForRectangle(obtuseJlist[j1],rectangle[i]);
				cout<<"\n \n \n Hey! change this direction for obtuseJunction! After rectangle extraction.\n";
				cout<<"the location is: "<<junction_tmp.location.x<<"  "<<junction_tmp.location.y<<endl;
				cout<<"before changing, two branches are: "<<junction_tmp.branch[0]<<" "<<junction_tmp.branch[1]<<endl;
				junction_tmp.branch[b]=junction_tmp.branch[b]<0?junction_tmp.branch[b]+PI:
					junction_tmp.branch[b]-PI;
				cout<<"after changing, two branches are: "<<junction_tmp.branch[0]<<" "<<junction_tmp.branch[1]<<endl;
				acuteJlist_tmp.push_back(junction_tmp);
				for (vector<Ljunct>::iterator t=obtuseJlist.begin();t!=obtuseJlist.end();t++)
				{
					if (t->location.x==obtuseJlist[j1].location.x&&
						t->location.y==obtuseJlist[j1].location.y)
					{
						t->tag=0;
						break;
					}
				}
			}
		}	
	}

	for (int i=0;i<acuteJlist_tmp.size();i++)
	{
		acuteJlist.push_back(acuteJlist_tmp[i]);
	}

	for (int i=0; i<obtuseJlist_tmp.size();i++)
	{
		obtuseJlist.push_back(obtuseJlist_tmp[i]);
	}
}

void JunctDec::changeDirection(vector<Triangle>& obtuseTriangle, vector<Triangle>& acuteTriangle,
        vector<Ljunct>& rightJlist, vector<Ljunct>& acuteJlist, vector<Ljunct>& obtuseJlist)
{
	vector<Ljunct> acuteJlist_tmp;
	vector<Ljunct> obtuseJlist_tmp;
    for(int i=0;i<obtuseTriangle.size();i++)
    {
        for(int j1=0;j1<rightJlist.size();j1++)
        {
            if(isLiesOnTriangle(rightJlist[j1],obtuseTriangle[i])&&rightJlist[j1].tag!=0)
            {
                int b=findBranchTag(rightJlist[j1],obtuseTriangle[i]);
				cout<<"\n \n \n Hey! change this rightJunction in obtuseTriangle. \n \n \n";
				cout<<"the location is:\n"<<rightJlist[j1].location.x<<"  "<<rightJlist[j1].location.y<<endl;
				cout<<"before changing, the branch is:\n"<<rightJlist[j1].branch[0]<<"  "<<rightJlist[j1].branch[1]<<endl;
                rightJlist[j1].branch[b]=rightJlist[j1].branch[b]<0?rightJlist[j1].branch[b]+PI:
                    rightJlist[j1].branch[b]-PI;

				cout<<"after changing, the branch is:\n"<<rightJlist[j1].branch[0]<<"  "<<rightJlist[j1].branch[1]<<endl;
				
            }
        }
        for(int j1=0;j1<acuteJlist.size();j1++)
        {
            if(isLiesOnTriangle(acuteJlist[j1],obtuseTriangle[i])&&acuteJlist[j1].tag!=0)
            {
                Ljunct junction_tmp=acuteJlist[j1];
                int b=findBranchTag(acuteJlist[j1],obtuseTriangle[i]);
				cout<<"\n \n \n Hey! change this acuteJunction in obtuseTriangle \n \n \n";
				cout<<"the location is:\n"<<junction_tmp.location.x<<"  "<<junction_tmp.location.y<<endl;
				cout<<"before changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                junction_tmp.branch[b]=junction_tmp.branch[b]<0?junction_tmp.branch[b]+PI:
                    junction_tmp.branch[b]-PI;
				cout<<"after changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                obtuseJlist_tmp.push_back(junction_tmp);
				for (vector<Ljunct>::iterator t=acuteJlist.begin();t!=acuteJlist.end();t++)
				{
					if (t->location.x==acuteJlist[j1].location.x&&
						t->location.y==acuteJlist[j1].location.y)
					{
						t->tag=0;
						break;
					}

				}
            }
        }

        for(int j1=0;j1<obtuseJlist.size();j1++)
        {
            if(isLiesOnTriangle(obtuseJlist[j1],obtuseTriangle[i])&&obtuseJlist[j1].tag!=0)
            {
                Ljunct junction_tmp=obtuseJlist[j1];

                int b=findBranchTag(obtuseJlist[j1],obtuseTriangle[i]);
				cout<<"\n \n \n Hey! change this obtuseJunction in obtuseTriangle. \n \n \n \n";
				cout<<"the location is:\n"<<junction_tmp.location.x<<"  "<<junction_tmp.location.y<<endl;
				cout<<"before changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                junction_tmp.branch[b]=junction_tmp.branch[b]<0?junction_tmp.branch[b]+PI:
                    junction_tmp.branch[b]-PI;
				cout<<"after changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                acuteJlist_tmp.push_back(junction_tmp);
				for (vector<Ljunct>::iterator t=obtuseJlist.begin();t!=obtuseJlist.end();t++)
				{
					if (t->location.x==obtuseJlist[j1].location.x&&
						t->location.y==obtuseJlist[j1].location.y)
					{
						t->tag=0;
						break;
					}

				}
            }
        }

    }

    for(int i=0;i<acuteTriangle.size();i++)
    {
        for(int j1=0;j1<rightJlist.size();j1++)
        {
            if(isLiesOnTriangle(rightJlist[j1],acuteTriangle[i])&&rightJlist[j1].tag!=0)
            {
                int b=findBranchTag(rightJlist[j1],acuteTriangle[i]);
				cout<<"\n \n \n Hey! change this right direction! in acuteTraingle \n \n \n";
				cout<<"the location is:\n"<<rightJlist[j1].location.x<<"  "<<rightJlist[j1].location.y<<endl;
				cout<<"before changing, the branch is:\n"<<rightJlist[j1].branch[0]<<"  "<<rightJlist[j1].branch[1]<<endl;
                rightJlist[j1].branch[b]=rightJlist[j1].branch[b]<0?rightJlist[j1].branch[b]+PI:
                    rightJlist[j1].branch[b]-PI;
				cout<<"after changing, the branch is:\n"<<rightJlist[j1].branch[0]<<"  "<<rightJlist[j1].branch[1]<<endl;
            }
        }
        for(int j1=0;j1<acuteJlist.size();j1++)
        {
            if(isLiesOnTriangle(acuteJlist[j1],acuteTriangle[i])&&acuteJlist[j1].tag!=0)
            {
                Ljunct junction_tmp=acuteJlist[j1];
                int b=findBranchTag(acuteJlist[j1],acuteTriangle[i]);
				cout<<"\n \n \n Hey! change this acuteJunction in acuteTriangle! \n \n \n";
				cout<<"the location is:\n"<<junction_tmp.location.x<<"  "<<junction_tmp.location.y<<endl;
				cout<<"before changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                junction_tmp.branch[b]=junction_tmp.branch[b]<0?junction_tmp.branch[b]+PI:
                    junction_tmp.branch[b]-PI;
				cout<<"after changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                obtuseJlist_tmp.push_back(junction_tmp);
				for (vector<Ljunct>::iterator t=acuteJlist.begin();t!=acuteJlist.end();t++)
				{
					if (t->location.x==acuteJlist[j1].location.x&&
						t->location.y==acuteJlist[j1].location.y)
					{
						t->tag=0;
						break;
					}
				}
            }
        }

        for(int j1=0;j1<obtuseJlist.size();j1++)
        {
            if(isLiesOnTriangle(obtuseJlist[j1],acuteTriangle[i])&&obtuseJlist[j1].tag!=0)
            {
                Ljunct junction_tmp=obtuseJlist[j1];

				cout<<"\n \n \n Hey! change this obtuseJunction in acuteTriangle! \n \n \n";
				
                int b=findBranchTag(obtuseJlist[j1],acuteTriangle[i]);
				cout<<"the location is:\n"<<junction_tmp.location.x<<"  "<<junction_tmp.location.y<<endl;
				cout<<"before changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                junction_tmp.branch[b]=junction_tmp.branch[b]<0?junction_tmp.branch[b]+PI:
                    junction_tmp.branch[b]-PI;
				cout<<"after changing, the branch is:\n"<<junction_tmp.branch[0]<<"  "<<junction_tmp.branch[1]<<endl;
                acuteJlist_tmp.push_back(junction_tmp);
				for (vector<Ljunct>::iterator t=obtuseJlist.begin();t!=obtuseJlist.end();t++)
				{
					if (t->location.x==obtuseJlist[j1].location.x&&
						t->location.y==obtuseJlist[j1].location.y)
					{
						t->tag=0;
						break;
					}

				}
            }
        }
    }

	for (int i=0;i<acuteJlist_tmp.size();i++)
	{
		acuteJlist.push_back(acuteJlist_tmp[i]);
	}

	for (int i=0;i<obtuseJlist_tmp.size();i++)
	{
		obtuseJlist.push_back(obtuseJlist_tmp[i]);
	}	
}

bool JunctDec::isTriangleRectangleConnect(Triangle& triangle, Rectangle& rectangle)
{
    int tag=0;
    if(isLiesOnTriangle(rectangle.junct1,triangle))
        tag++;
    if(isLiesOnTriangle(rectangle.junct2,triangle))
        tag++;
    if(isLiesOnTriangle(rectangle.junct3,triangle))
        tag++;
    if(isLiesOnTriangle(rectangle.junct4,triangle))
        tag++;
    if(tag==2)
        return true;
    else
        return false;
}


bool JunctDec::isTriangleRhombusConnect(Triangle& triangle, Rhombus& rhombus)
{
    int tag1=0;
    int tag2=0;
    if(isLiesOnTriangle(rhombus.acuteJunct1,triangle))
        tag1++;
    if(isLiesOnTriangle(rhombus.acuteJunct2,triangle))
        tag1++;
    if(isLiesOnTriangle(rhombus.obtuseJunct1,triangle))
        tag2++;
    if(isLiesOnTriangle(rhombus.obtuseJunct2,triangle))
        tag2++;
    
    if(tag1==1&&tag2==1)
        return true;
    else
        return false;
}

bool JunctDec::isRectangleRhombusConnect(Rectangle& rectangle,Rhombus& rhombus)
{
    if(checkParallelism(rectangle.junct1.branch[0],rectangle.junct2)
            &&((isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.acuteJunct1.location)&&
                isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.acuteJunct1.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.obtuseJunct2.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct2.location,rhombus.obtuseJunct2.location))))
        return true;

    if(checkParallelism(rectangle.junct1.branch[0],rectangle.junct3)
            &&((isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.acuteJunct1.location)&&
                isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.acuteJunct1.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.obtuseJunct2.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct3.location,rhombus.obtuseJunct2.location))))
        return true;

    if(checkParallelism(rectangle.junct1.branch[0],rectangle.junct4)
            &&((isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.acuteJunct1.location)&&
                isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.acuteJunct1.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.obtuseJunct2.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct1.location,rectangle.junct4.location,rhombus.obtuseJunct2.location))))
        return true;

    if(checkParallelism(rectangle.junct2.branch[0],rectangle.junct3)
            &&((isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.acuteJunct1.location)&&
                isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.acuteJunct1.location)&&
                 isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.obtuseJunct2.location))||
                (isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct3.location,rectangle.junct2.location,rhombus.obtuseJunct2.location))))
        return true;

    if(checkParallelism(rectangle.junct4.branch[0],rectangle.junct2)
            &&((isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.acuteJunct1.location)&&
                isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.acuteJunct1.location)&&
                 isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.obtuseJunct2.location))||
                (isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct4.location,rectangle.junct2.location,rhombus.obtuseJunct2.location))))
        return true;

    if(checkParallelism(rectangle.junct3.branch[0],rectangle.junct4)
            &&((isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.acuteJunct1.location)&&
                isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.acuteJunct1.location)&&
                 isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.obtuseJunct2.location))||
                (isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.obtuseJunct1.location))||
                (isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.acuteJunct2.location)&&
                 isLiesOnLine(rectangle.junct3.location,rectangle.junct4.location,rhombus.obtuseJunct2.location))))
        return true;
    else
        return false;
}


vector<Forward>  JunctDec::findForward(vector<Triangle>& acuteTriangle, vector<Rectangle>& rectangle, vector<Rhombus>& rhombus)
{
    vector<Forward> forward;
    for(int i=0; i<acuteTriangle.size();i++)
    {
		if (acuteTriangle[i].tag==0)
		{
			continue;
		}
		
        Forward forward_tmp;
        for(int j=0; j<rectangle.size();j++)
        {
			if (rectangle[j].tag==0)
			{
				continue;
			}
			
            if(isTriangleRectangleConnect(acuteTriangle[i],rectangle[j]))
            {
                for(int j1=0;j1<rhombus.size();j1++)
                {
                    if(!isRectangleRhombusConnect(rectangle[j],rhombus[j1]))
                    {
						rectangle[j].tag=0;
						acuteTriangle[i].tag=0;
                        forward_tmp.rectangle=rectangle[j];
                        forward_tmp.triangle=acuteTriangle[i];
                        forward.push_back(forward_tmp);
                    }
                }
            }
        }
    }
    return forward;
}

vector<ForwardRight> JunctDec::findForwardRight(vector<Triangle>& acuteTriangle, vector<Rectangle>& rectangle,
        vector<Triangle>& obtuseTriangle, vector<Rhombus>& rhombus)
{
    vector<ForwardRight> forwardRight;
    for(int i=0;i<acuteTriangle.size();i++)
    {
		if (acuteTriangle[i].tag==0)
		{
			continue;
		}
		
        ForwardRight forwardRight_tmp;
        for(int j=0;j<rectangle.size();j++)
        {
			if (rectangle[j].tag==0)
			{
				continue;
			}
			
            if(isTriangleRectangleConnect(acuteTriangle[i],rectangle[j]))
            {
                for(int n1=0;n1<rhombus.size();n1++)
                {
					if (rhombus[n1].tag==0)
					{
						continue;
					}
					
                    if(isRectangleRhombusConnect(rectangle[j],rhombus[n1]))
                    {
                        for(int n2=0;n2<obtuseTriangle.size();n2++)
                        {
							if (obtuseTriangle[n2].tag==0)
							{
								continue;
							}
							
                            if(isTriangleRhombusConnect(obtuseTriangle[n2],rhombus[n1])&&
                                    obtuseTriangle[n2].junct1.location.x>rectangle[j].junct1.location.x)
                        
                            {
                                forwardRight_tmp.acuteTriangle=acuteTriangle[i];
                                forwardRight_tmp.obtuseTriangle=obtuseTriangle[n2];
                                forwardRight_tmp.rectangle=rectangle[j];
                                forwardRight_tmp.rhombus=rhombus[n1];
								acuteTriangle[i].tag=0;
								obtuseTriangle[n2].tag=0;
								rectangle[j].tag=0;
								rhombus[n1].tag=0;

                                forwardRight.push_back(forwardRight_tmp);
                                continue;
                            }
                        }
                    }
                }
            }
        }
    }

    return forwardRight;
}

vector<ForwardLeft> JunctDec::findForwardLeft(vector<Rectangle>& rectangle, vector<Rhombus>& rhombus,
        vector<Triangle>& acuteTriangle, vector<Triangle>& obtuseTriangle)
{
    vector<ForwardLeft> forwardLeft;
    for(int i=0;i<acuteTriangle.size();i++)
    {
		if (acuteTriangle[i].tag==0)
		{
			continue;
		}
		
        ForwardLeft forwardLeft_tmp;
        for(int j=0;j<rectangle.size();j++)
        {
			if (rectangle[j].tag==0)
			{
				continue;
			}
			
            if(isTriangleRectangleConnect(acuteTriangle[i],rectangle[j]))
            {
                for(int n1=0;n1<rhombus.size();n1++)
                {
					if (rhombus[n1].tag==0)
					{
						continue;
					}
					
                    if(isRectangleRhombusConnect(rectangle[j],rhombus[n1]))
                    {
                        for(int n2=0;n2<obtuseTriangle.size();n2++)
                        {
							if (obtuseTriangle[n2].tag==0)
							{
								continue;
							}
							
                            if(isTriangleRhombusConnect(obtuseTriangle[n2],rhombus[n1])&&
                                    obtuseTriangle[n2].junct1.location.x<rectangle[j].junct1.location.x)
                            {
                                forwardLeft_tmp.acuteTriangle=acuteTriangle[i];
                                forwardLeft_tmp.obtuseTriangle=obtuseTriangle[n2];
                                forwardLeft_tmp.rectangle=rectangle[j];
                                forwardLeft_tmp.rhombus=rhombus[n1];

								acuteTriangle[i].tag=0;
								obtuseTriangle[n2].tag=0;
								rectangle[j].tag=0;
								rhombus[n1].tag=0;


                                forwardLeft.push_back(forwardLeft_tmp);
                                continue;
                            }
                        }
                    }
                }
            }
        }
    }

    return forwardLeft;
}

vector<ForwardLeftRight> JunctDec::findForwardLeftRight(vector<Rectangle>& rectangle,vector<Rhombus>& rhombus,
        vector<Triangle>& acuteTriangle, vector<Triangle>& obtuseTriangle)
{
    vector<ForwardLeftRight> forwardLeftRight;
    for(int i=0;i<acuteTriangle.size();i++)
    {
		if (acuteTriangle[i].tag==0)
		{
			continue;
		}
		
        ForwardLeftRight forwardLeftRight_tmp;
        for(int j=0;j<rectangle.size();j++)
        {
			if (rectangle[j].tag==0)
			{
				continue;
			}
			
            if(isTriangleRectangleConnect(acuteTriangle[i],rectangle[j]))
            {
                for(int n1=0;n1<rhombus.size();n1++)
                {
					if (rhombus[n1].tag==0)
					{
						continue;
					}
					
                    if(isRectangleRhombusConnect(rectangle[j],rhombus[n1]))
                    {
                        for(int n2=n1+1;n2<rhombus.size();n2++)
                        {
							if (rhombus[n2].tag==0)
							{
								continue;
							}
							
                            if(isRectangleRhombusConnect(rectangle[j],rhombus[n2]))
                            {
                                for(int n3=0;n3<obtuseTriangle.size();n3++)
                                {
									if (obtuseTriangle[n3].tag==0)
									{
										continue;
									}
									
                                    if(isTriangleRhombusConnect(obtuseTriangle[n3],rhombus[n2]))
                                    {
                                        for(int n4=0;n4<obtuseTriangle.size();n4++)
                                        {
											if (obtuseTriangle[n4].tag==0)
											{
												continue;
											}
											
                                            if(isTriangleRhombusConnect(obtuseTriangle[n4],rhombus[n1]))
                                            {
                                                forwardLeftRight_tmp.acuteTriangle=acuteTriangle[i];
                                                forwardLeftRight_tmp.rectangle=rectangle[j];
                                                forwardLeftRight_tmp.obtuseTriangle1=obtuseTriangle[n3];
                                                forwardLeftRight_tmp.obtuseTriangle2=obtuseTriangle[n4];
                                                forwardLeftRight_tmp.rhombus1=rhombus[n1];
                                                forwardLeftRight_tmp.rhombus2=rhombus[n2];

												acuteTriangle[i].tag=0;
												rectangle[j].tag=0;
												obtuseTriangle[n3].tag=0;
												obtuseTriangle[n4].tag=0;
												rhombus[n1].tag=0;
												rhombus[n2].tag=0;

                                                forwardLeftRight.push_back(forwardLeftRight_tmp);


                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

    }

    return forwardLeftRight;
}

bool JunctDec::isTriangleQuasiRhombusConnect(Triangle& triangle, QuasiRhombus& quasiRhombus)
{
    Ljunct junctAcute=quasiRhombus.acuteJunct;
    Ljunct junctObtuse=abs(junctAcute.location.x-quasiRhombus.obtuseJunct1.location.x)<0.001?
        quasiRhombus.obtuseJunct1:quasiRhombus.obtuseJunct2;
    junctObtuse=abs(junctAcute.location.x-quasiRhombus.obtuseJunct3.location.x)<0.001?
        quasiRhombus.obtuseJunct3:junctObtuse;

    if(isLiesOnTriangle(junctAcute,triangle)&&isLiesOnTriangle(junctObtuse,triangle))
        return true;
    else
        return false;
}

vector<Left> JunctDec::findLeft(vector<Triangle>& obtuseTriangle, vector<QuasiRhombus>& quasiRhombus)
{
    vector<Left> left;
    for(int i=0;i<quasiRhombus.size();i++)
    {
		if (quasiRhombus[i].tag==0)
		{
			continue;
		}
		
        Left left_tmp;
        for(int j=0;j<obtuseTriangle.size();j++)
        {
			if (obtuseTriangle[j].tag==0)
			{
				continue;
			}
			
            if(isTriangleQuasiRhombusConnect(obtuseTriangle[j],quasiRhombus[i])&&
                    obtuseTriangle[j].junct1.location.x<quasiRhombus[i].rightJunct1.location.x)
            {
                left_tmp.obtuseTriangle = obtuseTriangle[j];
                left_tmp.quasiRhombus=quasiRhombus[i];
				obtuseTriangle[j].tag=0;
				quasiRhombus[i].tag=0;
                left.push_back(left_tmp);
            }
        }
    }

    return left;
}

vector<Right> JunctDec::findRight(vector<Triangle>& obtuseTriangle, vector<QuasiRhombus>& quasiRhombus)
{
    vector<Right> right;
    for(int i=0;i<quasiRhombus.size();i++)
    {
		if (quasiRhombus[i].tag==0)
		{
			continue;
		}
		
        Right right_tmp;
        for(int j=0;j<obtuseTriangle.size();j++)
        {
			if (obtuseTriangle[j].tag==0)
			{
				continue;
			}
			
            if(isTriangleQuasiRhombusConnect(obtuseTriangle[j],quasiRhombus[i])&&
                    obtuseTriangle[j].junct1.location.x>quasiRhombus[i].rightJunct1.location.x)
            {
                right_tmp.obtuseTriangle=obtuseTriangle[j];
                right_tmp.quasiRhombus=quasiRhombus[i];
				obtuseTriangle[j].tag=0;
				quasiRhombus[i].tag=0;
                right.push_back(right_tmp);
            }
        }
    }

    return right;
}
/*
vector<Left> JunctDec::findLeft(vector<Triangle>& obtuseTriangle, vector<QuasiRhombus>& quasiRhombus)
{
    vector<Left> left;
    for(int i=0;i<quasiRhombus.size();i++)
    {
        Left left_tmp;
        for(int j=0;j<obtuseTriangle.size();j++)
        {
            if(isTriangleQuasiRhombusConnect(obtuseTriangle[j],quasiRhombus[i]))
            {
                left.obtuseTriangle=obtuseTriangle[j];
                left.quasiRhombus=quasiRhombus[i];
                left.push_back(left_tmp);
            }
        }
    }

    return left;
}
*/

bool JunctDec::classifyRoadMarking(vector<Ljunct>& Jlist, vector<Forward>& forward, vector<ForwardLeft>& forwardLeft,
        vector<ForwardRight>& forwardRight, vector<ForwardLeftRight>& forwardLeftRight,
		vector<Left>& left, vector<Right>& right,vector<BoundingBox>& boundingBox, Mat& img,vector<Triangle>& acuteTriangle,
vector<Triangle>& obtuseTriangle,vector<Rectangle>& rectangle,vector<Ljunct>& acuteJlist,vector<Ljunct>& obtuseJlist,
vector<Ljunct>& rightJlist,vector<Rectangle>& quaDrangle, vector<Rhombus> rhombus, vector<QuasiRhombus> quasiRhombus)
{
    //vector<Ljunct> acuteJlist;
    //vector<Ljunct> obtuseJlist;
    //vector<Ljunct> rightJlist;

    classifyJlist(Jlist,acuteJlist,obtuseJlist,rightJlist);

    cout<<"After classify:\n";
    cout<<"Jlist's size is:\n"<<Jlist.size()<<endl;
    cout<<"acuteJlist's size is: "<<acuteJlist.size()<<endl;
    cout<<"obtuseJlist's size is: "<<obtuseJlist.size()<<endl;
    cout<<"rightJlist's size is: "<<rightJlist.size()<<endl;

    //vector<Triangle> acuteTriangle;
    if (acuteJlist.size()>=3)
    { 
        acuteTriangle=findAcuteTriangle(acuteJlist);
        cout<<"the acute triangles that we have found:\n"<<acuteTriangle.size()<<endl;
		//displayTriangle(acuteTriangle, img);
    }
    //vector<Triangle> obtuseTriangle;
    if(obtuseJlist.size()>=3)
    {
        obtuseTriangle=findObtuseTriangle(acuteJlist,obtuseJlist);
        cout<<"the obtuse triangles that we have found:\n"<<obtuseTriangle.size()<<endl;

    }
	//after detect the acute and obtuse triangle, we have to change the direction first;
    changeDirection(obtuseTriangle, acuteTriangle, rightJlist, acuteJlist, obtuseJlist);


	//vector<Rectangle> quaDrangle;
	
	//we no longer have to detect the rectangle directly;
	if (rightJlist.size()>=4)
	{
		rectangle=findRectangle(rightJlist);
		cout<<"the rectangles that we have found:\n"<<rectangle.size()<<endl;
	}

	changeDirectionAfterRecExtraction(obtuseJlist,acuteJlist,rectangle);

	vector<Ljunct> allJlist;
	for (int i=0; i<rightJlist.size();i++)
	{
		if(rightJlist[i].tag!=0)
			allJlist.push_back(rightJlist[i]);
	}
	for (int i=0; i<acuteJlist.size(); i++)
	{
		if(acuteJlist[i].tag!=0)
			allJlist.push_back(acuteJlist[i]);
	}
	for (int i=0; i<obtuseJlist.size();i++)
	{
		if(obtuseJlist[i].tag!=0)
			allJlist.push_back(obtuseJlist[i]);
	}

	quaDrangle=findQuadrangle(allJlist);

	//we also do not have to detect rhombus;
    //vector<Rhombus> rhombus;

   // rhombus=findRhombus(acuteJlist, obtuseJlist);


	//cout<<"the rhombus that we have detected is: "<<rhombus.size()<<endl;

   // vector<QuasiRhombus> quasiRhombus;
    //quasiRhombus=findQuasiRhombus(acuteJlist, obtuseJlist, rightJlist);

	if (quaDrangle.size()>0)
	{
		for (int i=0; i<quaDrangle.size();i++)
		{
			Rhombus rhombus_tmp;
			rhombus_tmp.acuteJunct1=quaDrangle[i].junct1;
			rhombus_tmp.acuteJunct2=quaDrangle[i].junct2;
			rhombus_tmp.obtuseJunct1=quaDrangle[i].junct3;
			rhombus_tmp.obtuseJunct2=quaDrangle[i].junct4;

			rhombus.push_back(rhombus_tmp);
		}
	}
    if (rhombus.size()>0)
        forward=findForward(acuteTriangle, rectangle, rhombus);
    else
        forward=findForwardWithoutRhombus(acuteTriangle, rectangle);

    cout<<"!!!!!!!the Forward we have found are:\n"<<forward.size()<<endl;
   // vector<BoundingBox> boundingBox;
    for(int i=0; i<forward.size();i++)
    {
        BoundingBox boundingBox_tmp;
        boundingBox_tmp=findBoundingBox(forward[i]);
        boundingBox.push_back(boundingBox_tmp);
    }

    forwardLeft=findForwardLeft(rectangle, rhombus, acuteTriangle, obtuseTriangle);

    forwardRight=findForwardRight(acuteTriangle, rectangle, obtuseTriangle, rhombus);
	cout<<"the forwardRight we have detected is:  "<<forwardRight.size()<<endl;
	if (forwardRight.size()>0)
	{
		for (int i=0; i<forwardRight.size();i++)
		{
			BoundingBox boundingBox_tmp;
			boundingBox_tmp=findBoundingBoxForwardRight(forwardRight[i]);
			boundingBox.push_back(boundingBox_tmp);
		}	
	}
	

    forwardLeftRight=findForwardLeftRight(rectangle, rhombus, acuteTriangle, obtuseTriangle);

	cout<<"the forwardLeftRight we have detected is:  "<<forwardLeftRight.size()<<endl;
	if (forwardLeftRight.size()>0)
	{
		for (int i=0; i<forwardLeftRight.size();i++)
		{
			BoundingBox boundingBox_tmp;
			boundingBox_tmp=findBoundingBoxForwardLeftRight(forwardLeftRight[i]);
			boundingBox.push_back(boundingBox_tmp);
		}	
	}
	
	quasiRhombus=findHexagon(allJlist);

    left=findLeft(obtuseTriangle, quasiRhombus);

	cout<<"the Left we have detected is:  "<<left.size()<<endl;
	if (forwardLeftRight.size()>0)
	{
		for (int i=0; i<left.size();i++)
		{
			BoundingBox boundingBox_tmp;
			boundingBox_tmp=findBoundingBoxLeft(left[i]);
			boundingBox.push_back(boundingBox_tmp);
		}	
	}

    right=findRight(obtuseTriangle, quasiRhombus);

	cout<<"the Right we have detected is:  "<<right.size()<<endl;
	if (right.size()>0)
	{
		for (int i=0; i<right.size();i++)
		{
			BoundingBox boundingBox_tmp;
			boundingBox_tmp=findBoundingBoxRight(right[i]);
			boundingBox.push_back(boundingBox_tmp);
		}	
	}
   
    if(forward.size()+forwardLeft.size()+forwardRight.size()+forwardLeftRight.size()+
            left.size()+right.size()>0)
        return true;
    else
        return false;
  
}

vector<Forward>  JunctDec::findForwardWithoutRhombus(vector<Triangle>& acuteTriangle, vector<Rectangle>& rectangle)
{
	vector<Forward> forward;
	for(int i=0; i<acuteTriangle.size();i++)
	{
		Forward forward_tmp;
		for(int j=0; j<rectangle.size();j++)
		{
			if(isTriangleRectangleConnect(acuteTriangle[i],rectangle[j]))
			{

				forward_tmp.rectangle=rectangle[j];
				forward_tmp.triangle=acuteTriangle[i];
				forward.push_back(forward_tmp);
			}
		}
	}
	return forward;
}

template <typename T>
BoundingBox& JunctDec::findBoundingBox(const T& roadSign)
{
    vector<Ljunct> Jlist;
    BoundingBox boundingBox;
    float top=0.0;
    float bottom=0.0;
    float left=0.0;
    float right=0.0;

    if(typeid(T).name()==typeid(Forward).name())
    {
        Jlist.push_back(roadSign.triangle.junct1);
        Jlist.push_back(roadSign.triangle.junct2);
        Jlist.push_back(roadSign.triangle.junct3);
        Jlist.push_back(roadSign.rectangle.junct1);
        Jlist.push_back(roadSign.rectangle.junct2);
        Jlist.push_back(roadSign.rectangle.junct3);
        Jlist.push_back(roadSign.rectangle.junct4);
        top=Jlist[0].location.y;
        bottom=Jlist[0].location.y;
        left=Jlist[0].location.x;
        right=Jlist[0].location.x;
        for(int i=1;i<Jlist.size();i++)
        {
            if(Jlist[i].location.y<top)
                top=Jlist[i].location.y;
            if(Jlist[i].location.y>bottom)
                bottom=Jlist[i].location.y;
            if(Jlist[i].location.x<left)
                left=Jlist[i].location.x;
            if(Jlist[i].location.x>right)
                right=Jlist[i].location.x;
        }
        boundingBox.top=top;
        boundingBox.bottom=bottom;
        boundingBox.left=left;
        boundingBox.right=right;


        cout<<boundingBox.top<<endl;
        cout<<boundingBox.bottom<<endl;
        cout<<boundingBox.left<<endl;
        cout<<boundingBox.right<<endl;

        
    }
	return boundingBox;
}

BoundingBox& JunctDec::findBoundingBoxForwardLeft(const ForwardLeft& forwardLeft)
{
    vector<Ljunct> Jlist;
    BoundingBox boundingBox;
    float top=0.0;
    float bottom=0.0;
    float left=0.0;
    float right=0.0;

    Jlist.push_back(forwardLeft.acuteTriangle.junct1);
    Jlist.push_back(forwardLeft.acuteTriangle.junct2);
    Jlist.push_back(forwardLeft.acuteTriangle.junct3);
    Jlist.push_back(forwardLeft.obtuseTriangle.junct1);
    Jlist.push_back(forwardLeft.obtuseTriangle.junct2);
    Jlist.push_back(forwardLeft.obtuseTriangle.junct3);
    Jlist.push_back(forwardLeft.rectangle.junct1); 
    Jlist.push_back(forwardLeft.rectangle.junct2);
    Jlist.push_back(forwardLeft.rectangle.junct3);
    Jlist.push_back(forwardLeft.rectangle.junct4);

    Jlist.push_back(forwardLeft.rhombus.acuteJunct1);
    Jlist.push_back(forwardLeft.rhombus.acuteJunct2);
    Jlist.push_back(forwardLeft.rhombus.obtuseJunct1);
    Jlist.push_back(forwardLeft.rhombus.obtuseJunct2);
    top=Jlist[0].location.y;
    bottom=Jlist[0].location.y;
    left=Jlist[0].location.x;
    right=Jlist[0].location.x;
    for(int i=1;i<Jlist.size();i++)
    {
        if(Jlist[i].location.y<top)
            top=Jlist[i].location.y;
        if(Jlist[i].location.y>bottom)
            bottom=Jlist[i].location.y;
        if(Jlist[i].location.x<left)
            left=Jlist[i].location.x;
        if(Jlist[i].location.x>right)
            right=Jlist[i].location.x;
    }
    boundingBox.top=top;
    boundingBox.bottom=bottom;
    boundingBox.left=left;
    boundingBox.right=right;


    cout<<boundingBox.top<<endl;
    cout<<boundingBox.bottom<<endl;
    cout<<boundingBox.left<<endl;
    cout<<boundingBox.right<<endl;

    return boundingBox;

}

BoundingBox& JunctDec::findBoundingBoxForwardRight(const ForwardRight& forwardRight)
{
    vector<Ljunct> Jlist;
    BoundingBox boundingBox;
    float top=0.0;
    float bottom=0.0;
    float left=0.0;
    float right=0.0;

    Jlist.push_back(forwardRight.acuteTriangle.junct1);
    Jlist.push_back(forwardRight.acuteTriangle.junct2);
    Jlist.push_back(forwardRight.acuteTriangle.junct3);
    Jlist.push_back(forwardRight.obtuseTriangle.junct1);
    Jlist.push_back(forwardRight.obtuseTriangle.junct2);
    Jlist.push_back(forwardRight.obtuseTriangle.junct3);
    Jlist.push_back(forwardRight.rectangle.junct1); 
    Jlist.push_back(forwardRight.rectangle.junct2);
    Jlist.push_back(forwardRight.rectangle.junct3);
    Jlist.push_back(forwardRight.rectangle.junct4);

    Jlist.push_back(forwardRight.rhombus.acuteJunct1);
    Jlist.push_back(forwardRight.rhombus.acuteJunct2);
    Jlist.push_back(forwardRight.rhombus.obtuseJunct1);
    Jlist.push_back(forwardRight.rhombus.obtuseJunct2);
    top=Jlist[0].location.y;
    bottom=Jlist[0].location.y;
    left=Jlist[0].location.x;
    right=Jlist[0].location.x;
    for(int i=1;i<Jlist.size();i++)
    {
        if(Jlist[i].location.y<top)
            top=Jlist[i].location.y;
        if(Jlist[i].location.y>bottom)
            bottom=Jlist[i].location.y;
        if(Jlist[i].location.x<left)
            left=Jlist[i].location.x;
        if(Jlist[i].location.x>right)
            right=Jlist[i].location.x;
    }
    boundingBox.top=top;
    boundingBox.bottom=bottom;
    boundingBox.left=left;
    boundingBox.right=right;


    cout<<boundingBox.top<<endl;
    cout<<boundingBox.bottom<<endl;
    cout<<boundingBox.left<<endl;
    cout<<boundingBox.right<<endl;

    return boundingBox;
}

BoundingBox& JunctDec::findBoundingBoxForwardLeftRight(const ForwardLeftRight& forwardLeftRight)
{
    vector<Ljunct> Jlist;
    BoundingBox boundingBox;
    float top=0.0;
    float bottom=0.0;
    float left=0.0;
    float right=0.0;

    Jlist.push_back(forwardLeftRight.acuteTriangle.junct1);
    Jlist.push_back(forwardLeftRight.acuteTriangle.junct2);
    Jlist.push_back(forwardLeftRight.acuteTriangle.junct3);
    Jlist.push_back(forwardLeftRight.obtuseTriangle1.junct1);
    Jlist.push_back(forwardLeftRight.obtuseTriangle1.junct2);
    Jlist.push_back(forwardLeftRight.obtuseTriangle1.junct3);
    Jlist.push_back(forwardLeftRight.obtuseTriangle2.junct1);
    Jlist.push_back(forwardLeftRight.obtuseTriangle2.junct2);
    Jlist.push_back(forwardLeftRight.obtuseTriangle2.junct3);
    Jlist.push_back(forwardLeftRight.rectangle.junct1); 
    Jlist.push_back(forwardLeftRight.rectangle.junct2);
    Jlist.push_back(forwardLeftRight.rectangle.junct3);
    Jlist.push_back(forwardLeftRight.rectangle.junct4);

    Jlist.push_back(forwardLeftRight.rhombus1.acuteJunct1);
    Jlist.push_back(forwardLeftRight.rhombus1.acuteJunct2);
    Jlist.push_back(forwardLeftRight.rhombus1.obtuseJunct1);
    Jlist.push_back(forwardLeftRight.rhombus1.obtuseJunct2);
    Jlist.push_back(forwardLeftRight.rhombus2.acuteJunct1);
    Jlist.push_back(forwardLeftRight.rhombus2.acuteJunct2);
    Jlist.push_back(forwardLeftRight.rhombus2.obtuseJunct1);
    Jlist.push_back(forwardLeftRight.rhombus2.obtuseJunct2);

    top=Jlist[0].location.y;
    bottom=Jlist[0].location.y;
    left=Jlist[0].location.x;
    right=Jlist[0].location.x;
    for(int i=1;i<Jlist.size();i++)
    {
        if(Jlist[i].location.y<top)
            top=Jlist[i].location.y;
        if(Jlist[i].location.y>bottom)
            bottom=Jlist[i].location.y;
        if(Jlist[i].location.x<left)
            left=Jlist[i].location.x;
        if(Jlist[i].location.x>right)
            right=Jlist[i].location.x;
    }
    boundingBox.top=top;
    boundingBox.bottom=bottom;
    boundingBox.left=left;
    boundingBox.right=right;


    cout<<boundingBox.top<<endl;
    cout<<boundingBox.bottom<<endl;
    cout<<boundingBox.left<<endl;
    cout<<boundingBox.right<<endl;

    return boundingBox;
}

BoundingBox& JunctDec::findBoundingBoxLeft(const Left& left)
{
    vector<Ljunct> Jlist;
    BoundingBox boundingBox;
    float top=0.0;
    float bottom=0.0;
    float fleft=0.0;
    float right=0.0;

    Jlist.push_back(left.obtuseTriangle.junct1);
    Jlist.push_back(left.obtuseTriangle.junct2);
    Jlist.push_back(left.obtuseTriangle.junct3);

    Jlist.push_back(left.quasiRhombus.rightJunct1);
    Jlist.push_back(left.quasiRhombus.rightJunct2);
    Jlist.push_back(left.quasiRhombus.obtuseJunct1);
    Jlist.push_back(left.quasiRhombus.obtuseJunct2);
    Jlist.push_back(left.quasiRhombus.obtuseJunct3);
    Jlist.push_back(left.quasiRhombus.acuteJunct);
    top=Jlist[0].location.y;
    bottom=Jlist[0].location.y;
    fleft=Jlist[0].location.x;
    right=Jlist[0].location.x;
    for(int i=1;i<Jlist.size();i++)
    {
        if(Jlist[i].location.y<top)
            top=Jlist[i].location.y;
        if(Jlist[i].location.y>bottom)
            bottom=Jlist[i].location.y;
        if(Jlist[i].location.x<fleft)
            fleft=Jlist[i].location.x;
        if(Jlist[i].location.x>right)
            right=Jlist[i].location.x;
    }
    boundingBox.top=top;
    boundingBox.bottom=bottom;
    boundingBox.left=fleft;
    boundingBox.right=right;


    cout<<boundingBox.top<<endl;
    cout<<boundingBox.bottom<<endl;
    cout<<boundingBox.left<<endl;
    cout<<boundingBox.right<<endl;

    return boundingBox;
}

BoundingBox& JunctDec::findBoundingBoxRight(const Right& right)
{
    vector<Ljunct> Jlist;
    BoundingBox boundingBox;
    float top=0.0;
    float bottom=0.0;
    float left=0.0;
    float fright=0.0;

    Jlist.push_back(right.obtuseTriangle.junct1);
    Jlist.push_back(right.obtuseTriangle.junct2);
    Jlist.push_back(right.obtuseTriangle.junct3);

    Jlist.push_back(right.quasiRhombus.rightJunct1);
    Jlist.push_back(right.quasiRhombus.rightJunct2);
    Jlist.push_back(right.quasiRhombus.obtuseJunct1);
    Jlist.push_back(right.quasiRhombus.obtuseJunct2);
    Jlist.push_back(right.quasiRhombus.obtuseJunct3);
    Jlist.push_back(right.quasiRhombus.acuteJunct);
    top=Jlist[0].location.y;
    bottom=Jlist[0].location.y;
    left=Jlist[0].location.x;
    fright=Jlist[0].location.x;
    for(int i=1;i<Jlist.size();i++)
    {
        if(Jlist[i].location.y<top)
            top=Jlist[i].location.y;
        if(Jlist[i].location.y>bottom)
            bottom=Jlist[i].location.y;
        if(Jlist[i].location.x<left)
            left=Jlist[i].location.x;
        if(Jlist[i].location.x>fright)
            fright=Jlist[i].location.x;
    }
    boundingBox.top=top;
    boundingBox.bottom=bottom;
    boundingBox.left=left;
    boundingBox.right=fright;


    cout<<boundingBox.top<<endl;
    cout<<boundingBox.bottom<<endl;
    cout<<boundingBox.left<<endl;
    cout<<boundingBox.right<<endl;

    return boundingBox;
}


vector<QuasiRhombus> JunctDec::findHexagon(vector<Ljunct> allJlist)
{
	Ljunct Ljunct_initialize;
	Ljunct_initialize.class_id=2;
	Ljunct_initialize.tag=1;
	Ljunct_initialize.location=Point2f(0,0);
	Ljunct_initialize.branch[0]=0;
	Ljunct_initialize.branch[1]=0;
	Ljunct_initialize.strength[0]=0;
	Ljunct_initialize.strength[1]=0;
	vector<QuasiRhombus> Hexagon;
	for (int i=0; i<allJlist.size();i++)
	{
		QuasiRhombus Hexagon_tmp;
		if (allJlist[i].tag==0)
		{
			continue;
		}
		int branch1=4;
		Ljunct junction1=Ljunct_initialize;
		if (findNextJunforOneBranchWithReturn(allJlist,allJlist[i].location,allJlist[i].branch[0],junction1,&branch1)
			&& allJlist[i].location.x != junction1.location.x && allJlist[i].location.y != junction1.location.y)
		{
			double orientation1=branch1==0?junction1.branch[1]:junction1.branch[0];
			int branch2=4;
			Ljunct junction2=Ljunct_initialize;
			if (findNextJunforOneBranchWithReturn(allJlist,junction1.location,orientation1,junction2,&branch2)
				&& junction1.location.x != junction2.location.x && junction1.location.y != junction2.location.y)
			{
				double orientation2=branch2==0?junction2.branch[1]:junction2.branch[0];
				int branch3=4;
				Ljunct junction3=Ljunct_initialize;
				if (findNextJunforOneBranchWithReturn(allJlist,junction2.location,orientation2,junction3,&branch3)
					&& junction2.location.x != junction3.location.x && junction2.location.y != junction3.location.y)
				{
					double orientation3=branch3==0?junction3.branch[1]:junction3.branch[0];
					int branch4=4;
					Ljunct junction4=Ljunct_initialize;
					if (findNextJunforOneBranchWithReturn(allJlist,junction3.location,orientation3,junction4,&branch4)
						&& junction4.location.x == allJlist[i].location.x && junction4.location.y ==allJlist[i].location.y)
					{
						double orientation4=branch4==0?junction4.branch[1]:junction4.branch[0];
						int branch5=4;
						Ljunct junction5=Ljunct_initialize;
						if (findNextJunforOneBranchWithReturn(allJlist,junction4.location,orientation4,junction5,&branch5)
							&& junction5.location.x == allJlist[i].location.x && junction5.location.y ==allJlist[i].location.y)
						{
							double orientation5=branch5==0?junction5.branch[1]:junction5.branch[0];
							int branch6=4;
							Ljunct junction6=Ljunct_initialize;
							if (findNextJunforOneBranchWithReturn(allJlist,junction5.location,orientation5,junction6,&branch6)
								&& junction6.location.x == allJlist[i].location.x && junction6.location.y ==allJlist[i].location.y)
							{
								Hexagon_tmp.rightJunct1=junction1;
								Hexagon_tmp.rightJunct2=junction2;
								Hexagon_tmp.obtuseJunct1=junction3;
								Hexagon_tmp.obtuseJunct2=junction4;
								Hexagon_tmp.obtuseJunct3=junction5;
								Hexagon_tmp.acuteJunct=junction6;
								Hexagon.push_back(Hexagon_tmp);
								for (vector<Ljunct>::iterator t=allJlist.begin();t!=allJlist.end();t++)
								{
									if (t->location.x==junction1.location.x&&t->location.y==junction1.location.y)
									{
										t->tag=0;
									}
									if (t->location.x==junction2.location.x&&t->location.y==junction2.location.y)
									{
										t->tag=0;
									}
									if (t->location.x==junction3.location.x&&t->location.y==junction3.location.y)
									{
										t->tag=0;
									}
									if (t->location.x==junction4.location.x&&t->location.y==junction4.location.y)
									{
										t->tag=0;
									}
									if (t->location.x==junction5.location.x&&t->location.y==junction5.location.y)
									{
										t->tag=0;
									}
									if (t->location.x==junction6.location.x&&t->location.y==junction6.location.y)
									{
										t->tag=0;
									}

								}
							}
							
						}

					}
				}
			}

		}

	}

	return Hexagon;
}

//encode the junction string with anti-clockwise order;
vector<string> JunctDec::encodingLjunct(vector<Ljunct>& Jlist)
{
	vector<string> code_string;
	for (int i=0; i<Jlist.size(); i++)
	{
		if (Jlist[i].tag==0)
		{
			continue;
		}
		vector<Ljunct> lJunct_tmp;
		lJunct_tmp.push_back(Jlist[i]);
		Ljunct Ljunct_tmp_begin=Jlist[i];
		Ljunct Ljunct_tmp_end;
		
		int branch;
		if (findNextJunforOneBranchWithReturn(Jlist,Jlist[i].location,Jlist[i].branch[0],Ljunct_tmp_end,&branch))
		{
			int branch_tmp=branch==0?1:0;
			lJunct_tmp.push_back(Ljunct_tmp_end);
			Ljunct_tmp_begin=Ljunct_tmp_end;
			Ljunct_tmp_end.location.x=0;
			Ljunct_tmp_end.location.y=0;
			for (vector<Ljunct>::iterator t=Jlist.begin();t!=Jlist.end();t++)
			{
				if (t->location.x==Ljunct_tmp_begin.location.x&&t->location.y==Ljunct_tmp_begin.location.y)
				{
					t->tag=0;
				}
			}
			while (findNextJunforOneBranchWithReturn(Jlist,Ljunct_tmp_begin.location,Ljunct_tmp_begin.branch[branch_tmp],Ljunct_tmp_end,&branch)
				&& Jlist[i].location.x != Ljunct_tmp_end.location.x 
				&& Jlist[i].location.y != Ljunct_tmp_end.location.y
				&& Ljunct_tmp_end.location.x != 0 
				&& Ljunct_tmp_end.location.y != 0)
			{
				lJunct_tmp.push_back(Ljunct_tmp_end);
				Ljunct_tmp_begin=Ljunct_tmp_end;
				Ljunct_tmp_end.location.x=0;
				Ljunct_tmp_end.location.y=0;
				branch_tmp=branch==0?1:0;
				for (vector<Ljunct>::iterator t=Jlist.begin();t!=Jlist.end();t++)
				{
					if (t->location.x==Ljunct_tmp_begin.location.x&&t->location.y==Ljunct_tmp_begin.location.y)
					{
						t->tag=0;
					}
				}
			}
		}
		if (lJunct_tmp.size()>=7)
		{
			//find the top junction's location;
			Ljunct top_Ljunct;
			int top_location;
			for (int j=0; j<lJunct_tmp.size(); j++)
			{
				if (j ==0)
				{
					top_Ljunct=lJunct_tmp[j];
					top_location=j;
				}
				if (lJunct_tmp[j].location.y < top_Ljunct.location.y)
				{
					top_Ljunct=lJunct_tmp[j];
					top_location=j;
				}
			}
			bool isAntiClockwise=true;
			if (top_location<lJunct_tmp.size()-1)
			{
				if (lJunct_tmp[top_location].location.x > lJunct_tmp[top_location+1].location.x)
				{
					isAntiClockwise=true;
				}
				else
					isAntiClockwise=false;
			}
			else
			{
				if (lJunct_tmp[top_location].location.x > lJunct_tmp[0].location.x)
				{
					isAntiClockwise=true;
				}
				else
					isAntiClockwise=false;
			}


			string str;
			for (int j=top_location; j<lJunct_tmp.size(); j++)
			{
				float angle=abs(lJunct_tmp[j].branch[0]-lJunct_tmp[j].branch[1])>PI?
					2*PI-abs(lJunct_tmp[j].branch[0]-lJunct_tmp[j].branch[1]):
				abs(lJunct_tmp[j].branch[0]-lJunct_tmp[j].branch[1]);


				if (angle<=PI/3)
				{
					stringstream s;
					s<<0;
					str=str+s.str();
				}
				if (angle > PI*17/36 && angle < PI*19/36)
				{
					stringstream s;
					s<<1;
					str=str+s.str();
				}
				if (angle > 3*PI/4)
				{
					stringstream s;
					s<<2;
					str=str+s.str();
				}
				if( (angle > PI/3 && angle <=17*PI/36) || (angle >= 19*PI/36 && angle <= 3*PI/4) )
				{
					stringstream s;
					s<<3;
					str=str+s.str();
				}
			}

			for (int j=0; j<top_location; j++)
			{
				float angle=abs(lJunct_tmp[j].branch[0]-lJunct_tmp[j].branch[1])>PI?
					2*PI-abs(lJunct_tmp[j].branch[0]-lJunct_tmp[j].branch[1]):
				abs(lJunct_tmp[j].branch[0]-lJunct_tmp[j].branch[1]);


				if (angle<=PI/3)
				{
					stringstream s;
					s<<0;
					str=str+s.str();
				}
				if (angle > PI*17/36 && angle < PI*19/36)
				{
					stringstream s;
					s<<1;
					str=str+s.str();
				}
				if (angle > 3*PI/4)
				{
					stringstream s;
					s<<2;
					str=str+s.str();
				}
				if( (angle > PI/3 && angle <=17*PI/36) || (angle >= 19*PI/36 && angle <= 3*PI/4) )
				{
					stringstream s;
					s<<3;
					str=str+s.str();
				}
			}
			if (isAntiClockwise)
			{
				code_string.push_back(str);
			}
			else
			{
				string str_new;
				str_new.push_back(str[0]);
				for (int k=0; k<str.size()-1; k++)
				{
					str_new.push_back(str[str.size()-k-1]);
				}
				code_string.push_back(str_new);
			}
		}
		Jlist[i].tag=0;
	}

	return code_string;
}
