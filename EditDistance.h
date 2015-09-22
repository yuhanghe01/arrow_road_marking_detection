#ifndef EDITDISTANCE_H_
#define EDITDISTANCE_H_

#include <iostream>
#include <string>
#include "EditDistance.h"



double min(double a, double b)
{
	return a < b ? a : b;
}

double editAngle(std::string str1, std::string str2)
{
	int max1 = str1.size();
	int max2 = str2.size();

	double **ptr = new double*[max1 + 1];
	for(int i = 0; i < max1 + 1 ;i++)
	{
		ptr[i] = new double[max2 + 1];
	}

	for(int i = 0 ;i < max1 + 1 ;i++)
	{
		ptr[i][0] = i*1;
	}

	for(int i = 0 ;i < max2 + 1;i++)
	{
		ptr[0][i] = i*2;
	}

	for(int i = 1 ;i < max1 + 1 ;i++)
	{
		for(int j = 1 ;j< max2 + 1; j++)
		{
			double d;
			double temp = min(ptr[i-1][j] +1 , ptr[i][j-1] + 2);
			if(str1[i-1] == str2[j-1])
			{
				d = 0 ;
			}
			else 
			{
				if (str1[i-1]=='3'||str2[j-1]=='3')
				{
					d=1;
				}
				else
					d = 2;
			}
			ptr[i][j] = min(temp, ptr[i-1][j-1] + d);
		}
	}

	double dis = ptr[max1][max2];

	for(int i = 0; i < max1 + 1; i++)
	{
		delete[] ptr[i];
		ptr[i] = NULL;
	}

	delete[] ptr;
	ptr = NULL;

	return dis;
}



double editLocation(std::string str1, std::string str2)
{
	int max1 = str1.size();
	int max2 = str2.size();

	double **ptr = new double*[max1 + 1];
	for(int i = 0; i < max1 + 1 ;i++)
	{
		ptr[i] = new double[max2 + 1];
	}

	for(int i = 0 ;i < max1 + 1 ;i++)
	{
		ptr[i][0] = i*1;
	}

	for(int i = 0 ;i < max2 + 1;i++)
	{
		ptr[0][i] = i*2;
	}

	for(int i = 1 ;i < max1 + 1 ;i++)
	{
		for(int j = 1 ;j< max2 + 1; j++)
		{
			double d;
			double temp = min(ptr[i-1][j] +1 , ptr[i][j-1] + 2);
			if(str1[i-1] == str2[j-1])
			{
				d = 0 ;
			}
			else 
			{
				if (str1[i-1]=='2'||str2[j-1]=='2')
				{
					d=1;
				}
				else
					d = 2;
			}
			ptr[i][j] = min(temp, ptr[i-1][j-1] + d);
		}
	}

	double dis = ptr[max1][max2];
        if(max1-max2 >= 3 && max2 <= 12)
            dis += 2;

	for(int i = 0; i < max1 + 1; i++)
	{
		delete[] ptr[i];
		ptr[i] = NULL;
	}

	delete[] ptr;
	ptr = NULL;

	return dis;
}
double edittotal(string str1, string str2)
{
	int max1 = str1.size();
	int max2 = str2.size();
	double **ptr = new double*[max1 + 1];
	for(int i = 0; i < max1 + 1 ;i++)
	{
		ptr[i] = new double[max2 + 1];
	}

	for(int i = 0 ;i < max1 + 1 ;i++)
	{
		ptr[i][0] = i*1;
	}

	for(int i = 0 ;i < max2 + 1;i++)
	{
		ptr[0][i] = i*2;
	}

	for(int i = 1 ;i < max1 + 1 ;i++)
	{
		for(int j = 1 ;j< max2 + 1; j++)
		{
			double d;
			double temp = min(ptr[i-1][j] +1 , ptr[i][j-1] + 2);
			if(str1[i-1] == str2[j-1])
			{
				d = 0 ;
			}
			else 
			{
				if (((str1[i-1]=='0'||str1[i-1]=='1'||str1[i-1]=='2')&&(str2[j-1]=='0'||str2[j-1]=='1'||str2[j-1]=='2'))||((str1[i-1]=='5'||str1[i-1]=='6'||str1[i-1]=='7')&&(str2[j-1]=='5'||str2[j-1]=='6'||str2[j-1]=='7')))
					d=2;
				if((str1[i-1]=='0'||str1[i-1]=='1'||str1[i-1]=='2')&&str2[j-1]=='3'||(str2[j-1]=='0'||str2[j-1]=='1'||str2[j-1]=='2')&&str1[i-1]=='3'||(str1[i-1]=='5'||str1[i-1]=='6'||str1[i-1]=='7')&&str2[j-1]=='8'||(str2[j-1]=='5'||str2[j-1]=='6'||str2[j-1]=='7')&&str1[i-1]=='8')
					d=1;
				if(str1[i-1]=='4'||str2[j-1]=='4')
					d=1;
				if(((str1[i-1]=='0'||str1[i-1]=='1'||str1[i-1]=='2'||str1[i-1]=='3')&&(str2[j-1]=='5'||str2[j-1]=='6'||str2[j-1]=='7'||str2[j-1]=='8'))||((str2[j-1]=='0'||str2[j-1]=='1'||str2[j-1]=='2'||str2[j-1]=='3')&&(str1[i-1]=='5'||str1[i-1]=='6'||str1[i-1]=='7'||str1[i-1]=='8')))
					d=3;
			}
			ptr[i][j] = min(temp, ptr[i-1][j-1] + d);
		}
	}

	double dis = ptr[max1][max2];
        if(max1-max2 >=3 && max2 <= 12)
            dis += 2;
	for(int i = 0; i < max1 + 1; i++)
	{
		delete[] ptr[i];
		ptr[i] = NULL;
	}

	delete[] ptr;
	ptr = NULL;

	return dis;
}

string changet(string s1,string s2,int k1)
{
	char a[100],b[100],c[100];
	int num_3=0;
        int num_2=0;
	 strncpy(a,s1.c_str(),s1.length());  
	 strncpy(b,s2.c_str(),s2.length()); 
	 //////////test version////////////
	 for (int i=0;i<k1;i++)
	 {
		 if (b[i]=='3' || b[i]=='2')
                 {
                     num_2++;
                     if(b[i]=='3')
			 num_3++;
                 }
	 }
         double k2=double(num_3/k1);
	 if ((num_3>=4&&k1<=6)||((k2>=7.0/9.0)&& k1>8) || (double)num_3/k1>4.0/5.0 && k1<8)
	 {
		 return "false";
	 }
	 /*else
	 {*/
		 for (int i=0;i<k1;i++)
		 {
			 if (b[i]=='3')
			 {
				 b[i]='2';
			 }
		 }
    // }
	 //////////////////
	for(int i=0;i<k1;i++)
	{ 
		if(b[i]=='0')
		{
			c[i]=a[i]; 
		}
		if(b[i]=='1')
		{
			if(a[i]=='0')
				c[i]='5';
			if(a[i]=='1')
				c[i]='6';
			if(a[i]=='2')
				c[i]='7';
			if(a[i]=='3')
				c[i]='8';            
		}
		if(b[i]=='2')
			c[i]='4';
	}    
	string s(&c[0],&c[k1]);
	return s;
}

string changel(string s1,int k1)
{
	char a[100];
	strncpy(a,s1.c_str(),s1.length());  
	for (int i=0;i<k1;i++)
	{
		if (a[i]=='3')
		{
			a[i]='2';
		}
	}
    string s(&a[0],&a[k1]);
	return s;
}
#endif
