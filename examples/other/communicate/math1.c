#include <string.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#define PI 3.1415926
static unsigned long next=1;


double Myfmin(double a,double b){
    if (a>b){
        return b;
    }else{
        return a;
    }
}

double Myfmax(double a,double b){
    if(a>b){
        return a;
    }else{
        return b;
    }
}

double Mypow(double a,int n)
{
    if(n<0) return 1/pow(a,-n);
    double res = 1.0;
    while(n)
    {
        if(n&1) res *= a;
        a *= a;
        n >>= 1;
    }
    return res;
}

double Mysin(double x)
{
    double fl = 1;
    if(x>2*PI || x<-2*PI) x -= (int)(x/(2*PI))*2*PI;
    if(x>PI) x -= 2*PI;
    if(x<-PI) x += 2*PI;
    if(x>PI/2)
    {
        x -= PI;
        fl *= -1;
    }
    if(x<-PI/2)
    {
        x += PI;
        fl *= -1;
    }
    if(x>PI/4) return cos(PI/2-x);
    else return fl*(x - pow(x,3)/6 + pow(x,5)/120 - pow(x,7)/5040 +pow(x,9)/362880);
}

double Mycos(double x)
{
    double fl = 1;
    if(x>2*PI || x<-2*PI) x -= (int)(x/(2*PI))*2*PI;
    if(x>PI) x -= 2*PI;
    if(x<-PI) x += 2*PI;
    if(x>PI/2)
    {
        x -= PI;
        fl *= -1;
    }
    if(x<-PI/2)
    {
        x += PI;
        fl *= -1;
    }
    if(x>PI/4) return sin(PI/2-x);
    else return fl*(1 - pow(x,2)/2 + pow(x,4)/24 - pow(x,6)/720 + pow(x,8)/40320);
}

int Myrand(){
    next=next*1103515245+12345;
    return((unsigned)(next/65536)%32768);
}