#include "firstApproach.h"

// return a list of possible movement according to ax1. [0] if none
void check_ax1(float * data, int length, uint8_t * output)
{

    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<0.1 && min>-0.1)
        output[0] = 1;
    if (max<0.25 && min>-1)
        output[1] = 1;
    if (max<0.25 && min>-2)
        output[2] = 1;
    if (max<0.25 && min>-2)
        output[3] = 1;
    if (max<0.05 && min>-0.15)
        output[4] = 1;
    if (max<0.1 && min>-2.25)
        output[5] = 1;
}



// return a list of possible movement according to ax2. [0] if none
void check_ax2(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<0 && min>-2.25)
        output[0] = 1;
    if (max<-0 && min>-2)
        output[1] = 1;
    if (max<0.5 && min>-2.1)
        output[2] = 1;
    if (max<0.2 && min>-2)
        output[3] = 1;
    if (max<-0.6 && min>-1.25)
        output[4] = 1;
    if (max<-1.3 && min>-2.1)
        output[5] = 1;

}


// return a list of possible movement according to Gy2. [0] if none
void check_gy2(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<175 && min>-200)
        output[0] = 1;
    if (max<100 && min>-150)
        output[1] = 1;
    if (max<225 && min>-200)
        output[2] = 1;
    if (max<100 && min>-100)
        output[3] = 1;
    if (max<60 && min>-60)
        output[4] = 1;
    if (max<100 && min>-100)
        output[5] = 1;
    
}


// return a list of possible movement according to Gy2. [0] if none
void check_gy1(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<20 && min>-30)
        output[0] = 1;
    if (max<40 && min>-60)
        output[1] = 1;
    if (max<150 && min>-150)
        output[2] = 1;
    if (max<85 && min>-125)
        output[3] = 1;
    if (max<35 && min>-40)
        output[4] = 1;
    if (max<100 && min>-100)
        output[5] = 1;

}
    


void check_gz1(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<40 && min>-30)
        output[0] = 1;
    if (max<150 && min>-150)
        output[1] = 1;
    if (max<125 && min>-125)
        output[2] = 1;
    if (max<150 && min>-150)
        output[3] = 1;
    if (max<30 && min>-30)
        output[4] = 1;
    if (max<150 && min>-175)
        output[5] = 1;
    
}



void check_gz2(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<120 && min>-150)
        output[0] = 1;
    if (max<200 && min>-150)
        output[1] = 1;
    if (max<100 && min>-100)
        output[2] = 1;
    if (max<175 && min>-175)
        output[3] = 1;
    if (max<160 && min>-160)
        output[4] = 1;
    if (max<150 && min>-150)
        output[5] = 1;

}
    


void check_gx1(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<30 && min>-25)
        output[0] = 1;
    if (max<80 && min>-125)
        output[1] = 1;
    if (max<60 && min>-60)
        output[2] = 1;
    if (max<70 && min>-70)
        output[3] = 1;
    if (max<95 && min>-95)
        output[4] = 1;
    if (max<95 && min>-95)
        output[5] = 1;

}
    



void check_gx2(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<90 && min>-80)
        output[0] = 1;
    if (max<100 && min>-150)
        output[1] = 1;
    if (max<100 && min>-70)
        output[2] = 1;
    if (max<130 && min>-130)
        output[3] = 1;
    if (max<35 && min>-40)
        output[4] = 1;
    if (max<120 && min>-120)
        output[5] = 1;

}
    


void check_ay1(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<0.6 && min>0.1)
        output[0] = 1;
    if (max<1 && min>-0.5)
        output[1] = 1;
    if (max<0.7 && min>-0.15)
        output[2] = 1;
    if (max<1 && min>0)
        output[3] = 1;
    if (max<0.5 && min>0.12)
        output[4] = 1;
    if (max<1.2 && min>-0.4)
        output[5] = 1;
}
    


void check_ay2(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<0.58 && min>-0.22)
        output[0] = 1;
    if (max<0.8 && min>-0.5)
        output[1] = 1;
    if (max<0.4 && min>-0.8)
        output[2] = 1;
    if (max<1 && min>-1)
        output[3] = 1;
    if (max<0.3 && min>-0.55)
        output[4] = 1;
    if (max<0.2 && min>-0.85)
        output[5] = 1;
    
}



void check_az1(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<-0.9 && min>-1.2)
        output[0] = 1;
    if (max<-0.2 && min>-1.2)
        output[1] = 1;
    if (max<1.9 && min>0.9)
        output[2] = 1;
    if (max<-0.35 && min>-1.4)
        output[3] = 1;
    if (max<-0.9 && min>-1.3)
        output[4] = 1;
    if (max<-0.3 && min>-0.95)
        output[5] = 1;

}
    



void check_az2(float * data, int length, uint8_t * output)
{


    float max = getMax(data, length);
    float min = getMin(data, length);

    if (max<-0.8 && min>-2.2)
        output[0] = 1;
    if (max<0.2 && min>-0.6)
        output[1] = 1;
    if (max<2.1 && min>-0)
        output[2] = 1;
    if (max<1.3 && min>0.2)
        output[3] = 1;
    if (max<-1.8 && min>-2.1)
        output[4] = 1;
    if (max<1.5 && min>0.6)
        output[5] = 1;
}
    



float getMax(float * data, int length)
{
    float max = 0.0;
    for(int i=0; i<length; i++)
    {
        if(data[i]>max)
            max = data[i];
    }
    return max;
}


float getMin(float * data, int length)
{
    float min = 0.0;
    for(int i=0; i<length; i++)
    {
        if(data[i]<min)
            min = data[i];
    }
    return min;
}