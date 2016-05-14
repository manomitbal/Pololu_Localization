#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "mraa.h"
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <thread>

using namespace std;

int getEulerHeading();
float getLinAccelX();
float getLinAccelY();
void getAngVel();
int getQuatX();
int getQuatY();
void getPoseX(clock_t&, float&, float&, float&);
void getPoseY(clock_t&, float&, float&, float&);
void getVelocityX();
void getVelocityY();

void initI2C();
int sonarData();
float rectData(float);
float normalizeHeading();
float rectDegree(float);
void printSonar();
void printAccel();
void printPose();
void printHeading();

int fd = open("/dev/ttyMFD1", O_RDWR);
int flag = 0;

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
 
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
    ch = getchar();
 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
 
    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
 
    return 0;
}

void initI2C()
{
	mraa_init();
	mraa_i2c_context i2c;
    i2c = mraa_i2c_init(6);
	usleep(2000);
	system("i2cset -y 6 0x28 0x3D 0x0C");
	usleep(2000);
}

int getEulerHeading()
{
    int result1, result2, sum;
	FILE *in1, *in2;
	char buff[512];

	if(!(in1 = popen("i2cget -y 6 0x28 0x1B", "r"))){
		return -1;
	}

	while(fgets(buff, sizeof(buff), in1)!=NULL){
		result1 = strtol(buff,NULL,16);
		
	}
	
	if(!(in2 = popen("i2cget -y 6 0x28 0x1A", "r"))){
               return -1;
        }   
	
	while(fgets(buff, sizeof(buff), in2)!=NULL){
                result2 = strtol(buff,NULL,16);
                
        }   
	result1 = result1 << 8;
	result1 = result1 | result2;
	//result1 = result1/100;
	//sum = result1 + result2;
	pclose(in1);
	pclose(in2);
	return result1;

	
}

int sonarData()
{
    unsigned char buff[16]={0};
    unsigned int t;
    ssize_t size;
    //system("echo M > /dev/ttyMFD1");
    size = read(fd, buff, 1);
    t=(buff[1]<<8)|buff[0];
    t = (t*512/1000);
    return t;
                              
}

void printSonar()
{
    while(1)
    {
        printf("\n%d",sonarData());
        cout.flush();
    }
}

void printAccel()
{
    while(1)
    {
        printf("  %f %f %f ",getLinAccelX(), getLinAccelY(),rectDegree(normalizeHeading()));
        cout.flush();
    }

}

void printHeading()
{
    while(1)
    {
        printf("\t\t\t Heading = %d ",rectDegree(normalizeHeading()));
    }
}

void printPose()
{
    float result;
    float velocityx = 0, distancex = 0, velocityy = 0, distancey = 0;
    float posX = 0.0, posY = 0.0;
    clock_t lT = clock();
    while(1)
    {
        result = rectDegree(normalizeHeading());
        getPoseX(lT, velocityx, distancex, result);
        getPoseY(lT, velocityy, distancey, result);
        posX += distancex * 100000;
        posY += distancey * 100000;
        printf("  %f %f %f",posX, posY, result);
        cout.flush();
    }     
}

void getAngVel(clock_t& lastTime, float& lastDegree, float& angVel)
{
    float degree = rectDegree(normalizeHeading());
    clock_t timeNow;
    angVel = (float)((degree - lastDegree) / (timeNow - lastTime));
    timeNow = lastTime;
    degree = lastDegree;

}

float getLinAccelY()
{
	 int result1, result2, sum;
     float final;
     FILE *in1, *in2;
     char buff[1024];
	 //cout<<sonarData();
	
        if(!(in1 = popen("i2cget -y 6 0x28 0x29", "r"))){
                return -1;
        }

        while(fgets(buff, sizeof(buff), in1)!=NULL){
                result1 = (int)strtol(buff,NULL,16);
        }

        if(!(in2 = popen("i2cget -y 6 0x28 0x28", "r"))){
                return -1;
        }

        while(fgets(buff, sizeof(buff), in2)!=NULL){
                result2 = (int)strtol(buff,NULL,16);

        }
    result1 = (result1) << 8;
	final = (float)(result1 | result2);
    //final = final/100;
    if(final > 32767)
        final = final - 65536;
    final = final/100;
	//result1 = result1/100;
	pclose(in1);
	pclose(in2);
    return (final);
    
}

void getPoseX(clock_t& lastTime, float& distance, float& velocity, float& lastDegree)
{
	float accel = 0.0;
    float vzero = 0.0;
    float distzero = 0.0;
    float elapsed;
	clock_t end;
    accel = rectData(getLinAccelX());
    end = clock();
    elapsed = (float)(((float)end - (float)lastTime) / 1000000);
    getAngVel(lastTime, lastDegree, velocity);    
    distance = distzero + (velocity*elapsed) + (0.5 * accel * elapsed * elapsed);
    usleep(100);
    lastTime = end; 
    //vzero = velocity;
    distzero = distance;
}

float getLinAccelX()
{
	    int result1, result2, sum;
        float final;
        float degree = rectDegree(normalizeHeading());
        FILE *in1, *in2;
        char buff[1024];

        if(!(in1 = popen("i2cget -y 6 0x28 0x2B", "r"))){
                return -1;
        }

        while(fgets(buff, sizeof(buff), in1)!=NULL){
                result1 = strtol(buff,NULL,16);

        }

        if(!(in2 = popen("i2cget -y 6 0x28 0x2A", "r"))){
                return -1;
        }

        while(fgets(buff, sizeof(buff), in2)!=NULL){
                result2 = strtol(buff,NULL,16);

        }

	result1 = result1 << 8;
    final = (float)(result1 | result2);
    if(final > 32767)
                final = final - 65536;
	//result1 = result1/100;
    final = final/100;
    pclose(in1);
	pclose(in2);
	return (final);

}

void getPoseY(clock_t& lastTime, float& distance, float& velocity, float& lastDegree)
{
    float accel = 0.0;
    float vzero = 0.0;
    float distzero = 0.0;
    float elapsed;
    clock_t end;
    accel = rectData(getLinAccelY());
    end = clock();
    elapsed = (float)(((float)end - (float)lastTime) / 1000000);
    getAngVel(lastTime, lastDegree, velocity);    
    distance = distzero + (velocity*elapsed) + (0.5 * velocity * elapsed * elapsed);
    usleep(100);
    lastTime = end;
    //vzero = velocity;
    distzero = distance;
                
}       

float normalizeHeading()
{
    return (((float)getEulerHeading()/(float)5759) * (float)360);
}

float rectData(float accel)
{
    if((accel > 100 ) || (accel < -100) || (accel < 20 && accel > -20))
        return 0;
    else
        return accel;
}


float rectDegree(float degree)
{
    if(degree > 180)
        degree = degree - 360;
    return degree;
}

int main(int argc,char* argv[])
{

	initI2C();
	int count = 0;
    //float result;
    //float velocityx = 0, distancex = 0, velocityy = 0, distancey = 0;
    //float posX = 0.0, posY = 0.0;
    //clock_t lT = clock();
	system("stty -F /dev/ttyMFD1 115200 -icanon min 100 -hupcl brkint -icrnl -opost -onlcr -isig -echo ignpar time 2");
    system("echo M > /dev/ttyMFD1");
    thread first(printSonar);
    thread second(printAccel);
    //thread third(printHeading);
    first.join();
    second.join();
    //third.join();
       
    //while(!kbhit())
	//{
	    
      // if(sonarData() > 0)
      // {
            //result = rectDegree(normalizeHeading());
            //getPoseX(lT, velocityx, distancex, result);
            //getPoseY(lT, velocityy, distancey, result);
           // posX += distancex;
            //posY += distancey;
            //cout<<posX<<"  "<<posY<<"  "<<result<<"  "<<sonarData()<<endl;
            //cout<<sonarData()<<"   "<<getLinAccelY()<<endl;
	//	    if((float)((float)sonarData()*cos(result)) <= 90 && (float)((float)sonarData()*sin(result)) <= 90 && (float)((float)sonarData()*cos(result)) >= -90 
    //	    && (float)((float)sonarData()*sin(result)) >= -90 )
            //cout<<(float)((float)sonarData()*cos(result))<<"   "<<(float)((float)sonarData()*sin(result))<<endl;
            //cout<<sonarData()<<endl;
      //}  
           
	//}
	return 0;
}
