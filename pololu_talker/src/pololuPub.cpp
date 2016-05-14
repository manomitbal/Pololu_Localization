#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>


#define M_PI   3.14159265358979323846 /* pi */

using namespace std;
using namespace ros;

void getPose(double& lastTime, float& accel_old, float& velocity_old, float& distance_old, float& velocity_update, float& distance_update)
{
    double timeNow = ros::Time::now().toSec();;
    velocity_update = accel_old * (timeNow-lastTime) + velocity_old;
    distance_update = velocity_update * (timeNow-lastTime) + distance_old;
}

void getAngVel(clock_t& lastTime, float& lastDegree, float currentDegree, float& angVel)
{
    float degree = currentDegree;
    clock_t timeNow;
    angVel = (float)((degree - lastDegree) / (timeNow - lastTime));
    timeNow = lastTime;
    degree = lastDegree;

}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
      
    init(argc, argv, "Pololu_Sensors");
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    NodeHandle nh;
    Publisher pub1 = nh.advertise<std_msgs::Int32>("sonarData", 1000);
    Publisher pub2 = nh.advertise<geometry_msgs::Pose2D>("Pose", 1000);
    Publisher pub3 = nh.advertise<sensor_msgs::Range>("sonarRange", 1000);
    Publisher pub4 = nh.advertise<sensor_msgs::Imu>("imu/data_fixed", 1000);
    Rate loop_rate(1000);

    int count = 0;

    float oldX = 0.0, oldY = 0.0, alpha = 0.5, filterX = 0.0, filterY = 0.0;
    float oldSonar = 0.0, filterSonar = 0.0;
    float x_Accel = 0.0, y_Accel = 0.0, x_Velocity = 0.0, y_Velocity = 0.0, x_PoseT = 0.0, y_PoseT = 0.0, position_x = 0.0, position_y = 0.0, 
    x_VelocityOLD = 0.0, x_PoseTOLD = 0.0, y_VelocityOLD = 0.0, y_PoseTOLD = 0.0;
    int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    int enable = 1;
    
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        error("setsockopt(SO_REUSEADDR) failed");
    
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = 1989;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
        sizeof(serv_addr)) < 0) 
          error("ERROR on binding");
      double lT = ros::Time::now().toSec();
    
    while(ok())
    {
        listen(sockfd,5);
        clilen = sizeof(cli_addr);
        newsockfd = accept(sockfd, 
            (struct sockaddr *) &cli_addr, 
            &clilen);
        if (newsockfd < 0) 
            error("ERROR on accept");
        bzero(buffer,256);
        n = read(newsockfd,buffer,255);
        if (n < 0) error("ERROR reading from socket");
        string line(buffer);
        stringstream stream(line);
        std_msgs::Int32 sonarData;
        geometry_msgs::Pose2D pose_msg;
        sensor_msgs::Range range;
        sensor_msgs::Imu imu_msgs;
        while(stream)
        {
            float sonarDataT, headingT;
            stream >> sonarDataT >> x_Accel >> y_Accel >> headingT;
            sonarDataT = sonarDataT * 0.0254;
            filterSonar = oldSonar + alpha * ((sonarDataT) - oldSonar);
            filterX = oldX + alpha * ((x_Accel) - oldX);
            filterY = oldY + alpha * ((y_Accel) - oldY);
            
            
            getPose(lT, filterX, x_VelocityOLD, x_PoseTOLD, x_Velocity, x_PoseT);
            getPose(lT, filterY, y_VelocityOLD, y_PoseTOLD, y_Velocity, y_PoseT);
            lT = ros::Time::now().toSec();
            position_x = x_PoseT;
            position_y = y_PoseT;
            
            cout << "Pose x and y: "<<position_x<<" "<<position_y<<endl;
            imu_msgs.header.stamp = ros::Time::now();
            imu_msgs.header.frame_id = "imu";
            imu_msgs.linear_acceleration.x = filterX;
            imu_msgs.linear_acceleration.y = filterY;
            imu_msgs.linear_acceleration.z = 0.0;
            
            pose_msg.x = position_x;
            pose_msg.y = position_y;
            pose_msg.theta = (headingT * M_PI)/ 180;
            
            range.radiation_type = sensor_msgs::Range::ULTRASOUND;
            range.field_of_view = 0.09;
            range.min_range = 0.0;
            range.max_range = 2.0; 
            range.header.frame_id = "sonar_sensor";
            range.range = filterSonar;    
            transform.setOrigin(tf::Vector3(position_x, position_y, 0));
            q.setEuler(0, 0, (-1) * pose_msg.theta);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
            pub1.publish(sonarData);
            pub2.publish(pose_msg);
            pub3.publish(range);
            pub4.publish(imu_msgs);
            
            oldX = filterX;
            oldY = filterY;
            oldSonar = filterSonar;
            x_VelocityOLD = x_Velocity;
            x_PoseTOLD = x_PoseT;
            y_VelocityOLD = y_Velocity;
            y_PoseTOLD = y_PoseT;

            break;
        }
        
        close(newsockfd);
        
    }
    spin();
    close(newsockfd);
    close(sockfd);
    return 0; 
}

