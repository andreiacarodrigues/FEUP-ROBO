#ifndef ROBO_REACTIVE_ROBOT_HEADER
#define ROBO_REACTIVE_ROBOT_HEADER

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <opencv/cv.hpp>

class MyRobot
{
private:
    struct
    {
        double targetPose = -1;
        bool hasTargetPose = false;
        int turningDirection = 0; // 0 - None, 1 - Left, 2 - Right
        bool walking = false;
    } avoidObjectStruct;

    // Operation mode
    enum OperationModeEnum
    {
        FollowWall,
        FollowLine,
        FollowObject,
        AvoidObject
    };

    OperationModeEnum operationMode = FollowLine;

    // Follow wall state
    enum FollowWallStateEnum
    {
        None,
        Left,
        Right
    };

    FollowWallStateEnum followWallState = None;

    // The ROS node handle
    ros::NodeHandle nodeHandle;

    // Subscriber for the ros laser messsage
    ros::Subscriber subscriberLaser;
    ros::Subscriber subscriberOdometer;
    ros::Subscriber subscriberCamera;
    ros::Subscriber subscriberDepth;
    ros::Subscriber subscriberSpeed;

    // The twist publisher
    ros::Publisher publisherSpeed;

    // The sensors info
    sensor_msgs::LaserScan laserMsg;
    nav_msgs::Odometry odometerMsg;
    double pose;
    sensor_msgs::Image cameraMsg;
    sensor_msgs::Image depthMsg;
    int direction;
	bool foundLine = false;
	bool hadFoundLine = false;
	bool foundObject = false;
	double poseWhenObjectFound = -1;
	double directionBias = 0;

    bool laserMessageSet = false;

    // Register and Subscribe
    void registerSubscribersAndPublisher();
    void setDirectionBias();
    void getDirection(cv::Mat image);
    void getObjectDirection(cv::Mat image);
    bool checkIfWallInFront();

    // Updates
    void updateFollowWall(bool finish);
    void updateFollowLine();
    void updateFollowObject();
    void updateAvoidObject();
    bool updateDanger();

    void laserCallback(const sensor_msgs::LaserScan& msg);
    void odometerCallback(const nav_msgs::Odometry& msg);
    void cameraCallback(const sensor_msgs::Image& msg);
    void depthCallback(const sensor_msgs::Image& msg);
    void speedCallback(const geometry_msgs::Twist& msg);

public:
    bool haltMessagePublished = false;
    MyRobot(int argc, char **argv);
    bool update();
    bool halt();

};

#endif