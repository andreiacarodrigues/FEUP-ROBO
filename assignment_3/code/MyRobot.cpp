#include "MyRobot.h"
#include "Utils.h"

#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <cmath>

#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>

#define LASER_TOPIC_NAME "/scan"
#define ODOMETER_TOPIC_NAME "/odom"
#define CAMERA_TOPIC_NAME "/camera/rgb/image_raw"
#define DEPTH_TOPIC_NAME "/camera/depth/image_raw"
#define SPEED_TOPIC_NAME "/cmd_vel"

#define DETECT_DISTANCE 1.0
#define AVOID_DISTANCE 0.6
#define WALL_DISTANCE 0.5
#define LINEAR_SPEED 0.4
#define CONSTANT_K 2

#define GAUSSIAN_KERNEL_SIZE 3
#define GAUSSIAN_STDEV 0.1

#define TURNING_TOLERANCE 15

#define DANGER_DISTANCE 0.2

MyRobot::MyRobot(int argc, char **argv)
{
    if(argc < 2)
    {
        printf("Bad arguments\n");
        exit(1);
    }
    long parseInt = std::strtol(argv[1], nullptr, 10);
    switch(parseInt)
    {
        case 0:
            printf("Wall Follow Mode\n");
            operationMode = FollowWall;
            break;
        case 1:
            printf("Line Follow Mode\n");
            operationMode = FollowLine;
            break;
        case 2:
            printf("Object Follow Mode\n");
            operationMode = FollowObject;
            break;
        case 3:
            printf("Object Avoid Mode\n");
            operationMode = AvoidObject;
            break;
        default:
            printf("Bad arguments\n");
            exit(1);
    }
    registerSubscribersAndPublisher();
}

void MyRobot::registerSubscribersAndPublisher()
{
    subscriberLaser = nodeHandle.subscribe(LASER_TOPIC_NAME, 1, &MyRobot::laserCallback, this);
    subscriberOdometer = nodeHandle.subscribe(ODOMETER_TOPIC_NAME, 1, &MyRobot::odometerCallback, this);
    subscriberCamera = nodeHandle.subscribe(CAMERA_TOPIC_NAME, 1, &MyRobot::cameraCallback, this);
    subscriberDepth = nodeHandle.subscribe(DEPTH_TOPIC_NAME, 1, &MyRobot::depthCallback, this);
    subscriberSpeed = nodeHandle.subscribe(SPEED_TOPIC_NAME, 1, &MyRobot::speedCallback, this);
    publisherSpeed = nodeHandle.advertise<geometry_msgs::Twist>(SPEED_TOPIC_NAME, 1);
}

bool MyRobot::update()
{
    // Common Logic

    // Specific Logic
    switch(operationMode)
    {
        case FollowWall:
            updateFollowWall(false);
            break;
        case FollowLine:
            updateFollowLine();
            break;
        case FollowObject:
            updateFollowObject();
            break;
        case AvoidObject:
            updateAvoidObject();
            break;
    }
    return updateDanger();
}

void MyRobot::updateFollowWall(bool finish)
{
    if(!laserMessageSet)
    {
        return;
    }

    geometry_msgs::Twist speedMsg;
    speedMsg.linear.x = finish ? 0 : 0.4;
    speedMsg.angular.z = 0;


    double distance = std::numeric_limits<double>::max();
    double distanceFront = std::numeric_limits<double>::max();
    double angle = 0.0;
    double sensorAngle = toDegrees(laserMsg.angle_min);
    double angleIncrement = toDegrees(laserMsg.angle_increment);
    bool assigningSide = followWallState == None;

    for (float sensorDist : laserMsg.ranges)
    {
        if(sensorDist < laserMsg.range_min || sensorDist > laserMsg.range_max)
        {
            sensorAngle += angleIncrement;
            continue;
        }

        if(sensorAngle > 180)
        {
            sensorAngle = sensorAngle - 360;
        }

        if(sensorAngle <= 10 && sensorAngle >= -10 && sensorDist < distanceFront)
        {
            distanceFront = sensorDist;
        }

        if(sensorAngle <= 0 && followWallState != Left && sensorDist < distance)
        {
            distance = sensorDist;
            angle = sensorAngle;
        }
        else if(sensorAngle >= 0 && followWallState != Right && sensorDist < distance)
        {
            distance = sensorDist;
            angle = sensorAngle;
        }
        sensorAngle += angleIncrement;
    }

    if(distance >= laserMsg.range_min && distance <= laserMsg.range_max)
    {
        if(assigningSide)
        {
            if(angle <= 0)
            {
                followWallState = Right;
            }
            else
            {
                followWallState = Left;
            }
        }

        double beta = angle;
        double alpha;

        if(angle >= 0)
        {
            alpha = 90 - angle;
        }
        else
        {
            alpha = -90 - angle;
        }

        if(distanceFront < WALL_DISTANCE + 0.2)
        {
            speedMsg.linear.x = 0.1*LINEAR_SPEED;
        }
        else
        {
            speedMsg.linear.x = LINEAR_SPEED;
        }
        
        double angleFactor = sin(toRadians(alpha));
        double wallDistanceFactor = sin(toRadians(beta)) * ((distance - WALL_DISTANCE)/WALL_DISTANCE);
        double sensorFactor = angleFactor - wallDistanceFactor;
        speedMsg.angular.z = -CONSTANT_K * sensorFactor;
    }

    publisherSpeed.publish(speedMsg);
}

bool MyRobot::checkIfWallInFront()
{
    double distance = std::numeric_limits<double>::max();
    double sensorAngle = toDegrees(laserMsg.angle_min);
    double angleIncrement = toDegrees(laserMsg.angle_increment);

    for (float sensorDist : laserMsg.ranges)
    {
        if (sensorDist < laserMsg.range_min || sensorDist > laserMsg.range_max)
        {
            sensorAngle += angleIncrement;
            continue;
        }

        if (sensorAngle > 180)
        {
            sensorAngle = sensorAngle - 360;
        }

        if (sensorAngle <= 0 && sensorAngle >= -0.25 && followWallState != Left && sensorDist < distance)
        {
            distance = sensorDist;
        }
        else if (sensorAngle >= 0 && sensorAngle <= 0.25 && followWallState != Right && sensorDist < distance)
        {
            distance = sensorDist;;
        }
        sensorAngle += angleIncrement;
    }

    return distance >= laserMsg.range_min && distance <= laserMsg.range_max && distance <= DETECT_DISTANCE;
}

void MyRobot::updateFollowLine()
{
    std::cout << foundObject << foundLine << std::endl;
	if(!foundLine)
	{
	    foundObject = false;
		updateFollowWall(true);
        return;
	}

	if(!foundObject)
    {
	    foundObject = checkIfWallInFront();
	    if(foundObject)
        {
	        poseWhenObjectFound = pose;
        }
    }

	if(foundObject)
    {
	    if(!foundLine)
        {
	        foundObject = false;
        }

	    if(foundObject)
	    {
            updateFollowWall(true);
            return;
        }
    }
	
    // If direction is left
    geometry_msgs::Twist velocity;
    if (direction == 0)
	{
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.15;
        publisherSpeed.publish(velocity);
    }
    // If direction is straight
    if (direction == 1)
	{
        velocity.linear.x = 0.15;
        velocity.angular.z = 0;
        publisherSpeed.publish(velocity);
    }
    // If direction is right
    if (direction == 2)
	{
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.15;
        publisherSpeed.publish(velocity);
    }
}

void MyRobot::updateFollowObject()
{
    geometry_msgs::Twist velocity;
    if(!foundObject)
    {
        velocity.linear.x = 0;
        velocity.angular.z = 0;
        publisherSpeed.publish(velocity);
        return;
    }

    if (direction == 0)
    {
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.15;
        publisherSpeed.publish(velocity);
    }
    // If direction is straight
    if (direction == 1)
    {
        velocity.linear.x = 0.15;
        velocity.angular.z = 0;
        publisherSpeed.publish(velocity);
    }
    // If direction is right
    if (direction == 2)
    {
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.15;
        publisherSpeed.publish(velocity);
    }
}

void MyRobot::updateAvoidObject()
{
    if(!laserMessageSet)
    {
        return;
    }

    geometry_msgs::Twist speedMsg;
    speedMsg.linear.x = 0;
    speedMsg.angular.z = 0;

    if(avoidObjectStruct.hasTargetPose)
    {
        if((avoidObjectStruct.turningDirection == 1 && pose >= avoidObjectStruct.targetPose) || (avoidObjectStruct.turningDirection == 2 && pose <= avoidObjectStruct.targetPose))
        {
            avoidObjectStruct.walking = true;
            speedMsg.linear.x = LINEAR_SPEED;
            speedMsg.angular.z = 0;
            publisherSpeed.publish(speedMsg);
            avoidObjectStruct.hasTargetPose = false;
        }
    }
    else
    {
        if(avoidObjectStruct.walking)
        {
            double distance = std::numeric_limits<double>::max();
            double angle = 0.0;
            double sensorAngle = toDegrees(laserMsg.angle_min);
            double angleIncrement = toDegrees(laserMsg.angle_increment);
            bool somethingBehind = false;
            for (float sensorDist : laserMsg.ranges)
            {
                if (sensorDist < laserMsg.range_min || sensorDist > laserMsg.range_max || sensorDist > AVOID_DISTANCE*2)
                {
                    sensorAngle += angleIncrement;
                    continue;
                }

                if (sensorAngle > 180)
                {
                    sensorAngle = sensorAngle - 360;
                }

                if (std::abs(sensorAngle) <= 180 && std::abs(sensorAngle) >= 30)
                {
                    somethingBehind = true;
                    if(sensorDist < distance)
                    {
                        distance = sensorDist;
                        angle = sensorAngle;
                    }
                }

                if(distance < laserMsg.range_max)
                {
                    double alpha;
                    if(angle > 0)
                    {
                        alpha = angle - 180;
                    }
                    else
                    {
                        alpha = 180 + angle;
                    }
                    speedMsg.linear.x = LINEAR_SPEED;
                    speedMsg.angular.z = sin(toRadians(alpha));
                    publisherSpeed.publish(speedMsg);
                }

                sensorAngle += angleIncrement;
            }
            if(!somethingBehind)
            {
                avoidObjectStruct.walking = false;
                speedMsg.linear.x = 0;
                speedMsg.angular.z = 0;
                publisherSpeed.publish(speedMsg);
            }
            else
            {
                speedMsg.linear.x = LINEAR_SPEED;
                speedMsg.angular.z = 0;
                publisherSpeed.publish(speedMsg);
            }
        }

        double distance = std::numeric_limits<double>::max();
        double angle = 0.0;
        double sensorAngle = toDegrees(laserMsg.angle_min);
        double angleIncrement = toDegrees(laserMsg.angle_increment);

        for (float sensorDist : laserMsg.ranges)
        {
            if (sensorDist < laserMsg.range_min || sensorDist > laserMsg.range_max)
            {
                sensorAngle += angleIncrement;
                continue;
            }

            if (sensorAngle > 180)
            {
                sensorAngle = sensorAngle - 360;
            }

            if ((!avoidObjectStruct.walking || (std::abs(sensorAngle) <= 30)) && sensorDist < distance)
            {
                distance = sensorDist;
                angle = sensorAngle;
            }
            sensorAngle += angleIncrement;
        }

        if (distance >= laserMsg.range_min && distance <= laserMsg.range_max && distance <= AVOID_DISTANCE)
        {
            if (angle >= 0)
            {
                angle -= 180;
                avoidObjectStruct.turningDirection = 2;
                speedMsg.linear.x = 0;
                speedMsg.angular.z = -0.5;
            }
            else
            {
                angle += 180;
                avoidObjectStruct.turningDirection = 1;
                speedMsg.linear.x = 0;
                speedMsg.angular.z = 0.5;

            }

            avoidObjectStruct.targetPose = pose + angle;

            if(avoidObjectStruct.targetPose > 179)
            {
                avoidObjectStruct.targetPose -= 360;
            }
            else if(avoidObjectStruct.targetPose < -179)
            {
                avoidObjectStruct.targetPose += 360;
            }
            avoidObjectStruct.hasTargetPose = true;

            publisherSpeed.publish(speedMsg);
        }
    }
}

void MyRobot::laserCallback(const sensor_msgs::LaserScan &msg)
{
    MyRobot::laserMsg = msg;
    laserMessageSet = true;
}

void MyRobot::odometerCallback(const nav_msgs::Odometry &msg)
{
    if(operationMode == AvoidObject || operationMode == FollowLine)
    {
        MyRobot::odometerMsg = msg;
        auto orientation = msg.pose.pose.orientation;
        double sin = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
        double cos = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
        pose = toDegrees(atan2(sin, cos));
    }
}

void MyRobot::setDirectionBias()
{
    double value = pose - poseWhenObjectFound;
    if(value > 180)
    {
        value -= 360;
    }
    directionBias = 0.3*(value)/90.0;
    if(directionBias > 0.3)
    {
        directionBias = 0.3;
    }
    if(directionBias < -0.3)
    {
        directionBias = 0.3;
    }
}

void MyRobot::getDirection(cv::Mat image)
{
	// Image Dimensions
    cv::Size size = image.size();
    auto width = size.width;
    auto height = size.height;

	// Color Masking
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    cv::Scalar lowerBound = {20, 100, 100};
    cv::Scalar upperBound = {30, 255, 255};
    cv::Mat mask;
    cv::inRange(hsv, lowerBound, upperBound, mask);

    if(poseWhenObjectFound != -1 && foundLine && !hadFoundLine)
    {
        setDirectionBias();
        poseWhenObjectFound = -1;
        ROS_INFO("Direction Bias set to: %f", directionBias);
    }

    if(directionBias != 0)
    {
        ROS_INFO("Direction Bias: %f", directionBias);
    }
	// Cut Irrelevant Areas
    mask(cv::Rect(0           , 0 , width       , 0.8  * height )) = 0;
    mask(cv::Rect((0.7+directionBias) * width , 0 , (0.3-directionBias) * width , height        )) = 0;
    mask(cv::Rect(0           , 0 , (0.3+directionBias) * width , height        )) = 0;
	
	// Calculating Moments
    cv::Moments M = cv::moments(mask);
	
	// Check If It Found Anything Relevant
	hadFoundLine = foundLine;
	if(M.m00 <= 0)
    {
        foundLine = false;
		return;
    }
	foundLine = true;
	
	// Median Coordinates
	auto medianX = M.m10/M.m00;

	// Calculating Direction
    if (medianX < width/2 - TURNING_TOLERANCE)
	{
        direction = 0;
    }
	else if (medianX > width/2 + TURNING_TOLERANCE)
	{
        direction = 2;
    }
	else
	{
        direction = 1;
        directionBias = 0;
    }
}

void MyRobot::getObjectDirection(cv::Mat image)
{
    // Image Dimensions
    cv::Size size = image.size();
    auto width = size.width;
    auto height = size.height;

    // Color Masking
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    cv::Scalar lowerBound = {95, 105, 100};
    cv::Scalar upperBound = {105, 125, 200};
    cv::Mat mask;
    cv::inRange(hsv, lowerBound, upperBound, mask);

    // Cut Irrelevant Areas
    mask(cv::Rect(0           , 0 , width       , 0.4  * height )) = 0;
    //mask(cv::Rect(0.7 * width , 0 , 0.3 * width , height        )) = 0;
    //mask(cv::Rect(0           , 0 , 0.3 * width , height        )) = 0;

    // Calculating Moments
    cv::Moments M = cv::moments(mask);

    ROS_INFO("m00: %f", M.m00);
    // Check If It Found Anything Relevant
    if(M.m00 <= 0)
    {
        foundObject = false;
        return;
    }
    foundObject = true;

    // Median Coordinates
    auto medianX = M.m10/M.m00;
    auto medianY = M.m01/M.m00;

    // Calculating Direction
    if (medianX < width/2 - TURNING_TOLERANCE)
    {
        direction = 0;
    }
    else if (medianX > width/2 + TURNING_TOLERANCE)
    {
        direction = 2;
    }
    else
    {
        direction = 1;
        directionBias = 0;
    }
}

void MyRobot::cameraCallback(const sensor_msgs::Image &msg)
{
    if(operationMode == FollowLine)
    {
        MyRobot::cameraMsg = msg;

        cv_bridge::CvImagePtr image_pointer;
        try
        {
            image_pointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        cv::Mat image = image_pointer->image;

        cv::Mat filtered_image;
        cv::GaussianBlur(image, filtered_image, cv::Size(GAUSSIAN_KERNEL_SIZE, GAUSSIAN_KERNEL_SIZE), GAUSSIAN_STDEV, GAUSSIAN_STDEV);

        cv::Mat filtered_downsampled_image;
        cv::pyrDown(filtered_image, filtered_downsampled_image);

        getDirection(filtered_downsampled_image);

        std::cout << direction << std::endl;
    }
    else if(operationMode == FollowObject)
    {
        MyRobot::cameraMsg = msg;

        cv_bridge::CvImagePtr image_pointer;
        try
        {
            image_pointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        cv::Mat image = image_pointer->image;

        cv::Mat filtered_image;
        cv::GaussianBlur(image, filtered_image, cv::Size(GAUSSIAN_KERNEL_SIZE, GAUSSIAN_KERNEL_SIZE), GAUSSIAN_STDEV, GAUSSIAN_STDEV);

        cv::Mat filtered_downsampled_image;
        cv::pyrDown(filtered_image, filtered_downsampled_image);

        getObjectDirection(filtered_downsampled_image);

        std::cout << direction << std::endl;
    }
}

void MyRobot::depthCallback(const sensor_msgs::Image &msg)
{
    return;
    MyRobot::depthMsg = msg;

    cv_bridge::CvImagePtr image_pointer;
    try
    {
        image_pointer = cv_bridge::toCvCopy(msg);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool MyRobot::updateDanger()
{
    double sensorAngle = toDegrees(laserMsg.angle_min);
    double angleIncrement = toDegrees(laserMsg.angle_increment);
    for(float sensorDist : laserMsg.ranges)
    {
        if(sensorDist < laserMsg.range_min || sensorDist > laserMsg.range_max)
        {
            sensorAngle += angleIncrement;
            continue;
        }

        if(sensorAngle > 180)
        {
            sensorAngle = sensorAngle - 360;
        }

        if(sensorAngle > -90 && sensorAngle < 90 && sensorDist < DANGER_DISTANCE)
        {
            ROS_WARN("DANGER DETECTED");
            return false;
        }

        sensorAngle += angleIncrement;
    }
    return true;
}

geometry_msgs::Twist NullSpeed()
{
    geometry_msgs::Twist message;

    message.linear.x = 0;
    message.linear.y = 0;
    message.linear.z = 0;

    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = 0;

    return message;
}

bool MyRobot::halt()
{
    printf("Halting...\n");
    publisherSpeed.publish(NullSpeed());
    if(!haltMessagePublished)
    {
        printf("Waiting...\n");
        return false;
    }
    return true;
}

bool isNullSpeed(const geometry_msgs::Twist &msg)
{
    return msg.linear.x == 0;
}

void MyRobot::speedCallback(const geometry_msgs::Twist &msg)
{
    if(isNullSpeed(msg))
    {
        printf("Halted\n");
        haltMessagePublished = true;
    }

}