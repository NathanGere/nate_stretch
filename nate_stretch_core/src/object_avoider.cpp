#include <atomic>
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

ros::Publisher pub;

void callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    geometry_msgs::Twist motorizer;
    int size = scan->ranges.size();
    float right = scan->ranges[0];
    //float lc = scan->ranges[(size-1)/4];
    float center = scan->ranges[(size-1)/2];
    //float rc = scan->ranges[(size-1)*(3/4)];
    float left = scan->ranges[size-1];
    ROS_INFO("\nRange at Sensor Left: %f \n Range at Sensor Center: %f \n Range at Sensor Right: %f", scan->ranges[0], scan->ranges[size/2], scan->ranges[size]);
    
    int cntr = 0;
    int l = 0;
    int r = 0;

    if(center < 2.5)//010
    {
        cntr = 1;
    }
    if(right < 1.3)//001
    {
        r = 2;
    }
    if(l < 1.3)//100
    {
        l = 4;
    }

    int sensing = cntr + l + r;

    motorizer.linear.x = 4.0;

    switch(sensing)
    {
        case 0:
            motorizer.linear.x = 5.0;
            break;
        case 1:
            if(right > left)
            {
                motorizer.angular.z = -4.0;
            }
            else if(left > right)
            {
                motorizer.angular.z = 4.0;
            }
            else
            {
                motorizer.linear.x = 0.5;
                motorizer.angular.z = 2.0;
            }
            break;
        case 2://001
                motorizer.angular.z = 4.0;
            break;
        case 3://011
                motorizer.linear.x = 2.0;
                motorizer.angular.z = 5.0;
            break;
        case 4:
                motorizer.angular.z = -4.0;
            break;
        case 5:
                motorizer.linear.x = 2.0;
                motorizer.angular.z = -5.0;
            break;
        case 6:
                if(center > 3.5)
                {
                    motorizer.linear.x = 2.0;
                }
                if(right > left)
                {
                    motorizer.angular.z = 4.0;
                }
                if(left > right)
                {
                    motorizer.angular.z = -4.0;
                }
            break;
        case 7:
                motorizer.linear.x = 2.0;
                if(right > left)
                {
                    motorizer.angular.z = 4.0;
                }
                if(left > right)
                {
                    motorizer.angular.z = -4.0;
                }
                else 
                {
                    motorizer.angular.z = 0.5;
                }
            break;
    }

    pub.publish(motorizer);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_solver");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("robot/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("robot/base_scan",1000,callback);

    ros::spin();
}

 /*
    motorizer.linear.x=5.0;
    if(right <= 1.2 && left <= 1.2)
    {
        motorizer.linear.x=0.5;
        if(right < left && right + 0.4 > left)
        {
            motorizer.angular.z=2.0;
            motorizer.linear.x=1.0;
        }
        if(right < left)
        {
            motorizer.angular.z=1.0;
        }
        if(left < right)
        {
            motorizer.angular.z=-1.0;
        }
    }
    if(center <= 3.0)
    {
        motorizer.linear.x=0.5;
        if(right < left && right + 0.4 > left)
        {
            motorizer.linear.x = 1.0;
        }
        else if(right < left)
        {
            motorizer.angular.z=2.0;
        }
        else if(left < right)
        {
            motorizer.angular.z=-2.0;
        }
    }
    else if(left <= 1.3 && right > left)
    {
        motorizer.angular.z=-1.0;
    }
    else if(right <= 1.3 && left > right)
    {
        motorizer.angular.z=1.0;
    
    }
    else if(right <= 1.3)
    {
        motorizer.angular.z=1.0;
    }
    else if(left <= 1.3)
    {
        motorizer.angular.z=-1.0;
    }*/