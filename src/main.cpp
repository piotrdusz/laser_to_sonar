#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Point32.h>

ros::Publisher sonar_pub;

using namespace std;

double to_rad(double degrees) {
    return degrees*M_PI/180;
}

geometry_msgs::Point32 polarToCart(double angle, double range) {
    geometry_msgs::Point32 point;
    point.x = range*cos(angle);
    point.y = range*sin(angle);
    point.z = 0.1;
    return point;
}

bool goodAngle(double angle, double ref_angle, double epsilon) {
    angle = atan2(sin(angle), cos(angle));
    ref_angle = atan2(sin(ref_angle), cos(ref_angle));
    if(abs(angle-ref_angle) < epsilon) {
        return true;
    } else {
        return false;
    }
}

geometry_msgs::Point32 avaragePoint(std::vector<geometry_msgs::Point32>& points) {
    float x=0, y=0, z=0;
    if(points.size()==0) {
        return geometry_msgs::Point32();
    }
    for(size_t i=0; i<points.size(); ++i) {
        x += points[i].x;
        y += points[i].y;
        z += points[i].z;
    }
    geometry_msgs::Point32 output;
    output.x = x/points.size();
    output.y = y/points.size();
    output.z = z/points.size();
    return output;
}

void laserCallback(const sensor_msgs::LaserScan &msg) {
    double angle = msg.angle_min+M_PI;
    double angle_increment = msg.angle_increment;
    double a0 = to_rad(144), a1 = to_rad(90), a2 = to_rad(44), a3=to_rad(12);
    sensor_msgs::PointCloud sonar_cloud;
    sonar_cloud.header.frame_id = "husarion/sonar_link";
    sonar_cloud.header.stamp = ros::Time::now();
    double epsilon = 1.5*angle_increment;
    std::vector<geometry_msgs::Point32> p0s, p1s, p2s, p3s, p4s, p5s, p6s, p7s;
    for (size_t i=0; i<msg.ranges.size(); ++i, angle+=angle_increment) {
        double range = msg.ranges[i];
        if(range > msg.range_max) {
            continue;
        }
        if (goodAngle(angle, a0, epsilon)) {
            p7s.push_back(polarToCart(angle, range));
        } else if (goodAngle(angle, a1, epsilon)) {
            p0s.push_back(polarToCart(angle, range));
        } else if (goodAngle(angle, a2, epsilon)) {
            p1s.push_back(polarToCart(angle, range));
        } else if (goodAngle(angle, a3, epsilon)) {
            p2s.push_back(polarToCart(angle, range));
        } else if (goodAngle(angle, -a3, epsilon)) {
            p3s.push_back(polarToCart(angle, range));
        } else if (goodAngle(angle, -a2, epsilon)) {
            p4s.push_back(polarToCart(angle, range));
        } else if (goodAngle(angle, -a1, epsilon)) {
            p5s.push_back(polarToCart(angle, range));
        } else if (goodAngle(angle, -a0, epsilon)) {
            p6s.push_back(polarToCart(angle, range));
        }
    }
    sonar_cloud.points.push_back(avaragePoint(p0s));
    sonar_cloud.points.push_back(avaragePoint(p1s));
    sonar_cloud.points.push_back(avaragePoint(p2s));
    sonar_cloud.points.push_back(avaragePoint(p3s));
    sonar_cloud.points.push_back(avaragePoint(p4s));
    sonar_cloud.points.push_back(avaragePoint(p5s));
    sonar_cloud.points.push_back(avaragePoint(p6s));
    sonar_cloud.points.push_back(avaragePoint(p7s));
    sonar_pub.publish(sonar_cloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_to_sonar");
    ros::NodeHandle n;
    ros::Rate rate(30);
    ros::Subscriber laser_sub = n.subscribe("/base_scan", 10, &laserCallback);
    sonar_pub = n.advertise<sensor_msgs::PointCloud>("/husarion/sonar", 10);

    ros::Duration(1).sleep();
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
}
