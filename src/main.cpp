#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Point32.h>

ros::Publisher sonar_pub;

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
    return abs(angle-ref_angle) < epsilon/2;
}

void laserCallback(const sensor_msgs::LaserScan &msg) {
    double angle = msg.angle_min+M_PI;
    double angle_increment = msg.angle_increment;
    double a0 = to_rad(144), a1 = to_rad(90), a2 = to_rad(44), a3=to_rad(12);
    sensor_msgs::PointCloud sonar_cloud;
    sonar_cloud.header.frame_id = "husarion/sonar_link";
    sonar_cloud.header.stamp = ros::Time::now();
    geometry_msgs::Point32 p0, p1, p2, p3, p4, p5, p6, p7;
    for (const auto &range : msg.ranges) {
        if (goodAngle(angle, a0, angle_increment)) {
            p7 = polarToCart(angle, range);
        } else if (goodAngle(angle, a1, angle_increment)) {
            p0 = polarToCart(angle, range);
        } else if (goodAngle(angle, a2, angle_increment)) {
            p1 = polarToCart(angle, range);
        } else if (goodAngle(angle, a3, angle_increment)) {
            p2 = polarToCart(angle, range);
        } else if (goodAngle(angle, -a3, angle_increment)) {
            p3 = polarToCart(angle, range);
        } else if (goodAngle(angle, -a2, angle_increment)) {
            p4 = polarToCart(angle, range);
        } else if (goodAngle(angle, -a1, angle_increment)) {
            p5 = polarToCart(angle, range);
        } else if (goodAngle(angle, -a0, angle_increment)) {
            p6 = polarToCart(angle, range);
        }
        angle += angle_increment;
    }
    sonar_cloud.points = {p0, p1, p2, p3, p4, p5, p6, p7};
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
