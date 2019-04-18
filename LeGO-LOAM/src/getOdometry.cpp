#include <cmath>
#include <vector>
#include<iostream>
#include<fstream>
#include<sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


using namespace std;

ofstream outFile1;//odo
ofstream outFile2;//integrated

int count1 = 0;
int count2 = 0;
void OdometryCallback1(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
        count1++;
    /*
        cout << laserOdometry->header.stamp
                << "," << laserOdometry->pose.pose.position.x << "," << laserOdometry->pose.pose.position.y << "," << laserOdometry->pose.pose.position.z << ","
                << laserOdometry->pose.pose.orientation.x << "," << laserOdometry->pose.pose.orientation.y << ","
                << laserOdometry->pose.pose.orientation.z << "," << laserOdometry->pose.pose.orientation.w << "" << endl;
    */
        outFile1 << laserOdometry->header.stamp
                << ","<< laserOdometry->pose.pose.position.x << ","<< laserOdometry->pose.pose.position.y<< ","<< laserOdometry->pose.pose.position.z << ","
                << laserOdometry->pose.pose.orientation.x << ","<< laserOdometry->pose.pose.orientation.y << ","
                << laserOdometry->pose.pose.orientation.z << ","<< laserOdometry->pose.pose.orientation.w << ""<< endl;
}
void OdometryCallback2(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
    count2++;
        outFile2 << laserOdometry->header.stamp
                << ","<< laserOdometry->pose.pose.position.x << ","<< laserOdometry->pose.pose.position.y<< ","<< laserOdometry->pose.pose.position.z << ","
                << laserOdometry->pose.pose.orientation.x << ","<< laserOdometry->pose.pose.orientation.y << ","
                << laserOdometry->pose.pose.orientation.z << ","<< laserOdometry->pose.pose.orientation.w << ""<< endl;
}

int main(int argc, char** argv)
{

        ros::init(argc, argv, "aaa");
        ros::NodeHandle nh;
    string seqNum;
    string saveFileName2;
    if(!ros::param::get("/outputName",saveFileName2))
        {
        cout<<"没有得到输出文件名"<<endl;
        //return 1;
        }
    //一定要写地址全名!!!重要
    //string saveFileName2 = "/home/localization/work/qt_568/siyuanIntegraPlusIMU.txt";
        //string saveFileName = "0000002.txt";

        outFile2.open(saveFileName2.c_str(),ios::out);
    if(!outFile2)
        cout<<"创建文件失败"<<endl;


    ros::Subscriber subIntegratedLaserOdom = nh.subscribe<nav_msgs::Odometry>
                ("integrated_to_init", 5, OdometryCallback2);
        ros::spin();

    outFile2.close();

        return 0;
}
