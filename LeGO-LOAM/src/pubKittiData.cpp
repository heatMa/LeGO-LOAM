#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
//#pragma warning(disable:4996)
#include<iostream>
#include<pcl/visualization/pcl_visualizer.h>
#include<string>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include<Eigen/Geometry>
#include<vector>
//降采样头文件
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/approximate_voxel_grid.h>
//转换时间戳
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>

using namespace Eigen;
using namespace std;
using namespace pcl;
void getFileNames(string dir, vector<string> &fileName);
//比较函数
bool computePairNum(std::string pair1, std::string pair2)
{
    return pair1 < pair2;
}
//排序
void sort_filelists(std::vector<std::string>& filists, std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(), filists.end(), computePairNum);
}
//读取文件列表，文件夹后一定要记得加/,不然文件读不到
void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}


//从.bin文件加载点云
void readKittiPclBinData(std::string &in_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points_xyzi)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        PointXYZI pxyzi;
        input.read((char*)&pxyzi.x, 3 * sizeof(float));//3D Point
        input.read((char *) &pxyzi.intensity, sizeof(float));
        points_xyzi->push_back(pxyzi);
    }
    //保存时这两个变量一定要自己设置
    points_xyzi->width = i;
    points_xyzi->height = 1;
    input.close();
}

//加载位姿
vector<Matrix4d> loadPoses(string file_name) {
  vector<Matrix4d> poses;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return poses;
  while (!feof(fp)) {
    Matrix4d P(Matrix4d::Identity());
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P(0,0), &P(0,1), &P(0,2), &P(0,3),
                   &P(1,0), &P(1,1), &P(1,2), &P(1,3),
                   &P(2,0), &P(2,1), &P(2,2), &P(2,3) )==12) {
      poses.push_back(P);
    }
  }
  fclose(fp);
  return poses;
}


//加载内参矩阵
vector<Matrix4d> loadCalib(string file_name) {
  vector<Matrix4d> calib;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return calib;

  while (!feof(fp)) {
    Matrix4d P(Matrix4d::Identity());
    char a[10];//P0 P1 P2 P3 Tr
    //相较于加载位姿，多加了一个%s，所以最后参数判断为13！！fuck bug
    if (fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   a,
                   &P(0,0), &P(0,1), &P(0,2), &P(0,3),
                   &P(1,0), &P(1,1), &P(1,2), &P(1,3),
                   &P(2,0), &P(2,1), &P(2,2), &P(2,3) )==13) {
      calib.push_back(P);
    }
  }
  fclose(fp);
  return calib;
}
//把leftcamer真值转换为lidar真值
vector<Matrix4d> getLidarPoses(const vector<Matrix4d>& leftcammer_poses,const vector<Matrix4d>& calib)
{
    vector<Matrix4d> lidarPoses;
    Matrix4d Tr=calib[4];
    for(int i=0;i<leftcammer_poses.size();++i)
    {
        Matrix4d lidarPose;
        lidarPose=leftcammer_poses[i]*Tr;
        lidarPoses.push_back(lidarPose);
        //cout<<lidarPose(0,3)<<","<<lidarPose(1,3)<<","<<lidarPose(2,3)<<endl;
    }
    return lidarPoses;
}

//加载时间戳
vector<float> loadTimes(string file_name)
{
    vector<float> times;
    ifstream times_file(file_name.c_str());
    while(!times_file.eof())
    {
        float stamp;
        times_file >> stamp;
        times.push_back(stamp);
    }
    //忘了写返回值，但是竟然编译器竟然不报错？？？导致了下面这条运行错误
    //terminate called after throwing an instance of 'std::runtime_error
    //what():  Time is out of dual 32-bit range
    return times;
}

//将真值转换为ROS Odometry格式数据发布出去
nav_msgs::Odometry getgroundTruthOdo(const Matrix4d& pose) {
        nav_msgs::Odometry odom;

        Matrix3d R=pose.block(0,0,3,3);
        Vector3d t=pose.block(0,3,3,1);
        //利用eigen从矩阵获得四元数
        Eigen::Quaterniond quat(R);
        quat.normalize();
        //转换为ROS下的四元数
        geometry_msgs::Quaternion odom_quat;
        odom_quat.w = quat.w();
        odom_quat.x = quat.x();
        odom_quat.y = quat.y();
        odom_quat.z = quat.z();
        odom.pose.pose.position.x = t[0];
        odom.pose.pose.position.y = t[1];
        odom.pose.pose.position.z = t[2];
        odom.pose.pose.orientation = odom_quat;

        return odom;
    }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointPublish");
    ros::NodeHandle nh;
    ros::Publisher g_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    ros::Publisher g_lidarpos_pub = nh.advertise<nav_msgs::Odometry>("/imu/gps", 2);
    ros::Rate loop_rate(10);//频率

    std::string bin_path;
    std::string times_txt_name;
    std::string calib_txt_name;
    std::string pos_txt_name;

    //KITTI位置
    std::string KITTI_directory;
    //序列号
    std::string seqNum;

    if(ros::param::get("/KITTI_directory",KITTI_directory) && ros::param::get("/seqNum",seqNum))
    {
        //文件夹后一定要记得加/,不然文件读不到
        bin_path = KITTI_directory+"/data_odometry_velodyne/dataset/sequences/"+seqNum+"/velodyne/";
        times_txt_name = KITTI_directory+"/data_odometry_velodyne/dataset/sequences/"+seqNum+"/times.txt";
        calib_txt_name = KITTI_directory+"/data_odometry_velodyne/dataset/sequences/"+seqNum+"/calib.txt";
        pos_txt_name = KITTI_directory+"/data_odometry_velodyne/dataset/poses/"+seqNum+".txt";
    }
    else{

        cout<<"KITTI_directory目录错误 "<<"bin_path: "<<bin_path <<endl;
        cout<<"KITTI_directory目录错误 "<<"times_txt: "<<times_txt_name <<endl;
        return 1;
    }
    //左相机位姿真值
    vector<Eigen::Matrix4d> pose_list=loadPoses(pos_txt_name);
    //时间戳
    vector<float> time_list=loadTimes(times_txt_name);
    //内参矩阵
    vector<Matrix4d> calib_list=loadCalib(calib_txt_name);
    //Lidar位姿真值
    vector<Matrix4d> lidar_pose_list=getLidarPoses(pose_list,calib_list);

    //.bin激光点云文件列表
    std::vector<std::string> file_lists;
    //读取文件名
    read_filelists(bin_path, file_lists,"bin");
    sort_filelists( file_lists, "bin" );

    size_t dataSize=file_lists.size();
    cout<<"激光点云总帧数："<<dataSize<<endl;
    for (int i = 0; i < file_lists.size(); ++i)
        //for (int i = 0; i < 100; ++i)
    {
        std::string bin_file = bin_path + file_lists[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
        //读数据
        readKittiPclBinData(bin_file, cloudXYZI);

        //转换成ros下的点云数据格式
        //要使用make_shared，不然会运行报错
        sensor_msgs::PointCloud2::Ptr output_msg=boost::make_shared<sensor_msgs::PointCloud2> ();
        //pcl_conversions::fromPCL(*cloud, *output_msg);
        pcl::toROSMsg(*cloudXYZI,*output_msg);
        //设置时间戳
        output_msg->header.stamp=ros::Time().fromSec(time_list[i]);

        //发布ros格式的点云数据
        g_cloud_pub.publish(*output_msg);

        //发布真值
//        cout<<pose_list[i](0,3)<<" "<<pose_list[i](1,3)<<" "<<pose_list[i](2,3)<<endl;
        nav_msgs::Odometry odo=getgroundTruthOdo(pose_list[i]);
        odo.header.stamp = ros::Time().fromSec(time_list[i]);
        odo.header.frame_id = "/camera_init";
        odo.child_frame_id = "/aft_mapped";
        g_lidarpos_pub.publish(odo);

        if(i%100==0)
        {
          int minute=((int)(time_list[i]-time_list[0]))/60;
          float second=time_list[i]-time_list[0]-60*minute;
          cout << "publish第 " << i << " 帧PCD, 持续时间 "<<minute<<":"<<second<<endl;
          cout<<"groundTruth "<<odo.pose.pose.position.x<<" "<<odo.pose.pose.position.y<<" "<<odo.pose.pose.position.z<<endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
