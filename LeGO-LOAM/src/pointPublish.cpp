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
#include<vector>
//降采样头文件
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/approximate_voxel_grid.h>
//转换时间戳
#include <pcl_conversions/pcl_conversions.h>
//转换欧拉角为四元数需要的头文件
#include<tf/transform_datatypes.h>
using namespace Eigen;
using namespace std;
using namespace pcl;
template<class Type> Type string2num(const string& str) {

    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}
template<class Type> string num2string(const Type &num) {
    stringstream ss;
    string str;
    ss << num;
    ss >> str;
    return str;
}

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

void read_filelists(const std::string& dir_path, std::vector<std::string>& out_filelsits, std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL) {
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0) {
            out_filelsits.push_back(ptr->d_name);
        }
        else {
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(), type.size());
            if (tmp_cut_type == type) {
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

void rgb2xyzi(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB) {

    int m = cloudRGB->points.size();
    for (int i = 0; i < m; i++) {
        pcl::PointXYZI p;
        p.x = cloudRGB->points[i].y;
        p.y = -cloudRGB->points[i].x;
        p.z = cloudRGB->points[i].z;
        p.intensity = cloudRGB->points[i].g;//r是反射率，g是线数线数
        cloudI->points.push_back(p);
    }
    cloudI->width = m;
    cloudI->height = 1;
}
//加载XYZRGB格式
void readCollectData(std::string &in_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points_xyzi)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read(in_file, *cloudXYZRGB);
    rgb2xyzi(points_xyzi, cloudXYZRGB);
}

////加载XYZI格式
//void readCollectData(std::string &in_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points_xyzi)
//{
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZRGB(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZRGB(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PCDReader reader;
//    reader.read(in_file, *cloudXYZRGB);
//    //转化为loam中velodyne的坐标
//    for(int i=0;i<cloudXYZRGB->points.size();i++)
//    {
//        pcl::PointXYZI p;
//        p.x = cloudXYZRGB->points[i].y;
//        p.y = -cloudXYZRGB->points[i].x;
//        p.z = cloudXYZRGB->points[i].z;
//        //p.intensity = cloudXYZRGB->points[i].r;
//        p.intensity = cloudXYZRGB->points[i].intensity;
//        points_xyzi->points.push_back(p);
//    }
//    points_xyzi->width=cloudXYZRGB->points.size();
//    points_xyzi->height=1;
//    //rgb2xyzi(points_xyzi, cloudXYZRGB);
//}
//读取文件中的一行数据
void loadGps(string gps,vector<float> &gps_vec)
{
    float temp;
    string str,tempstr;
    str=gps;
    //cout<<str<<endl;
    for(int i=0;i<22;i++)
    {
        tempstr=str.substr(0,str.find_first_of(" "));
        str=str.substr(str.find_first_of(" ")+1);
        temp=string2num<float>(tempstr);
        gps_vec.push_back(temp);
        //cout<<temp<<" ";
    }
    //cout<<endl;
}
void readIMUData(sensor_msgs::Imu &imuData,const string &imuFileName,float timeSec)
{
    static float x0=0,y0=0,z0=0;
    static bool count=0;

    ifstream imuFile;
    vector<float> gps_vec;
    string imuString;
    imuFile.open(imuFileName.c_str());
    getline(imuFile,imuString);
    //读取数据到gps_vec中
    loadGps(imuString,gps_vec);

    double x,y,z,roll,pitch,yaw,vx,vy,vz,ax,ay,az,rollv,pitchv,yawv;
    if(count==0)
    {
        x0=gps_vec[1];
        y0=gps_vec[2];
        z0=gps_vec[3];
        count++;
    }
    x=gps_vec[1]-x0;
    y=gps_vec[2]-y0;
    z=gps_vec[3]-z0;

    vx=gps_vec[7];
    vy=gps_vec[8];
    vz=gps_vec[9];

    ax=gps_vec[10]*100;
    ay=gps_vec[11]*100;
    az=gps_vec[12]*100;

    //醉了，这里惯导顺序是roll,pitch,yaw，分别是绕y,x,z轴旋转的结果,注意！！！
    //但是在录激光数据的时候，存放的顺序是yaw pitch roll！！fuck
    //要转换为弧度制

    //惯导下的角度,pitch 是绕x ,roll是绕y，这里做一个转换，变为roll绕x,pitch绕y
    yawv=gps_vec[13]*100;
    pitchv=gps_vec[15]*100;
    rollv=gps_vec[14]*100;
    //还是角度变换的锅！！！
    yaw=M_PI/2 - gps_vec[4] / 180 * M_PI;
    pitch=gps_vec[6] / 180 * M_PI;
    roll=gps_vec[5] / 180 * M_PI;

    //欧拉角转四元数
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);

    //
    imuData.header.stamp=ros::Time().fromSec(timeSec);
    q.normalize();
    imuData.orientation.x=q.getX();
    imuData.orientation.y=q.getY();
    imuData.orientation.z=q.getZ();
    imuData.orientation.w=q.getW();
    //加速度 惯导是x向右，y向前，z向上，需要转换成和KITTI一样的 KITTI惯导x前 ，y左，z上

//    imuData.linear_acceleration.x=ax;
//    imuData.linear_acceleration.y=ay;
//    imuData.linear_acceleration.z=az;

    //这里赋值为位置，直接使用
    imuData.linear_acceleration.x=x;
    imuData.linear_acceleration.y=y;
    imuData.linear_acceleration.z=z;

    //这里赋值为速度
    imuData.angular_velocity.x=vy;
    imuData.angular_velocity.y=vx;
    imuData.angular_velocity.z=vz;

    imuData.angular_velocity.x=rollv;
    imuData.angular_velocity.y=pitchv;
    imuData.angular_velocity.z=yawv;

    imuFile.close();
    cout <<"pos:  "<<x<<" "<<y<<" "<<z<<"   "
        <<"angle:  "<<roll/180*M_PI<<" "<<pitch/180*M_PI<<" "<<yaw/180*M_PI<<"  "
        <<"vel:  "<<vx<<","<<vy<<","<<vz<<" "
       <<"acc:  "<<ax<<","<<ay<<","<<az<<"  "
       <<"angelRate:    "<<rollv<<","<<pitchv<<","<<yawv<<endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointPublish");
    ros::NodeHandle nh("~");

    //文件名后一定要记得加/,不然文件读不到
    std::string pcd_path;
    //        pcd_path= "/media/localization/Localization/cmf/地图/常熟/10/HDL/";
    //        pcd_path="/media/localization/Localization/tu/HDL/";
    ros::param::get("PCD_directory",pcd_path);
    //nh.getParam("PCD_directory",pcd_path);
    cout<<"数据文件夹： "<<pcd_path<<endl;
    ros::Publisher g_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    ros::Publisher g_imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data11", 50);
    ros::Rate loop_rate(10);//频率

    //时间戳
    vector<float> time_list;
    //原始文件名列表
    std::vector<std::string> file_lists;
    //根据前后两帧时间差,处理过后的文件名列表
    std::vector<std::string> sample_file_lists;
    //读取文件名
    read_filelists(pcd_path, file_lists, "pcd");
    sort_filelists(file_lists, "pcd");

    cout << "file_lists size " << file_lists.size() << endl;


    //读取时间到time_list中
    for (int i = 0; i < file_lists.size(); ++i)
    {
        string time_string = file_lists[i].substr(0, file_lists[i].length() - 4)+" ";
        vector<float> time_vec;
        string str, tempstr;
        str = time_string;
        for (int j = 0; j < 8; j++)
        {
            tempstr = str.substr(0, str.find_first_of(" "));

            str = str.substr(str.find_first_of(" ") + 1);

            if(tempstr==" ")
                continue;
            time_vec.push_back(string2num<float>(tempstr));
        }
        float hour = time_vec[4];
        float minute = time_vec[5];
        float second = time_vec[6];
        float uSecond = time_vec[7];
        float timeSeq = hour * 3600 + minute * 60 + second + uSecond/1000;

        if(i==0)
        {
            time_list.push_back(timeSeq);
            sample_file_lists.push_back(file_lists[0]);
        }

        else
        {   //如果前后帧大于0.095
            if(timeSeq - time_list.back()>=0.09)
            {
                time_list.push_back(timeSeq);
                sample_file_lists.push_back(file_lists[i]);
            }
        }
    }
    cout<<"sample_file_lists"<<sample_file_lists.size()<<endl;

    //定义起始的时间
    float publish_time=time_list[0];
    //开始发布消息
    for (int i = 0; i < sample_file_lists.size() && ros::ok(); ++i)
        //for (int i = 0; i < 100; ++i)
    {
        float startTime=clock();
        std::string pcd_file = pcd_path + sample_file_lists[i];
        ///////////////////////////////////////////////////////////
        //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
        sensor_msgs::Imu imuData;
        //读数据
        pcl::PCDReader reader;
        //reader.read(pcd_file, *cloudXYZI);
        readCollectData(pcd_file, cloudXYZI);
        string imuFileName=pcd_path+"../IMU/"+file_lists[i].substr(0, file_lists[i].length() - 4)+".txt";

        //转换成rosData
        //要加上,不然会运行报错
        sensor_msgs::PointCloud2::Ptr output_msg = boost::make_shared<sensor_msgs::PointCloud2>();

        //pcl_conversions::fromPCL(*cloud, *output_msg);
        pcl::toROSMsg(*cloudXYZI, *output_msg);
        //设置时间戳
        output_msg->header.stamp = ros::Time().fromSec(time_list[i]);
        //发布output_msg
        if(time_list[i]-publish_time<0.09)
        {
            cout<<"发布IMU: "<<i<<"   ";
            readIMUData(imuData,imuFileName,time_list[i]);
            //usleep(100);
            g_imu_pub.publish(imuData);
            continue;
        }
        else
        {
            if(cloudXYZI->points.size()<=0 )
            {
                cout<<" "<<i<<"------->点数为空"<<endl;
                continue;
            }
            readIMUData(imuData,imuFileName,time_list[i]);
            g_cloud_pub.publish(*output_msg);
            g_imu_pub.publish(imuData);
            publish_time=time_list[i];
        }
        //ros::spinOnce();
        loop_rate.sleep();
        cout <<"发布点云 : "<<i<<"    point_size: "<<cloudXYZI->points.size()<<"    time: "<<num2string(time_list[i])<<
               "    related: "<<time_list[i]-time_list[0]<<"  真实频率: "<<(clock()-startTime)/CLOCKS_PER_SEC<<endl;
    }
    return 0;
}


