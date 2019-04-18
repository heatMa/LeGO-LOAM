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
#include <nav_msgs/Odometry.h>
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

//原点相对坐标
float g_originX;
float g_originY;
float g_originZ;

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
        //录的数据是y前 x右 z上，转过后是x前，y左，z上
        p.x = cloudRGB->points[i].y;
        p.y = -cloudRGB->points[i].x;
        p.z = cloudRGB->points[i].z;
        p.intensity = cloudRGB->points[i].g;
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
    cout<<"惯导位置"<<gps_vec[1]-x0<<" "<<gps_vec[2]-y0<<" "<<gps_vec[3]-z0<<endl;

    //这里赋值都忘了吗？日了狗了
    x=gps_vec[1];
    y=gps_vec[2];
    z=gps_vec[3];

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

    imuData.linear_acceleration.x=ax;
    imuData.linear_acceleration.y=ay;
    imuData.linear_acceleration.z=az;

    imuData.angular_velocity.x=rollv;
    imuData.angular_velocity.y=pitchv;
    imuData.angular_velocity.z=yawv;


    //利用eigen从矩阵获得四元数
    Eigen::Quaterniond quat;
    //std::cout<<"tempVel[5]: "<<tempVel[5]<<std::endl;   //Eigen这个数组越界没有内存警告牛逼了onf quat;
    quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    quat.normalize();


    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    //当前位姿变换矩阵表示
    T.pretranslate(Eigen::Vector3d(x,y,z));
    T.rotate(quat);
    //这两种赋值都可以
    //T.rotate(quat.toRotationMatrix());

    //Lidar相对惯导位置
    float L=1.2967;//前
    float W=-0.0795;//左
    float H=0.5389;//上
    //转换成向量
    Eigen::Vector3d lidar_pos=T * Eigen::Vector3d(L,W,H);
    x=lidar_pos[0]-g_originX;
    y=lidar_pos[1]-g_originY;
    z=lidar_pos[2]-g_originZ;

    //位置、速度、朝向真值都保存在这里
    imuData.orientation_covariance[0]=x;
    imuData.orientation_covariance[1]=y;
    imuData.orientation_covariance[2]=z;

    imuData.orientation_covariance[3]=vx;
    imuData.orientation_covariance[4]=vy;
    imuData.orientation_covariance[5]=vz;

    imuData.orientation_covariance[6]=roll;
    imuData.orientation_covariance[7]=pitch;
    imuData.orientation_covariance[8]=yaw;


    imuFile.close();
    cout <<"pos:  "<<x<<" "<<y<<" "<<z<<"   "
        <<"angle °:  "<<roll*180/3.14<<" "<<pitch*180/3.14<<" "<<yaw*180/3.14<<"  "
        <<"vel:  "<<vx<<","<<vy<<","<<vz<<" "
       <<"acc:  "<<ax<<","<<ay<<","<<az<<"  "
       <<"angelRate:    "<<rollv<<","<<pitchv<<","<<yawv<<endl;
}


void getLidarInitPos(string originFileName)
{
    ifstream imuFile;
    string imuString;
    vector<float> gps_vec;
    imuFile.open(originFileName.c_str());
    getline(imuFile,imuString);
    //读取数据到gps_vec中
    loadGps(imuString,gps_vec);

    //还是角度变换的锅！！！
    float yaw=M_PI/2 - gps_vec[4] / 180 * M_PI;
    float pitch=gps_vec[6] / 180 * M_PI;
    float roll=gps_vec[5] / 180 * M_PI;
    //欧拉角转四元数

    //利用eigen从矩阵获得四元数
    Eigen::Quaterniond quat;
    //std::cout<<"tempVel[5]: "<<tempVel[5]<<std::endl;   //Eigen这个数组越界没有内存警告牛逼了onf quat;
    quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    quat.normalize();

    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    //当前位姿变换矩阵表示
    T.pretranslate(Eigen::Vector3d(gps_vec[1],gps_vec[2],gps_vec[3]));
    T.rotate(quat);
    //这两种赋值都可以
    //T.rotate(quat.toRotationMatrix());

    //Lidar相对惯导位置
    float L=1.2967;//前
    float W=-0.0795;//左
    float H=0.5389;//上
    Eigen::Vector3d lidar_pos=T * Eigen::Vector3d(L,W,H);
    g_originX=lidar_pos[0];
    g_originY=lidar_pos[1];
    g_originZ=lidar_pos[2];

    cout<<"初始惯导位置"<<gps_vec[1]<<","<<gps_vec[2]<<","<<gps_vec[3]<<endl;
    cout<<"初始激光位置"<<g_originX<<","<<g_originY<<","<<g_originZ<<endl;
    //getchar();

}

ros::Publisher RAW_IMUpos_pub;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointPublish");
    ros::NodeHandle nh("~");

    RAW_IMUpos_pub=nh.advertise<nav_msgs::Odometry>("/2RAWIMUodom",5,false);
    
    //文件名后一定要记得加/,不然文件读不到
    std::string pcd_path;
    //        pcd_path= "/media/localization/Localization/cmf/地图/常熟/10/HDL/";
    //        pcd_path="/media/localization/Localization/tu/HDL/";
    ros::param::get("PCD_directory",pcd_path);
    //nh.getParam("PCD_directory",pcd_path);
    cout<<"数据文件夹： "<<pcd_path<<endl;
    ros::Publisher g_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    ros::Publisher g_imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/gps", 50);
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
            if(timeSeq - time_list.back()>=0.00)
            {
                time_list.push_back(timeSeq);
                sample_file_lists.push_back(file_lists[i]);
            }
        }
    }
    cout<<"sample_file_lists"<<sample_file_lists.size()<<endl;

    //读取第一帧IMU的坐标，此后坐标都相对于这个坐标
    string originFileName=pcd_path+"../IMU/"+file_lists[0].substr(0, file_lists[0].length() - 4)+".txt";
    getLidarInitPos(originFileName);
    //定义起始的时间
    float publish_time=time_list[000];
    //开始发布消息
    for (int i = 000; i < sample_file_lists.size() && ros::ok(); ++i)
        //for (int i = 0; i < 100; ++i)
    {
        cout<<"i:"<<i<<endl;
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
        //新加的！！！！！
        output_msg->header.frame_id="velodyne64_link";
        //发布output_msg
        if(time_list[i]-publish_time<0.09)
        {
            cout<<"发布IMU: "<<i<<"   ";
            readIMUData(imuData,imuFileName,time_list[i]);
            imuData.header.frame_id="imu_link";
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
            imuData.header.frame_id="imu_link";
            g_cloud_pub.publish(*output_msg);
            g_imu_pub.publish(imuData);
            publish_time=time_list[i];
        }
        //ros::spinOnce();
        loop_rate.sleep();
        cout <<"发布点云 : "<<i<<"    point_size: "<<cloudXYZI->points.size()<<"    time: "<<num2string(time_list[i])<<
               "    related: "<<time_list[i]-time_list[0]<<"  真实频率: "<<(clock()-startTime)/CLOCKS_PER_SEC<<endl;
    }
    cout<<"发布点云结束"<<endl;
    return 0;
}


typedef float T;
    typedef Eigen::Matrix<T, 3, 1> Vector3t;
    typedef Eigen::Matrix<T, 4, 4> Matrix4t;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
    typedef Eigen::Quaternion<T> Quaterniont;

void f(const VectorXt &control,double dt,VectorXt state_){
    float roll, pitch, yaw;
    VectorXt next_state(9);
    Vector3t pt = state_.middleRows(0, 3);
    Vector3t vt = state_.middleRows(3, 3);
    roll = state_[6]; pitch = state_[7]; yaw = state_[8];

    Vector3t raw_acc = control.middleRows(0, 3);
    Vector3t raw_gyro = control.middleRows(3, 3);
    //矫正acc
    Vector3t acc=Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * raw_acc;

    // position
    next_state.middleRows(0, 3) = pt + vt * dt+ 0.5 * acc *dt *dt;					//
    // velocity
    next_state.middleRows(3, 3) = vt + acc * dt;		// acceleration didn't contribute to accuracy due to large noise
    // orientation
    Vector3t next_angles = Vector3t(roll,pitch,yaw) + raw_gyro * dt;
    next_state.middleRows(6, 3) = next_angles;
    state_=next_state;

    //转换为ROS Odometry格式数据发布出来
    nav_msgs::Odometry odom;
    //odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    //利用eigen从矩阵获得四元数
    Eigen::Quaternionf quat;
    quat = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    quat.normalize();
    //转换为ROS下的四元数
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();
    //转换为ROS Odometry.msg，转换的时候把速度方向变一下，中间不能变！！！
    odom.pose.pose.position.x = next_state(1);
    odom.pose.pose.position.y = -next_state(0);
    odom.pose.pose.position.z = next_state(2);
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "velodyne";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    RAW_IMUpos_pub.publish(odom);

}

