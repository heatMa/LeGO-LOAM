#include <iostream>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
//降采样的头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace  std;



void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0,0,1);
}


int
 main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_below3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_groundDS(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    // 读入点云PCD文件
    reader.read("/media/localization/新加卷/school/siyuan2/HDL/2019 01 06  22 29 55 085.pcd",*cloud);

    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //电云中点的索引
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);

    //提取3m以下的点
    seg.setDistanceThreshold (3);
    seg.setInputCloud (cloud);
    //这里相当于把点的索引算出来了，存入了inliers中
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cout<<"平面模型参数："<<*coefficients<<std::endl;

    // 保存3m以下的点到cloud_below3中
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*cloud_below3);

    std::cerr << "低于三米的点 after filtering: " << std::endl;
    std::cerr << *cloud_below3 << std::endl;


    // 提取地面0.2m以下的点
    seg.setDistanceThreshold (0.2);
    seg.setInputCloud (cloud_below3);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }


    // 提取除地面外的物体
    extract.setInputCloud (cloud_below3);
    extract.setIndices (inliers);
    extract.filter (*cloud_ground);
    std::cout<<"地面点大小："<<cloud_ground->points.size()<<std::endl;

    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    std::cerr << "3m下的非地面点 after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_resultDS(new pcl::PointCloud<pcl::PointXYZ>);
    //降采样
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.05f,0.05f,0.5f);
    sor.filter(*cloud_resultDS);
    std::cout<<"降采样后3m下点："<<cloud_resultDS->points.size()<<std::endl;

    sor.setInputCloud(cloud_ground);
    sor.setLeafSize(0.05f,0.05f,0.5f);
    sor.filter(*cloud_groundDS);
    std::cout<<"降采样前地面点："<<cloud_ground->points.size()<<std::endl;
    std::cout<<"降采样后地面点："<<cloud_groundDS->points.size()<<std::endl;


    //kd Tree 搜索最近点
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
    kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtree->setInputCloud(cloud_resultDS);



    pcl::PointXYZ searchPoint;
    for(size_t i=0;i<cloud_resultDS->points.size();++i)
    {
         searchPoint=cloud_groundDS->points[i];
         vector<int> pointIdxRadiusSearch;
         vector<float> pointRadiusSquaredDistance;
         kdtree->radiusSearch(searchPoint, 3.5, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0);
         pcl::PointCloud<pcl::PointXYZ> aaa;
         for(size_t i=0;i<pointIdxRadiusSearch.size();i++)
         {
             cout<<cloud_resultDS->points[pointIdxRadiusSearch[i]].x<<" "
                     <<cloud_resultDS->points[pointIdxRadiusSearch[i]].y<<" "
                     <<cloud_resultDS->points[pointIdxRadiusSearch[i]].z<<" "
                     <<pointRadiusSquaredDistance[pointIdxRadiusSearch[i]]<<endl;
         }

    }





    // 点云可视化
    pcl::visualization::CloudViewer viewer("Filtered");
    viewer.showCloud(cloud_resultDS);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    while(!viewer.wasStopped()){

    }
    return (0);
}
