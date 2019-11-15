#include "pch.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "common.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::visualization::PCLVisualizer viewer;

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("xiangti.pcd", *cloud) == -1)	//读入点云
	{
		PCL_ERROR("COULD NOT READ FILE\n");
	}
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> init(cloud, 50, 50, 50); //灰色
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, init, "cloud");//添加显示

	std::cout << "points sieze is:" << cloud->size() << std::endl;

	//下采样处理
	voxelFilter(cloud, 1.0, 3.0, 1.0); //体素大小 1.0 3.0 1.0
	statisticalFilter(cloud, 0.6, 60, 1.2); //限制大小参数0.6，邻域值60，阈值1.2
	radiusFilter(cloud, 6, 14);//半径6，邻域值14
	pcl::io::savePCDFile("filtered.pcd",*cloud);//保存处理后点云
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> filted_color(cloud, 255, 255, 1); //黄色
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, filted_color, "filted");//添加处理后显示


	////分割去除点云平面
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr surface(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//seg(cloud, surface);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> surface_color(surface, 255, 0, 0); //红色
	//viewer.addPointCloud<pcl::PointXYZRGBA>(surface, surface_color, "surface");//添加分割后显示

	////画边界
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//drawBound(cloud, boundPoints);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> bound_color(boundPoints, 0, 0, 255); //蓝色
	//viewer.addPointCloud<pcl::PointXYZRGBA>(boundPoints, bound_color, "boundPoints");//画边界后显示


	//提取点云（每个加强筋）
	std::string dir = "G:\\点云边界提取项目\\边界提取项目\\边界提取项目\\提取";//指定提取文件夹路径
	//清空文件夹
	extract(cloud, dir);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cluster_color(cloud, 0, 255, 0); //绿色
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, cluster_color, "cluster");

	//提取转角
	getCircle(dir);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	system("pause");
}
