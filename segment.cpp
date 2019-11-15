#include "pch.h"
#include"common.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

int seg(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ext_cloud) {
	//平面分割
	pcl::SACSegmentation<pcl::PointXYZRGBA> sac;
	pcl::PointIndices::Ptr inliner(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sac_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	sac.setInputCloud(cloud);
	sac.setMethodType(pcl::SAC_RANSAC); //分割方式
	sac.setModelType(pcl::SACMODEL_PLANE);  //平面模型
	sac.setMaxIterations(10); //最大迭代次数
	sac.setDistanceThreshold(0.01); //设置距离阈值

	std::cout << "set ok" << "\n";

	//提取平面和非平面
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ext_cloud_rest(new pcl::PointCloud<pcl::PointXYZRGBA>); //非平面

	int i = cloud->size(), j = 0;
	pcl::ExtractIndices<pcl::PointXYZRGBA>ext;
	while (cloud->size() > i*0.8)//当提取的点数小于总数的时，跳出循环
	{
		std::cout << cloud->size() << "\n";
		ext.setInputCloud(cloud);
		sac.segment(*inliner, *coefficients);
		if (inliner->indices.size() == 0)
		{
			break;
		}
		//按照索引提取点云
		ext.setIndices(inliner);
		ext.setNegative(false);
		ext.filter(*ext_cloud);
		ext.setNegative(true);
		ext.filter(*ext_cloud_rest);
		
		*cloud = *ext_cloud_rest;

	}

	std::cout << "平面分割后点云数量：" << cloud->size()<<"\n";

}