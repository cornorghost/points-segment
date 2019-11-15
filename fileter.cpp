#include "pch.h"
#include"common.h"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<iostream>

//radius
int radiusFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, int radius, int neighbor) {
	std::cout << "半径滤波" << "\n";
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;  //创建滤波器
	sor.setInputCloud(cloudInput);    //设置输入点云
	sor.setRadiusSearch(radius);     //设置半径为的范围内找临近点 6
	sor.setMinNeighborsInRadius(neighbor); //设置查询点的邻域点集数小于的删除 14
	sor.filter(*cloudInput);
	std::cout << "滤波后点云数" << cloudInput->size() << "\n";
	return 1;
}

//volxel
int voxelFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, float x, float y, float z) {
	std::cout << "体素滤波" << "\n";
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(cloudInput);
	sor.setDownsampleAllData(true);
	sor.setLeafSize(x, y, z);//体素大小 1.0 3.0 1.0
	sor.filter(*cloudInput);
	std::cout << "滤波后点云数" << cloudInput->size() << "\n";
	return 1;
}

//statistic
int statisticalFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, float sizeLimit, int mean, float thresh) {
	int size = cloudInput->size();
	while (cloudInput->size() >= size * sizeLimit) { //0.6
		std::cout << "统计滤波" << "\n";
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloudInput);
		sor.setMeanK(mean);       //设置在进行统计时考虑查询点临近点数 60
		sor.setStddevMulThresh(thresh);   //设置判断是否为离群点的阀值 1.2
		sor.filter(*cloudInput);
		std::cout << "滤波后点云数" << cloudInput->size() << "\n";
	}
	return 1;
}