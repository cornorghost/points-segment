#include "pch.h"
#include"common.h"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<iostream>

//radius
int radiusFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, int radius, int neighbor) {
	std::cout << "�뾶�˲�" << "\n";
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;  //�����˲���
	sor.setInputCloud(cloudInput);    //�����������
	sor.setRadiusSearch(radius);     //���ð뾶Ϊ�ķ�Χ�����ٽ��� 6
	sor.setMinNeighborsInRadius(neighbor); //���ò�ѯ�������㼯��С�ڵ�ɾ�� 14
	sor.filter(*cloudInput);
	std::cout << "�˲��������" << cloudInput->size() << "\n";
	return 1;
}

//volxel
int voxelFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, float x, float y, float z) {
	std::cout << "�����˲�" << "\n";
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(cloudInput);
	sor.setDownsampleAllData(true);
	sor.setLeafSize(x, y, z);//���ش�С 1.0 3.0 1.0
	sor.filter(*cloudInput);
	std::cout << "�˲��������" << cloudInput->size() << "\n";
	return 1;
}

//statistic
int statisticalFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, float sizeLimit, int mean, float thresh) {
	int size = cloudInput->size();
	while (cloudInput->size() >= size * sizeLimit) { //0.6
		std::cout << "ͳ���˲�" << "\n";
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloudInput);
		sor.setMeanK(mean);       //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ����� 60
		sor.setStddevMulThresh(thresh);   //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ 1.2
		sor.filter(*cloudInput);
		std::cout << "�˲��������" << cloudInput->size() << "\n";
	}
	return 1;
}