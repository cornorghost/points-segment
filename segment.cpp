#include "pch.h"
#include"common.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

int seg(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ext_cloud) {
	//ƽ��ָ�
	pcl::SACSegmentation<pcl::PointXYZRGBA> sac;
	pcl::PointIndices::Ptr inliner(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sac_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	sac.setInputCloud(cloud);
	sac.setMethodType(pcl::SAC_RANSAC); //�ָʽ
	sac.setModelType(pcl::SACMODEL_PLANE);  //ƽ��ģ��
	sac.setMaxIterations(10); //����������
	sac.setDistanceThreshold(0.01); //���þ�����ֵ

	std::cout << "set ok" << "\n";

	//��ȡƽ��ͷ�ƽ��
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ext_cloud_rest(new pcl::PointCloud<pcl::PointXYZRGBA>); //��ƽ��

	int i = cloud->size(), j = 0;
	pcl::ExtractIndices<pcl::PointXYZRGBA>ext;
	while (cloud->size() > i*0.8)//����ȡ�ĵ���С��������ʱ������ѭ��
	{
		std::cout << cloud->size() << "\n";
		ext.setInputCloud(cloud);
		sac.segment(*inliner, *coefficients);
		if (inliner->indices.size() == 0)
		{
			break;
		}
		//����������ȡ����
		ext.setIndices(inliner);
		ext.setNegative(false);
		ext.filter(*ext_cloud);
		ext.setNegative(true);
		ext.filter(*ext_cloud_rest);
		
		*cloud = *ext_cloud_rest;

	}

	std::cout << "ƽ��ָ�����������" << cloud->size()<<"\n";

}