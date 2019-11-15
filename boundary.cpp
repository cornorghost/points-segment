#include"pch.h"
#include "common.h"

#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

int drawBound(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundPoints) {
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normEst;  //其中pcl::PointXYZRGBA表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(400);  //法向估计的点数
	normEst.compute(*normals);
	std::cout << "normal size is " << normals->size() << std::endl;

	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
	est.setInputCloud(cloud);
	est.setInputNormals(normals);
	//est.setAngleThreshold(90);
	est.setSearchMethod(tree);
	//est.setKSearch(300);  //一般这里的数值越高，最终边界识别的精度越好
	est.setRadiusSearch(8.0);  //搜索半径
	est.compute(boundaries);

	pcl::PointCloud<pcl::PointXYZRGBA> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			boundPoints->push_back(cloud->points[i]);
			countBoundaries++;
		}
		else
			noBoundPoints.push_back(cloud->points[i]);

	}
	std::cout << "boudary size is：" << countBoundaries << std::endl;

	pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
	pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);

	return 1;
}