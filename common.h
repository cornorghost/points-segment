#pragma once
#ifndef COMMON_H_
#define COMMON_H_

#include <pcl/io/pcd_io.h>

int radiusFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, int radius, int neighbor);
int voxelFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, float x, float y, float z);
int statisticalFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput, float sizeLimit, int mean, float thresh);

int seg(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ext_cloud);

int extract(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string direct_name);

int drawBound(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundPoints);

int getCircle(std::string direct_name);

#endif