#include"pch.h"
#include "common.h"
#include <direct.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#define BOOST_TYPEOF_EMULATION
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/segmentation/supervoxel_clustering.h>  
#include <pcl/segmentation/lccp_segmentation.h>  
#include <fstream>
#define Random(x) (rand() % x)

typedef pcl::LCCPSegmentation<pcl::PointXYZRGBA>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

int extract(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string direct_name) {
	//超体聚类
	float voxel_resolution = 8.0f; //体素立方体边长，建议大小5.0 8.0
	float seed_resolution = 3.0f; //体核边长，建议值4.0 3.0
	float color_importance = 0.0f; //颜色比重
	float spatial_importance = 1.0f; //1.0
	float normal_importance = 1.0f; //法线比重1.0
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;

	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);

	PCL_INFO("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	pcl::PointCloud<pcl::PointXYZL>::Ptr overseg = super.getLabeledCloud();
	//std::ofstream outFile1("过分割3.txt", std::ios_base::out);
	//for (int i = 0; i < overseg->size(); i++) {
	//	outFile1 << overseg->points[i].x << "\t" << overseg->points[i].y << "\t" << overseg->points[i].z << "\t" << overseg->points[i].label << std::endl;
	//}
	int label_max1 = 0;
	for (int i = 0; i < overseg->size(); i++) {
		if (overseg->points[i].label > label_max1)
			label_max1 = overseg->points[i].label;
	}

	//超聚结果
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	//ColoredCloud1->height = 1;
	//ColoredCloud1->width = overseg->size();
	//ColoredCloud1->resize(overseg->size());
	//for (int i = 0; i < label_max1; i++) {
	//	int color_R = Random(255);
	//	int color_G = Random(255);
	//	int color_B = Random(255);

	//	for (int j = 0; j < overseg->size(); j++) {
	//		if (overseg->points[j].label == i) {
	//			ColoredCloud1->points[j].x = overseg->points[j].x;
	//			ColoredCloud1->points[j].y = overseg->points[j].y;
	//			ColoredCloud1->points[j].z = overseg->points[j].z;
	//			ColoredCloud1->points[j].r = color_R;
	//			ColoredCloud1->points[j].g = color_G;
	//			ColoredCloud1->points[j].b = color_B;
	//		}
	//	}
	//}
	//pcl::io::savePCDFileASCII("过分割3.pcd", *ColoredCloud1);



	//LCCP分割  
	unsigned int k_factor = 8; //邻域值，建议8
	float concavity_tolerance_threshold = 20; //公差值，建议22
	float smoothness_threshold = 2.0; //阈值，建议2.0
	uint32_t min_segment_size = 4000; //最小分割点云大小建议，4000
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;
	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<pcl::PointXYZRGBA> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();

	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");

	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);

	//std::ofstream outFile2("分割后合并3.txt", std::ios_base::out);
	//for (int i = 0; i < lccp_labeled_cloud->size(); i++) {
	//	outFile2 << lccp_labeled_cloud->points[i].x << "\t" << lccp_labeled_cloud->points[i].y << "\t" << lccp_labeled_cloud->points[i].z << "\t" << lccp_labeled_cloud->points[i].label << std::endl;
	//}

	int label_max2 = 0;
	for (int i = 0; i < lccp_labeled_cloud->size(); i++) {
		if (lccp_labeled_cloud->points[i].label > label_max2)
			label_max2 = lccp_labeled_cloud->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	coloredCloud2->height = 1;
	coloredCloud2->width = lccp_labeled_cloud->size();
	coloredCloud2->resize(lccp_labeled_cloud->size());
	for (int i = 0; i < label_max2; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);

		//提取
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>); //保存分割出来的文件
		for (int j = 0; j < lccp_labeled_cloud->size(); j++) {
			if (lccp_labeled_cloud->points[j].label == i) {
				coloredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
				coloredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
				coloredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
				coloredCloud2->points[j].r = color_R;
				coloredCloud2->points[j].g = color_G;
				coloredCloud2->points[j].b = color_B;

				pcl::PointXYZRGBA point;
				point.x = lccp_labeled_cloud->points[j].x;
				point.y = lccp_labeled_cloud->points[j].y;
				point.z = lccp_labeled_cloud->points[j].z;
				point.rgba = lccp_labeled_cloud->points[j].label;
				cloud_cluster->push_back(point);
			}
		}

		//创建文件夹并保存
		if (cloud_cluster->size() >= 3000 && cloud_cluster->size() <= 30000) { //点云数目大于4000的才保存
			std::stringstream folderPath;
			folderPath << direct_name << "\\cluster_cloud_" << i;

			if (0 != access(folderPath.str().c_str(), 0)) //如果不存在，创建一个文件夹
			{
				mkdir(folderPath.str().c_str()); 

			}
			std::stringstream ss;
			ss << folderPath.str() << "\\cluster_cloud_" << i << ".pcd";
			std::stringstream ss1;
			ss1 << direct_name << "\\cluster_cloud_" << i << ".pcd";
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			pcl::io::savePCDFile(ss.str(), *cloud_cluster);
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			pcl::io::savePCDFile(ss1.str(), *cloud_cluster);
		}

	}
	pcl::io::savePCDFileASCII("整体点云分割后合并.pcd", *coloredCloud2);
	return 1;
}