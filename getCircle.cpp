#include "pch.h"
#include"common.h"

#include <vector>
#include <pcl/common/common.h>


//��ȡ�ļ���
void GetAllFiles(std::vector<std::string>& files,std::string direct_name)
{
	_int64 hFile = 0;
	//�ļ���Ϣ    
	struct _finddata_t fileinfo;
	std::string p;
	std::string name;
	if ((hFile = _findfirst(p.assign(direct_name).append("\\*.pcd").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
			{
				name = fileinfo.name;
				//name.erase(name.end() - 4, name.end());
				files.push_back(name);
				//cout << fileinfo.attrib;
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

//���ĵ�
int getCenter(pcl::PointXYZRGBA *&minpt,pcl::PointXYZRGBA *&maxpt, pcl::PointXYZRGBA *&center) {
	center->x = (minpt->x + maxpt->x) / 2;
	center->y = (minpt->y + maxpt->y) / 2;
	center->z = (minpt->z + maxpt->z) / 2;
	return 1;
}

//��Χ��
int getBound(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA *&minpt, pcl::PointXYZRGBA *&maxpt) {
	//���崢�漫ֵ��������
	pcl::getMinMax3D(*cloud, *minpt, *maxpt);
	//������
	std::cout << "max:x" << maxpt->x << std::endl;
	std::cout << "max:y" << maxpt->y << std::endl;
	std::cout << "max:z" << maxpt->z << std::endl;
	std::cout << "min:x" << minpt->x << std::endl;
	std::cout << "min:y" << minpt->y << std::endl;
	std::cout << "min:z" << minpt->z << std::endl;
	return 1;
}

//ת��
int getCircle(std::string direct_name) {
	std::vector<std::string> files; //���ڴ洢pcd�ļ���
	GetAllFiles(files, direct_name);

	//����ÿ���ļ�
	for (int i = 0; i < files.size(); i++) {
		std::string file_name = direct_name + "\\" + files[i]; //�ļ�·��
		std::string dir_name = direct_name + "\\" + files[i].substr(0, files[i].rfind("."));  //��Ӧ����洢���ļ���
		std::cout << "�ļ�����" << file_name << "\n";
		std::cout << "�ļ�������" << dir_name << "\n";
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::io::loadPCDFile(file_name, *cloud);

		pcl::PointXYZRGBA *minpt = new pcl::PointXYZRGBA;
		pcl::PointXYZRGBA *maxpt = new pcl::PointXYZRGBA;
		pcl::PointXYZRGBA *center=new pcl::PointXYZRGBA;
		getBound(cloud, minpt, maxpt);
		std::cout << "getBound" <<"\n";
		getCenter(minpt, maxpt, center);
		std::cout << "getCenter" << "\n";

		//��ȡת��
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); //���Ͻ�
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); //���Ͻ�
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr up_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); //�ϰ벿��
		if (cloud->size() >= 7000) {//�����������
			for (int j = 0; j < cloud->size(); j++) {
				if (cloud->points[j].x > center->x &&cloud->points[j].z > center->z) { //���
					pcl::PointXYZRGBA cloud_temp;
					cloud_temp.x = cloud->points[j].x;
					cloud_temp.y = cloud->points[j].y;
					cloud_temp.z = cloud->points[j].z;
					cloud_temp.rgba = cloud->points[j].z;
					left_cloud->push_back(cloud_temp);
				}
				else if (cloud->points[j].x < center->x &&cloud->points[j].z < center->z)
				{
					pcl::PointXYZRGBA cloud_temp;
					cloud_temp.x = cloud->points[j].x;
					cloud_temp.y = cloud->points[j].y;
					cloud_temp.z = cloud->points[j].z;
					cloud_temp.rgba = cloud->points[j].z;
					right_cloud->push_back(cloud_temp);
				}
			}

			std::stringstream ss_left;
			std::stringstream ss_right;
			ss_left << dir_name << "\\left.pcd";
			ss_right << dir_name << "\\right.pcd";
			left_cloud->width = left_cloud->points.size();
			left_cloud->height = 1;
			left_cloud->is_dense = true;
			right_cloud->width = right_cloud->points.size();
			right_cloud->height = 1;
			right_cloud->is_dense = true;
			pcl::io::savePCDFile(ss_left.str(), *left_cloud); //��
			std::cout << "save left" << "\n";
			pcl::io::savePCDFile(ss_right.str(), *right_cloud); //��
			std::cout << "save right" << "\n";
		}
		else        //������
		{
			for (int j = 0; j < cloud->size(); j++) {
				if (cloud->points[j].x > center->x) {
					pcl::PointXYZRGBA cloud_temp;
					cloud_temp.x = cloud->points[j].x;
					cloud_temp.y = cloud->points[j].y;
					cloud_temp.z = cloud->points[j].z;
					cloud_temp.rgba = cloud->points[j].z;
					up_cloud->push_back(cloud_temp);
				}
			}
			up_cloud->width = up_cloud->points.size();
			up_cloud->height = 1;
			up_cloud->is_dense = true;
			std::stringstream ss_up;
			ss_up << dir_name << "\\up.pcd";
			std::cout << ss_up.str() << "\n";
			pcl::io::savePCDFile(ss_up.str(), *up_cloud); //��
			std::cout << "save up" << "\n";
		}
	}
}