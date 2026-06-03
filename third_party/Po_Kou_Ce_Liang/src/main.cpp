#include "BevelMeasurement.h"

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>

// 两种解算坡口角度和钝边长度方法:plane_fit or direct_points.(config配置文件中设置)
int main()
{
    const std::string cloudPath = "D:/3 Data/11 LanYou_S1/0 Template/Po_Kou_Ce_Liang/src_0.pcd";
    const std::string configPath = "D:/1 自研/15 兰铀算法/测量算法/坡口测量/坡口测量/Po_Kou_Ce_Liang/config.txt";
	//const std::string templateDir = "D:/3 Data/11 LanYou_S1/0 Template/Po_Kou_Ce_Liang/";

    bevel::CloudT::Ptr raw(new bevel::CloudT);
	if (cloudPath.size() >= 4 && cloudPath.substr(cloudPath.size() - 4) == ".pcd")
	{
		if (pcl::io::loadPCDFile<bevel::PointT>(cloudPath, *raw) != 0)
		{
			std::cerr << "Failed to load PCD: " << cloudPath << "\n";
			return 2;
		}
	}
	else if (!bevel::loadTextPointCloud(cloudPath, raw)) 
	{
        std::cerr << "Failed to load TXT point cloud: " << cloudPath << "\n";
        return 2;
    }

    const bevel::BevelMeasurementResult result = bevel::solveBevelFromRawCloud(raw, configPath);
    if (!result.ok) 
	{
        std::cerr << "Solve failed: " << result.message << "\n";
        return 3;
    }

    std::cout << "bevel_type=" << result.bevelType << "\n";
    std::cout << "angle_deg=" << result.angleDeg << "\n";
    std::cout << "length=" << result.length << "\n";
    std::cout << "icp_fitness=" << result.icpFitness << "\n";
    std::cout << "quality_code=" << result.qualityCode << "\n";
    return 0;
}
