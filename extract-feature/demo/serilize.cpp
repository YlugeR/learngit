#include <pcl_conversions/pcl_conversions.h>
#include "module_manager/module_manager.h"
#include "seg_extract_based_box.h"
#include "dirManager.h"

using namespace Robosense;

ModuleManager *module_manager;

int main(int argc, char** argv)
{
    // check versions
    COUTG("OpenCV version: "<<CV_VERSION);
    COUTG("ROS version: "<<ROS_VERSION_MAJOR<<"."<<ROS_VERSION_MINOR<<"."<<ROS_VERSION_PATCH);
    COUTG("PCL version: "<<PCL_VERSION_PRETTY);
    COUTG("Boost version: "<<BOOST_LIB_VERSION);
    COUTG("Eigen version: "<<EIGEN_WORLD_VERSION<<"."<<EIGEN_MAJOR_VERSION<<"."<<EIGEN_MINOR_VERSION);

    ros::init(argc, argv, "test_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~"); //parameter node

    int port;
    std::string key_path, pcap_path, eth_name, lidar_config_path, args_path, pcd_label_path, save_path;
    // 从launch文件读取全局参数
    private_nh.param("key_path", key_path, std::string(""));
    private_nh.param("pcap", pcap_path, std::string(""));
    private_nh.param("ethernet_name", eth_name, std::string(""));
    private_nh.param("lidar_config_path", lidar_config_path, std::string(""));
    private_nh.param("args_path", args_path, std::string(""));
    private_nh.param("pcd_label_path", pcd_label_path, std::string(""));
    private_nh.param("save_path", save_path, std::string(""));

    // 授权module
    module_manager = new ModuleManager(key_path, pcap_path, eth_name, port);

    // 输入打标数据所在的文件夹路径,获得当前路径下所有的打标文件的文件名,存入pcap_vec
    dirManager file_manager;
    std::vector<std::string> pcap_vec, file_vec;
    file_manager.getDirAndName(pcd_label_path,pcap_vec);
    file_manager.getNameInDir(pcd_label_path,file_vec);

    ExtractSeg es = ExtractSeg();

    // // 遍历每一个打标文件，解析标签，扣出分割点云，计算特征
    for(int i = 0; i < pcap_vec.size(); i++)
    {
        // int i= 0;
        // pcap_vec[i] = "/home/leezonpen/data/RS-32/32_liuxiandadao_to_shahexilu_11805081815";
        //step 1.输入打标文件路径，解析标签，解析信息存储在类sf中
         SerilizeLabelFile sf;
        // sf.serilizeData_new(pcap_vec[i]);
        sf.serilizeData(pcap_vec[i]);

        //step 2.根据解析标签对象sf,扣出分割点云segments,存储于label_frames中
        // Robosense::FrameLabelInfo表示一帧点云中所有的打标segments对应的信息
        std::vector< FrameLabelInfo> label_frames;
        es.initConfigPath(args_path, lidar_config_path);
        es.extractSegBag(sf,label_frames);
        std::cout<<"extract segments finish!"<<std::endl;

        //step 3.对step 2中解析出的点云segments计算其特征，并写入txt
        //每一个打标文件对应的特征存储文件名
        std::string feature_file = save_path + file_vec[i] + ".txt";
        es.saveFeatureToCSV(feature_file,label_frames);
        std::cout<<"compute feature finish!"<<std::endl;
    }
    // std::cout<<"Finished."<<std::endl;

    return 0;
}




