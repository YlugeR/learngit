#ifndef LABEL_MANAGER_H
#define LABEL_MANAGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include "parse.h"

namespace Robosense {

  class LabelDataManager
  {
  public:
    LabelDataManager(){}
    ~LabelDataManager(){}

    void serialize(const std::string & filename,const std::vector<FrameLabelInfo>& frames);
    void serialize_pcd(const std::string & filename,const std::vector<FrameLabelInfo>& frames);
    bool deserialize(const std::string & filename, std::vector<FrameLabelInfo>& frames);
    bool deserialize_pcd(const std::string & filename, std::vector<SegLabelInfo>& segments);
    bool deserialize_pcd_class(const std::string & filename, const std::string& label, std::vector<SegLabelInfo>& segments);

  };

  void publishSegCloud(const ros::Publisher& pub,const std::vector<FrameLabelInfo>& frames);
  void publishFrameCloud(const ros::Publisher& pub,const std::vector<FrameLabelInfo>& frames);

}



#endif
