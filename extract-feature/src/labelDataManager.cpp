#include "labelDataManager.h"
#include "ros/rate.h"
#include "dirManager.h"
namespace Robosense {



  void LabelDataManager::serialize(const std::string & filename,const std::vector<FrameLabelInfo>& frames)
  {
    std::ofstream ostrm;
    ostrm.open(filename.c_str(), std::ios::app);
    if(!ostrm.is_open())
    {
      std::cout<<filename<<" open fail!"<<std::endl;
      return;
    }
    ostrm<<"frame_num "<<frames.size()<<std::endl;
    for(int i = 0; i < frames.size(); i++)
    {
      ostrm<<"frame_pcd_file "<<frames[i].frame_pcd_file<<std::endl;
      ostrm<<"seg_num "<<frames[i].label_info.size()<<std::endl;
      for(int j = 0; j < frames[i].label_info.size(); j++)
      {
        SegLabelInfo tmp = frames[i].label_info[j];
        ostrm<<"label "<<tmp.label<<std::endl;
        ostrm<<"seg_points "<<tmp.seg.points.size()<<std::endl;
        for(int pp = 0; pp < tmp.seg.points.size(); pp++)
        {
          pcl::PointXYZINormal pt = tmp.seg.points[pp];
          ostrm.write((char*)&pt.x, sizeof(float));
          ostrm.write((char*)&pt.y, sizeof(float));
          ostrm.write((char*)&pt.z, sizeof(float));
          ostrm.write((char*)&pt.intensity, sizeof(float));
          ostrm.write((char*)&pt.normal_x, sizeof(float));
          ostrm.write((char*)&pt.normal_y, sizeof(float));
          ostrm.write((char*)&pt.normal_z, sizeof(float));
          ostrm.write((char*)&pt.curvature, sizeof(float));
          ostrm<<std::endl;
        }
      }
    }
    ostrm.close();
  }

  void LabelDataManager::serialize_pcd(const std::string & filename,const std::vector<FrameLabelInfo>& frames)
  {
    int unknow = 0;
    int smallMot = 0;
    int bigMot = 0;
    int ped = 0;
    int nonMot = 0;


    for(int i = 0; i < frames.size(); i++)
    {
      for(int j = 0; j < frames[i].label_info.size(); j++)
      {
        std::stringstream ss_seg;
        std::string file_seg;
        if(frames[i].label_info[j].label == "unknown")
        {
          ss_seg << unknow;
          file_seg = filename + "unknown/" + ss_seg.str() + ".pcd";
          unknow++;
        }

        else if(frames[i].label_info[j].label == "smallMot")
        {
          ss_seg << smallMot;
          file_seg = filename + "smallMot/" + ss_seg.str() + ".pcd";
          smallMot++;
        }

        else if(frames[i].label_info[j].label == "bigMot")
        {
          ss_seg << bigMot;
          file_seg = filename + "bigMot/" + ss_seg.str() + ".pcd";

          bigMot++;
        }

        else if(frames[i].label_info[j].label == "pedestrian")
        {
          ss_seg << ped;
          file_seg = filename + "pedestrian/" + ss_seg.str() + ".pcd";

          ped++;
        }

        else if(frames[i].label_info[j].label == "nonMot")
        {
          ss_seg << nonMot;
          file_seg = filename + "nonMot/" + ss_seg.str() + ".pcd";

          nonMot++;
        }
        else
          continue;


//        std::cout<<filename<<std::endl;
//          std::cout<<ss_seg.str()<<std::endl;
//        std::cout<<file_seg<<std::endl;
        pcl::PointCloud<pcl::PointXYZINormal> seg = frames[i].label_info[j].seg;
        if(seg.points.size() > 5)
        {
          pcl::io::savePCDFileASCII(file_seg,seg);
        }
      }

    }

  }

  bool LabelDataManager::deserialize_pcd(const std::string & filename, std::vector<SegLabelInfo>& segments)
  {
    std::string seg_path;
    dirManager file_manager;
    std::vector<std::string> seg_file_vec;
    std::string label;

    for(int k = 0; k < 5; k++)
    {
      if(k == 0) {
        seg_path = filename + "/unknown/";
        label = "unknown";
      }
      if(k == 1) {
        seg_path = filename + "/smallMot/";
        label = "smallMot";
      }
      if(k == 2) {
        seg_path = filename + "/bigMot/";
        label = "bigMot";
      }
      if(k == 3) {
        seg_path = filename + "/pedestrian/";
        label = "pedestrian";
      }
      if(k == 4) {
        seg_path = filename + "/nonMot/";
        label = "nonMot";
      }

      seg_file_vec.clear();
      file_manager.getDirAndName(seg_path,seg_file_vec);
      std::cout<<seg_path<<std::endl;
      std::cout<<"segmensts num: "<<seg_file_vec.size()<<std::endl;
      for(int i = 0; i < seg_file_vec.size(); i++)
      {
//        std::cout<<seg_file_vec[i]<<std::endl;
        SegLabelInfo seg;
        seg.label = label;
        if(pcl::io::loadPCDFile(seg_file_vec[i],seg.seg) == -1)
          continue;
        segments.push_back(seg);
      }
    }
  }

  bool LabelDataManager::deserialize_pcd_class(const std::string & filename, const std::string& label,std::vector<SegLabelInfo>& segments)
  {
    dirManager file_manager;
    std::vector<std::string> seg_file_vec;
    file_manager.getDirAndName(filename,seg_file_vec);
    std::cout<<filename<<std::endl;
    std::cout<<"segmensts num: "<<seg_file_vec.size()<<std::endl;

    segments.clear();
    for(int i = 0; i < seg_file_vec.size(); i++)
    {
//        std::cout<<seg_file_vec[i]<<std::endl;
      SegLabelInfo seg;
      seg.label = label;
      if(pcl::io::loadPCDFile(seg_file_vec[i],seg.seg) == -1)
        continue;
      segments.push_back(seg);
    }
  }

  bool LabelDataManager::deserialize(const std::string &filename,std::vector<FrameLabelInfo>& frames)
  {
    std::ifstream istrm(filename.c_str(),std::ios::in);
    if(istrm.eof())
      return false;
    frames.clear();

    std::string line;
    istrm >>line;
    if(line.compare("frame_num") != 0)
    {
      std::cout<<"Expected 'frame_num',got fail "<<line<<std::endl;
      return false;
    }
    int frame_num = 0;
    istrm >> frame_num;
    std::getline(istrm,line);
    for(int i = 0; i < frame_num; i++)
    {
      FrameLabelInfo tmp_frame;

      istrm >> line;
      if(line.compare("frame_pcd_file") != 0)
      {
        std::cout<<"Expected 'frame_pcd_file',got fail "<<line<<std::endl;
        return false;
      }
      istrm>>tmp_frame.frame_pcd_file;
      std::getline(istrm,line);
      istrm >> line;
      if(line.compare("seg_num") != 0)
      {
        std::cout<<"Expected 'seg_num',got fail "<<line<<std::endl;
        return false;
      }
      int seg_num = 0;
      istrm >> seg_num;
      std::getline(istrm,line);
      for(int j = 0; j < seg_num; j++)
      {
        SegLabelInfo tmp_seg;
        istrm >> line;
        if(line.compare("label") != 0)
        {
          std::cout<<"Expected 'label',got fail "<<line<<std::endl;
          return false;
        }
        istrm >> tmp_seg.label;
        std::getline(istrm,line);
        istrm >> line;
        if(line.compare("seg_points") != 0)
        {
          std::cout<<"Expected 'seg_points',got fail "<<line<<std::endl;
          return false;
        }
        int seg_points = 0;
        istrm >> seg_points;
        tmp_seg.seg.height = 1;

        std::getline(istrm, line);

        for(int k = 0; k < seg_points;++k )
        {
          pcl::PointXYZINormal pt;
          istrm.read((char*)&pt.x,sizeof(float));
          istrm.read((char*)&pt.y,sizeof(float));
          istrm.read((char*)&pt.z,sizeof(float));
          istrm.read((char*)&pt.intensity,sizeof(float));
          istrm.read((char*)&pt.normal_x,sizeof(float));
          istrm.read((char*)&pt.normal_y,sizeof(float));
          istrm.read((char*)&pt.normal_z,sizeof(float));
          istrm.read((char*)&pt.curvature,sizeof(float));
          std::getline(istrm, line);
          tmp_seg.seg.points.push_back(pt);
        }
        tmp_seg.seg.width = tmp_seg.seg.points.size();
        tmp_frame.label_info.push_back(tmp_seg);
      }
      frames.push_back(tmp_frame);
    }
    return true;
  }

  void publishSegCloud(const ros::Publisher& pub,const std::vector<FrameLabelInfo>& frames)
  {
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
      for(int i = 0; i < frames.size(); i++)
      {
        for(int j = 0; j < frames[i].label_info.size(); j++)
        {
          pcl::PointCloud<pcl::PointXYZINormal> seg = frames[i].label_info[j].seg;
          seg.height = 1;
          seg.width = seg.points.size();
          seg.header.frame_id = "rslidar";
          pub.publish(seg);
          loop_rate.sleep();
        }
      }
    }
  }

  void publishFrameCloud(const ros::Publisher& pub,const std::vector<FrameLabelInfo>& frames)
  {
    ros::Rate loop_rate(1);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    for(int i = 0; i < frames.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr frame_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      if(pcl::io::loadPCDFile(frames[i].frame_pcd_file,*frame_cloud_ptr) == -1)
      {
        PCL_ERROR("Couldn't read file \n");
        return;
      }

      for(int pt = 0; pt < frame_cloud_ptr->size(); pt++)
      {
        pcl::PointXYZINormal tmp_pt;
        tmp_pt.x = frame_cloud_ptr->points[pt].x;
        tmp_pt.y = frame_cloud_ptr->points[pt].y;
        tmp_pt.z = frame_cloud_ptr->points[pt].z;
        pcloud->push_back(tmp_pt);
      }
      for(int k = 0; k < frame_cloud_ptr->size(); k++)
        pcloud->points[k].intensity = 100;
      for(int j = 0; j < frames[i].label_info.size(); j++)
      {
        pcl::PointCloud<pcl::PointXYZINormal> seg = frames[i].label_info[j].seg;
        int intensity = 0;
        if(frames[i].label_info[j].label == "unknown")
          intensity = 160;
        else
          intensity = 255;
        for(int k = 0; k < seg.points.size(); k++)
          seg.points[k].intensity = intensity;
        *pcloud += seg;
      }
      pcloud->height = 1;
      pcloud->width = pcloud->points.size();
      pcloud->header.frame_id = "rslidar";
      pub.publish(*pcloud);
      loop_rate.sleep();
    }
  }
}

