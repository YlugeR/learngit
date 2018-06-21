#include "seg_extract_based_box.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

static int empty_num = 0;
namespace Robosense{

  ExtractSeg::ExtractSeg()
  {
    usrConfig_ = RoboUsrConfig();
    lidarConfig_ = RoboLidarConfig();
    robosense_all_ = NULL;
    transformer_ = new Transformer(lidarConfig_, usrConfig_);
  }

  ExtractSeg::~ExtractSeg() 
  {
    delete transformer_;
    transformer_ = NULL;

    delete robosense_all_;
    robosense_all_ = NULL;
  }

  void ExtractSeg::extractSegBag(const SerilizeLabelFile& sf,std::vector<FrameLabelInfo>& frame_vec)
  {
    std::cout<<"frame size:"<<sf.data_.size()<<std::endl;
    frame_vec.clear();

    for(int i = 0; i < sf.data_.size(); i++)
    {
      // if(i==1)
      //   break;
      std::cout << "Segment extract frame " << i << "/" << sf.data_.size() << std::endl;
      FrameLabelInfo tmp_frame;
      extractSegFrame(sf,i,tmp_frame);
      frame_vec.push_back(tmp_frame);
    }
  }

  void ExtractSeg::extractSegFrame(const SerilizeLabelFile& sf,const int& frame_id,FrameLabelInfo& frame)
  {
    // 读取标签和PCD
    if(frame_id > sf.data_.size() - 1 || frame_id < 0)
    {
      std::cout<<"the input frame id out of range!"<<std::endl;
      return;
    }
    RoboLabelData frame_info = sf.data_[frame_id];
    std::string pcd_file = sf.pcd_file_ + "/" + frame_info.pcd_name_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr frame_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file,*frame_cloud_ptr) == -1)
    {
      PCL_ERROR("Couldn't read file \n");
      return;
    }

    // 矫正点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr align_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // transformer_->initCalibration(frame_cloud_ptr,temp_cloud_ptr);
    // transformer_->autoAlign(temp_cloud_ptr,align_cloud_ptr);
    pcl::copyPointCloud(*frame_cloud_ptr, *align_cloud_ptr);

    // // 逐帧显示点云
    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(align_cloud_ptr);//在这个窗口显示点云
    // while (!viewer.wasStopped())
    // {
    // }

    // 点云XYZI 转换成 点云XYZINor 类型
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normal_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
    convertPoint(align_cloud_ptr,cloud_with_normal_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // Robosense::computeNormal2(frame_cloud_ptr,cloud_with_normal_ptr);
    // Robosense::estimateCurveFromNormal(cloud_with_normal_ptr);

    frame.frame_pcd_file = pcd_file;
    SegLabelInfo label_seg;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr manual_cloud_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
    Eigen::Vector3f axis_vector(0, 0, 1);
    Eigen::Matrix4f auto_rot_mat;

    for(int j = 0; j < sf.data_[frame_id].label_v_.size(); ++j)
    {
      RoboLabel label_info = sf.data_[frame_id].label_v_[j];
      if(label_info.available_ == false)
          continue;
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr seg_cloud_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
      extractSegBasedBox(cloud_with_normal_ptr,label_info,seg_cloud_ptr);

      label_seg.label = label_info.type_;
      label_seg.seg = *seg_cloud_ptr;
      frame.label_info.push_back(label_seg);

      if(label_info.type_ == "nonMot")
      {
        COUT(label_info.type_);
        auto_rot_mat = calcRotationMatrix(-1.0f, axis_vector);
        manual_cloud_ptr->clear();
        pcl::transformPointCloud(*seg_cloud_ptr, *manual_cloud_ptr, auto_rot_mat);

        label_seg.label = label_info.type_;
        label_seg.seg = *manual_cloud_ptr;
        frame.label_info.push_back(label_seg);

        auto_rot_mat = calcRotationMatrix(1.0f, axis_vector);
        manual_cloud_ptr->clear();
        pcl::transformPointCloud(*seg_cloud_ptr, *manual_cloud_ptr, auto_rot_mat);

        label_seg.label = label_info.type_;
        label_seg.seg = *manual_cloud_ptr;
        frame.label_info.push_back(label_seg);
      }

      // 将正样本从点云中抠掉
      removeSegBasedBox(align_cloud_ptr,label_info,out_cloud_ptr);
      pcl::copyPointCloud(*out_cloud_ptr, *align_cloud_ptr);
    }

    // // 逐帧显示点云
    // pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer1");
    // viewer1.showCloud(align_cloud_ptr);//在这个窗口显示点云
    // while (!viewer1.wasStopped())
    // {
    // }

    // 生成负样本
    std::vector<NormalRoboPerceptron> percept_vec_filtered;
    robosense_all_->mainPerceptionProcess(align_cloud_ptr);
    percept_vec_filtered = robosense_all_->getObjectPeceptResults();
    convertPoint(align_cloud_ptr,cloud_with_normal_ptr);

    for(int j = 0; j < percept_vec_filtered.size(); ++j)
    {
      if(percept_vec_filtered[j].cluster.box.center.x == 0.0f ||
         percept_vec_filtered[j].cluster.box.center.y == 0.0f ||
         percept_vec_filtered[j].cluster.box.center.z == 0.0f ||
         percept_vec_filtered[j].cluster.box.size.x == 0.0f ||
         percept_vec_filtered[j].cluster.box.size.y == 0.0f ||
         percept_vec_filtered[j].cluster.box.size.z == 0.0f 
      )
          continue;

      RoboLabel label_info;
      label_info.x_ = percept_vec_filtered[j].cluster.box.center.x;
      label_info.y_ = percept_vec_filtered[j].cluster.box.center.y;
      label_info.z_ = percept_vec_filtered[j].cluster.box.center.z;
      label_info.phi_ = percept_vec_filtered[j].cluster.box.angle;
      label_info.theta_ = 0.0f;
      label_info.psi_ = 0.0f;
      label_info.size_[0] = percept_vec_filtered[j].cluster.box.size.x;
      label_info.size_[1] = percept_vec_filtered[j].cluster.box.size.y;
      label_info.size_[2] = percept_vec_filtered[j].cluster.box.size.z;
      label_info.type_ = "unknown";
      label_info.weight_ = '1';
      label_info.available_ = true;
      // COUT("segemet["<<j<<"]");
      // COUT("x:"<<label_info.x_<<",y:"<<label_info.y_<<",z:"<<label_info.z_);
      // COUT("size0:"<<label_info.size_[0]<<",size1:"<<label_info.size_[1]<<",size2:"<<label_info.size_[2]);

      pcl::PointCloud<pcl::PointXYZINormal>::Ptr seg_cloud_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
      extractSegBasedBox(cloud_with_normal_ptr,label_info,seg_cloud_ptr);

      label_seg.label = label_info.type_;
      label_seg.seg = *seg_cloud_ptr;
      frame.label_info.push_back(label_seg);
    }

  }

  void ExtractSeg::initConfigPath(const std::string &usrConfigPath, const std::string &lidarConfigPath)
  {
    // 载入参数xml
    usrConfig_.load(usrConfigPath);
    // 载入Lidar参数
    lidarConfig_.load(lidarConfigPath);

    delete transformer_;
    transformer_ =  new Transformer(lidarConfig_, usrConfig_); 

    delete robosense_all_;
    robosense_all_ = new RobosenseALL(lidarConfigPath, usrConfigPath);
  }

  void ExtractSeg::convertPoint(PointCloudConstPtr cloud_in, NormalPointCloudPtr cloud_out)
  {
    if (cloud_in->size()==0)
    {
      return;
    }

    cloud_out->width = cloud_in->width;
    cloud_out->height = cloud_in->height;
    cloud_out->is_dense = false;
    cloud_out->resize(cloud_in->size());
    cloud_out->header = cloud_in->header;
    cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
    cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;

    pcl::PointXYZINormal normal;
    pcl::PointXYZI pointKey;

    for (int i = 0; i < cloud_in->height; ++i)
    {
      for (int j = 0; j < cloud_in->width; ++j)
      {
        pointKey = cloud_in->at(i*cloud_in->width+j);
        normal.x = pointKey.x;
        normal.y = pointKey.y;
        normal.z = pointKey.z;

        normal.intensity = pointKey.intensity;
        normal.normal_x = NAN;
        normal.normal_y = NAN;
        normal.normal_z = NAN;
        normal.curvature = NAN;
        cloud_out->at(i*cloud_in->width+j) = normal;
      }
    }
  }

  void ExtractSeg::removeSegBasedBox(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                                const RoboLabel& label_info, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
  {
    float crop_min_x, crop_min_y, crop_min_z;
    float crop_max_x, crop_max_y, crop_max_z;
    crop_min_x = -label_info.size_[0] / 2 - 0.05;
    crop_min_y = -label_info.size_[1] / 2 - 0.05;
    crop_min_z = -label_info.size_[2] / 2;

    crop_max_x = label_info.size_[0] / 2 + 0.05;
    crop_max_y = label_info.size_[1] / 2 + 0.05;
    crop_max_z = label_info.size_[2] / 2 ;

    pcl::CropBox<pcl::PointXYZI> clipper(true);
    clipper.setInputCloud(in_cloud_ptr);
    clipper.setMin(Eigen::Vector4f(crop_min_x, crop_min_y, crop_min_z, 0));
    clipper.setMax(Eigen::Vector4f(crop_max_x, crop_max_y, crop_max_z, 0));//最大点的坐标

    clipper.setTranslation(Eigen::Vector3f(label_info.x_,label_info.y_,label_info.z_));
    clipper.setRotation(Eigen::Vector3f(0,0,label_info.phi_));

    std::vector<int> indices;
    clipper.filter(indices);

    pcl::PointIndicesPtr indicesPtr(new pcl::PointIndices);
    for (int i = 0; i < indices.size(); ++i) {
        indicesPtr->indices.push_back(indices[i]);
    }

    pcl::ExtractIndices<pcl::PointXYZI> extracts;
    extracts.setIndices(indicesPtr);
    extracts.setInputCloud(in_cloud_ptr);
    extracts.setNegative(true);
    extracts.filter(*out_cloud_ptr);
  }

  void ExtractSeg::extractSegBasedBox(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud_ptr,
                                                const RoboLabel& label_info,pcl::PointCloud<pcl::PointXYZINormal>::Ptr out_cloud_ptr)
  {
    float crop_min_x, crop_min_y, crop_min_z;
    float crop_max_x, crop_max_y, crop_max_z;
    
    crop_min_x = -label_info.size_[0] / 2 - 0.05;
    crop_min_y = -label_info.size_[1] / 2 - 0.05;
    crop_min_z = -label_info.size_[2] / 2;

    crop_max_x = label_info.size_[0] / 2 + 0.05;
    crop_max_y = label_info.size_[1] / 2 + 0.05;
    crop_max_z = label_info.size_[2] / 2 ;

    pcl::CropBox<pcl::PointXYZINormal> clipper(true);
    clipper.setInputCloud(in_cloud_ptr);
    clipper.setMin(Eigen::Vector4f(crop_min_x, crop_min_y, crop_min_z, 0));
    clipper.setMax(Eigen::Vector4f(crop_max_x, crop_max_y, crop_max_z, 0));//最大点的坐标

    clipper.setTranslation(Eigen::Vector3f(label_info.x_,label_info.y_,label_info.z_));
    clipper.setRotation(Eigen::Vector3f(0,0,label_info.phi_));

    std::vector<int> indices;
    clipper.filter(indices);

    pcl::PointIndicesPtr indicesPtr(new pcl::PointIndices);
    for (int i = 0; i < indices.size(); ++i) 
    {
      indicesPtr->indices.push_back(indices[i]);
    }

    pcl::ExtractIndices<pcl::PointXYZINormal> extracts;
    extracts.setIndices(indicesPtr);
    extracts.setInputCloud(in_cloud_ptr);
    extracts.setNegative(false);
    extracts.filter(*out_cloud_ptr);
  }

  void ExtractSeg::saveFeatureToCSV(const std::string feature_file,const std::vector<SegLabelInfo>& segments)
  {
    std::ofstream outfile;
    outfile.open(feature_file.c_str(),std::ios::app);
    outfile <<"norm0,norm1,norm2,norm3,norm4,norm5,norm6,norm7,norm8,"
            << "norm_dev,"
            << "inertia0,inertia1,inertia2,inertia3,inertia4,inertia5,"
            << "n_inertia0,n_inertia1,n_inertia2,n_inertia3,n_inertia4,n_inertia5,"
            << "cov0,cov1,cov2,cov3,cov4,cov5,"
            << "n_cov0,n_cov1,n_cov2,n_cov3,n_cov4,n_cov5,"
            << "eig0,eig1,eig2,eig3,"
            << "n_eig0,n_eig1,n_eig2,n_eig3,"
            << "mid_cov0,mid_cov1,mid_cov2,mid_cov3,mid_cov4,mid_cov5,"
//            << "n_mid_cov0,n_mid_cov1,n_mid_cov2,n_mid_cov3,n_mid_cov4,n_mid_cov5,"
            << "width,length,height,dis,ang,ori,"
            << "z_min,z_max,"
            << "max_intensity,mean_intensity,dev_intensity,"
            << "hist_x1,hist_x2,hist_x3,hist_x4,hist_x5,hist_x6,hist_x7,hist_x8,hist_x9,hist_x10,"
            << "hist_y1,hist_y2,hist_y3,hist_y4,hist_y5,hist_y6,hist_y7,hist_y8,hist_y9,hist_y10,"
            << "hist_z1,hist_z2,hist_z3,hist_z4,hist_z5,hist_z6,hist_z7,hist_z8,hist_z9,hist_z10,"
            // << "hist_z1,hist_z2,hist_z3,hist_z4,hist_z5,hist_z6,hist_z7,hist_z8,hist_z9,hist_z10,"
            << "label"<<std::endl;

    Robosense::RoboFeature des_3d;
    for(int i = 0; i < segments.size(); i++)
    {
      Robosense::SegLabelInfo seg = segments[i];
      if(seg.seg.points.size() <=5)
        continue;
      des_3d.computeFeature3D_train(seg.seg.makeShared());
      std::vector<float> feature;
      des_3d.getFeature(feature);

      for(int k = 0; k < feature.size(); k++)
        outfile<<feature[k]<<",";
      outfile<<seg.label<<std::endl;
    }
    outfile.close();
  }

  void ExtractSeg::saveFeatureToCSV(const std::string feature_file,const std::vector<FrameLabelInfo>& frame_vec)
  {
    std::ofstream outfile;
    outfile.open(feature_file.c_str(),std::ios::app);
    // outfile <<"norm0,norm1,norm2,norm3,norm4,norm5,norm6,norm7,norm8,"
            // << "norm_dev,"
    outfile << "inertia0,inertia1,inertia2,inertia3,inertia4,inertia5,"
            // << "n_inertia0,n_inertia1,n_inertia2,n_inertia3,n_inertia4,n_inertia5,"
            << "cov0,cov1,cov2,cov3,cov4,cov5,"
            // << "n_cov0,n_cov1,n_cov2,n_cov3,n_cov4,n_cov5,"
            << "eig0,eig1,eig2,eig3,"
            // << "n_eig0,n_eig1,n_eig2,n_eig3,"
            << "mid_cov0,mid_cov1,mid_cov2,mid_cov3,mid_cov4,mid_cov5,"
            // << "n_mid_cov0,n_mid_cov1,n_mid_cov2,n_mid_cov3,n_mid_cov4,n_mid_cov5,"
            << "width,length,height,dis,ang,ori,"
            << "z_min,z_max,"
            << "max_intensity,mean_intensity,dev_intensity,"
            << "hist_x1,hist_x2,hist_x3,hist_x4,hist_x5,hist_x6,hist_x7,hist_x8,hist_x9,hist_x10,"
            << "hist_y1,hist_y2,hist_y3,hist_y4,hist_y5,hist_y6,hist_y7,hist_y8,hist_y9,hist_y10,"
            << "hist_z1,hist_z2,hist_z3,hist_z4,hist_z5,hist_z6,hist_z7,hist_z8,hist_z9,hist_z10,"
            // << "hist_z1,hist_z2,hist_z3,hist_z4,hist_z5,hist_z6,hist_z7,hist_z8,hist_z9,hist_z10,"
            << "label"<<std::endl;

    Robosense::RoboFeature des_3d;
    for(int i = 0; i < frame_vec.size(); i++)
    {
      for(int j = 0; j < frame_vec[i].label_info.size(); j++)
      {

        Robosense::SegLabelInfo seg = frame_vec[i].label_info[j];
        if(seg.seg.points.size() <=5)
          continue;
        des_3d.computeFeature3D_train(seg.seg.makeShared());
        std::vector<float> feature;
        des_3d.getFeature(feature);

        for(int k = 0; k < feature.size(); k++)
        {
            outfile<<feature[k]<<",";
        }
        outfile<<seg.label<<std::endl;
      }
    }
    outfile.close();
  }

  void ExtractSeg::saveFeatureToCSV_Frame(const std::string & filename,const std::string feature_file)
  {
    std::ofstream outfile;
    outfile.open(feature_file.c_str(),std::ios::app);
    outfile <<"norm0,norm1,norm2,norm3,norm4,norm5,norm6,norm7,norm8,"
            << "norm_dev,"
            << "inertia0,inertia1,inertia2,inertia3,inertia4,inertia5,"
            << "n_inertia0,n_inertia1,n_inertia2,n_inertia3,n_inertia4,n_inertia5,"
            << "cov0,cov1,cov2,cov3,cov4,cov5,"
            << "n_cov0,n_cov1,n_cov2,n_cov3,n_cov4,n_cov5,"
            << "eig0,eig1,eig2,eig3,"
            << "n_eig0,n_eig1,n_eig2,n_eig3,"
            << "mid_cov0,mid_cov1,mid_cov2,mid_cov3,mid_cov4,mid_cov5,"
            << "n_mid_cov0,n_mid_cov1,n_mid_cov2,n_mid_cov3,n_mid_cov4,n_mid_cov5,"
            << "width,length,height,dis,ang,ori,"
            << "z_min,z_max,"
            << "max_intensity,mean_intensity,dev_intensity,"
            << "hist_x1,hist_x2,hist_x3,hist_x4,hist_x5,hist_x6,hist_x7,hist_x8,hist_x9,hist_x10,"
            << "hist_y1,hist_y2,hist_y3,hist_y4,hist_y5,hist_y6,hist_y7,hist_y8,hist_y9,hist_y10,"
            << "hist_z1,hist_z2,hist_z3,hist_z4,hist_z5,hist_z6,hist_z7,hist_z8,hist_z9,hist_z10,"
            // << "hist_z1,hist_z2,hist_z3,hist_z4,hist_z5,hist_z6,hist_z7,hist_z8,hist_z9,hist_z10,"
            << "label"<<std::endl;

    LabelDataManager label_manager;
    std::string seg_path;
    std::string label;
    for(int k = 0; k < 5; k++)
    {
      if(k == 0) 
      {
        seg_path = filename + "/unknown/";
        label = "unknown";
      }
      if(k == 1) 
      {
        seg_path = filename + "/smallMot/";
        label = "smallMot";
      }
      if(k == 2) 
      {
        seg_path = filename + "/bigMot/";
        label = "bigMot";
      }
      if(k == 3) 
      {
        seg_path = filename + "/pedestrian/";
        label = "pedestrian";
      }
      if(k == 4) 
      {
        seg_path = filename + "/nonMot/";
        label = "nonMot";
      }

      std::vector<SegLabelInfo> seg_vec;

      label_manager.deserialize_pcd_class(seg_path,label,seg_vec);

      std::cout<<"load pcd finish!"<<std::endl;

      Robosense::RoboFeature des_3d;
      for(int i = 0; i < seg_vec.size(); i++)
      {
        Robosense::SegLabelInfo seg = seg_vec[i];
        if(seg.seg.points.size() <=5)
          continue;
        des_3d.computeFeature3D_train(seg.seg.makeShared());
        std::vector<float> feature;
        des_3d.getFeature(feature);

        for(int k = 0; k < feature.size(); k++)
          outfile<<feature[k]<<",";
        outfile<<seg.label<<std::endl;
      }

      std::cout<<"save feature finish!"<<std::endl;

    }
    outfile.close();
  }

}
