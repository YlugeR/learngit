#ifndef SEG_EXTRACT_BASED_BOX_H
#define SEG_EXTRACT_BASED_BOX_H

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include "parse.h"
#include "robo_calcfeature.h"
#include "common/geometry/geo_base.h"
#include "common/geometry/transformer.h"
#include "labelDataManager.h"
#include "robosense/robosense.h"

namespace Robosense{

    class ExtractSeg
    {
    public:
        ExtractSeg();
        ~ExtractSeg();

        /**
         * @brief extract label segments of a rosbag
         * @input sf: data serilized object
         * @output frame_vec: the output label segments of rosbags
         */
        void extractSegBag(const SerilizeLabelFile& sf,std::vector<FrameLabelInfo>& frame_vec);

        /**
         * @brief extract label segments of a frame
         * @input sf: data serilized object
         * @input frame_id: the input frame id
         * @output frame: the output label segments of frame
         */
        void extractSegFrame(const SerilizeLabelFile& sf,const int& frame_id,FrameLabelInfo& frame);

        /**
         * @brief init configuration file path
         * @input usrConfigPath: xml file path
         * @input lidarConfigPath: txt file path
         */
        void initConfigPath(const std::string &usrConfigPath, const std::string &lidarConfigPath);

        void convertPoint(PointCloudConstPtr cloud_in, NormalPointCloudPtr cloud_out);

        void saveFeatureToCSV(const std::string feature_file,const std::vector<FrameLabelInfo>& frame_vec);
        void saveFeatureToCSV(const std::string feature_file,const std::vector<SegLabelInfo>& segments);
        void saveFeatureToCSV_Frame(const std::string & seg_file,const std::string feature_file);



    private:
        void extractSegBasedBox(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud_ptr,
                                         const RoboLabel& label_info,pcl::PointCloud<pcl::PointXYZINormal>::Ptr out_cloud_ptr);
        void removeSegBasedBox(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                         const RoboLabel& label_info, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr);

        RoboUsrConfig usrConfig_;
        RoboLidarConfig lidarConfig_;
        Transformer* transformer_;
        RobosenseALL *robosense_all_;
    };

}

#endif
