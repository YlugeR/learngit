#ifndef PARSH_H
#define PARSH_H

#include <iostream>
#include <iosfwd>
#include <fstream>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

namespace Robosense {


//解析之后单个object info
  struct SegLabelInfo {
    pcl::PointCloud<pcl::PointXYZINormal> seg;//object包含的points
    std::string label;//标签
  };

  struct FrameLabelInfo {
    std::string frame_pcd_file;
    std::vector<SegLabelInfo> label_info;
  };

  struct RoboLabel {
    bool dir_;
    string global_id_;
    float x_;//box中心坐标x
    float y_;//box中心坐标y
    float z_;//box中心坐标z
    float psi_;
    float theta_;
    float phi_;//box的length方向与x的夹角
    float size_[3];//box的length,width,height
    string type_;//标签，unknown,pedestrain,nonMot,smallMot,bigMot
    float weight_;
    bool available_;
  };

  //单帧的打标信息，单帧包含多个object
  struct RoboLabelData {
    string pcap_name_;
    string pcd_name_;
    int frame_id_;
    int number_box_;
    vector<RoboLabel> label_v_;
    long total_frame_;
  };

  class SerilizeLabelFile {
  public:
    SerilizeLabelFile();

    ~SerilizeLabelFile() {}

    /**
     * @brief serilize the label data
     * @input file_path: input the label file folder,eg. "/home/bigcong/data/sti_train_data/170825-1708-LM120/"
     * @return true if the file folder is correct
     */
    bool serilizeData(const string file_path);

    bool serilizeData_new(const string file_path);

    bool serilizeData_kitii(const string file_path);

    std::string pcd_file_;//the label pcd folder,eg. "/home/bigcong/data/sti_train_data/170825-1708-LM120/pcd"
    std::vector<RoboLabelData> data_;//the serilized label information

  private:

    bool parse_rsfile(string filename, vector<RoboLabelData> &data);

    bool parse_rsfile_new(string filename, vector<RoboLabelData> &data);

    float stringToFloat(const string &str);

    int stringToInt(const string &str);

    int stringToLong(const string &str);

    bool stringToBool(const string &str);

    string Int2Str(int &num);

    string Float2str(float &num);

    string Long2Str(long &num);

    string Bool2String(bool flag);

    void moveOn(string &content, string Symbol);

    string EraseMaohao(string str);

    string EraseDquotes(string str);

    string getBtSymbol(string cont, string sym1, string sym2);

    string getSymAndMove(string &cont, string sym1, string sym2);

    vector<string> split(const string &s, const string &seperator);

    int hex_char_value(char c);

    int hex_to_decimal(const char *szHex, int len);

    void loadLabelDataFromFile(std::string filepath, RoboLabelData &label_vec);
  };

}
#endif








