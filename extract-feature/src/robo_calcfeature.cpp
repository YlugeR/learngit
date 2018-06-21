#include "robo_calcfeature.h"
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/vfh.h>
#include "common/geometry/geo_base.h"
#include "pcl/features/normal_3d.h"
#include "module_manager/module_manager.h"
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#define USE_NORMAL_STATISTIC 0
#define USE_AUTHORITY 1
#define USE_SEMANTIC_BOX 0

namespace Robosense
{

RoboFeature::RoboFeature(const float &estimate_lidar_height)
{
#if USE_AUTHORITY
  if(!ModuleManager::initModule())
    return;
#endif
  estimate_lidar_height_ = estimate_lidar_height;
}

void RoboFeature::getFeature(std::vector<float> &feature)
{
  feature.clear();
  std::vector<float>().swap(feature);

  feature.insert(feature.end(), inertia_tensor_.begin(), inertia_tensor_.end());//6
  feature.insert(feature.end(), cov_mat_.begin(), cov_mat_.end());//6
  feature.insert(feature.end(), cov_eigenvalue_.begin(), cov_eigenvalue_.end());//4
  feature.insert(feature.end(), mid_cov_.begin(), mid_cov_.end());//6
  feature.insert(feature.end(), local_pose_.begin(), local_pose_.end());//6
  feature.insert(feature.end(), range_z_.begin(), range_z_.end());//2
  feature.insert(feature.end(), intensity_statistic_.begin(), intensity_statistic_.end());//3
  feature.insert(feature.end(), hist_.begin(), hist_.end());//30

#if USE_NORMAL_STATISTIC
  feature.insert(feature.end(),normal_his_.begin(),normal_his_.end());//9
  feature.insert(feature.end(),normal_dev_.begin(),normal_dev_.end());//1
  feature.insert(feature.end(),n_inertia_tensor_.begin(),n_inertia_tensor_.end());//6
  feature.insert(feature.end(),n_cov_mat_.begin(),n_cov_mat_.end());//6
  feature.insert(feature.end(),n_cov_eigenvalue_.begin(),n_cov_eigenvalue_.end());//4
  feature.insert(feature.end(),n_mid_cov_.begin(),n_mid_cov_.end());//6
#endif

}

void RoboFeature::computeFeature3D_train(const NormalPointCloudPtr &in_cloud_ptr)
{
  NormalPointCloudPtr pcloud(new NormalPointCloud);
  *pcloud = *in_cloud_ptr;

  BoundingBoxCalculator<NormalPoint> *box_cal = new BoundingBoxCalculator<NormalPoint>();
  BoundingBox box = box_cal->calcBoundingBox(pcloud);

  //try to classifiy based on semantic box
  BoundingBox semantic_box;
  ObjectLimit obj_limit;
  obj_limit.obj_max_height = 4.5f;
  obj_limit.obj_size_height = 4.f;
  boxInferByGeo(box, obj_limit, estimate_lidar_height_, semantic_box);
  box = resizeBox(box, Point3f(box.size.x, semantic_box.size.y, box.size.z));

#if USE_NORMAL_STATISTIC
  normalHistorgram(pcloud);
  normalDevDescriptor(pcloud);
#endif
  calcuInertiaTensor(pcloud);//与安装位置相关
  calcuCovMat(pcloud);
  calcuCovEigenvalue(cov_mat_, cov_eigenvalue_);
#if USE_NORMAL_STATISTIC
  calcuCovEigenvalue(n_cov_mat_,n_cov_eigenvalue_);
#endif
  calcuMidCov(pcloud);
#if USE_SEMANTIC_BOX
  calcuLocalPoseNew(semantic_box);//try to classifiy based on semantic box
#else
  calcuLocalPoseNew(box);
#endif
  calcuIntensityStatistic(pcloud);
  calcuHistogram(pcloud, box);
}

void RoboFeature::computeFeature3D_predict(const NormalRoboCluster &in_cluster)
{
  NormalPointCloudPtr pcloud(new NormalPointCloud);
  *pcloud = *in_cluster.pointCloud;

#if USE_SEMANTIC_BOX
  BoundingBox box = in_cluster.semantic_box; //try to classifiy based on semantic box
#else
  BoundingBox box = in_cluster.box;
#endif

#if USE_NORMAL_STATISTIC
  normalHistorgram(pcloud);
  normalDevDescriptor(pcloud);
#endif
  calcuInertiaTensor(pcloud);
  calcuCovMat(pcloud);
  calcuCovEigenvalue(cov_mat_, cov_eigenvalue_);
#if USE_NORMAL_STATISTIC
  calcuCovEigenvalue(n_cov_mat_,n_cov_eigenvalue_);
#endif
  calcuMidCov(pcloud);
  calcuLocalPoseNew(box);
  calcuIntensityStatistic(pcloud);
  calcuHistogram(pcloud, box);
}

void RoboFeature::calcuHistogram(NormalPointCloudPtr cloud, const BoundingBox &box)//30
{

  // // 逐帧显示点云
  // PointCloudPtr temp(new PointCloud);
  // pcl::copyPointCloud(*cloud, *temp);
  // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  // viewer.showCloud(temp);//在这个窗口显示点云
  // while (!viewer.wasStopped())
  // {
  // }

  NormalPointCloudPtr align_cloud_ptr(new NormalPointCloud);
  pcl::copyPointCloud(*cloud, *align_cloud_ptr);
  align_cloud_ptr->clear();
  // Eigen::Vector3f ori_vec( box.heading.x, box.heading.y, 0);
  // Eigen::Vector3f target_vec(1, 0, 0);
  // Eigen::Matrix4f auto_rot_mat = calcRotationMatrix(ori_vec, target_vec);
  Eigen::Vector3f axis_vector(0, 0, 1);
  Eigen::Matrix4f auto_rot_mat = calcRotationMatrix(-box.angle, axis_vector);
  pcl::transformPointCloud(*cloud, *align_cloud_ptr, auto_rot_mat);

  // // 逐帧显示点云
  // pcl::copyPointCloud(*align_cloud_ptr, *temp);
  // pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
  // viewer1.showCloud(temp);//在这个窗口显示点云
  // while (!viewer1.wasStopped())
  // {
  // }

  if(box.heading.x==1 && box.heading.y==0)
  {
    pcl::copyPointCloud(*cloud, *align_cloud_ptr);
  }

  int bin_size = 10;
  NormalPoint max_pt, min_pt;
  int pt_num = align_cloud_ptr->size();

  min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
  max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;
  for (int i = 0; i < pt_num; ++i) {
    NormalPoint pt = align_cloud_ptr->points[i];
    min_pt.x = std::min(min_pt.x, pt.x);
    max_pt.x = std::max(max_pt.x, pt.x);
    min_pt.y = std::min(min_pt.y, pt.y);
    max_pt.y = std::max(max_pt.y, pt.y);
    min_pt.z = std::min(min_pt.z, pt.z);
    max_pt.z = std::max(max_pt.z, pt.z);
  }

  int xstep = bin_size;
  int ystep = bin_size;
  int zstep = bin_size;
  int stat_len = xstep + ystep + zstep;
  // int stat_len = bin_size;
  std::vector<int> stat_feat(stat_len, 0);
  float xsize = (max_pt.x - min_pt.x) / xstep + 0.000001;
  float ysize = (max_pt.y - min_pt.y) / ystep + 0.000001;
  float zsize = (max_pt.z - min_pt.z) / zstep + 0.000001;


  for (int i = 0; i < pt_num; ++i) {
    NormalPoint pt = align_cloud_ptr->points[i];
    stat_feat[floor((pt.x - min_pt.x) / xsize)]++;
    stat_feat[xstep + floor((pt.y - min_pt.y) / ysize)]++;
    stat_feat[xstep + ystep + floor((pt.z - min_pt.z) / zsize)]++;
    // stat_feat[floor((pt.z - min_pt.z) / zsize)]++;
  }
  // update feature
  hist_.resize( stat_len, 0);
  for (size_t i = 0; i < stat_feat.size(); ++i) {
    hist_[i] = static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num);
  }

}

void RoboFeature::calcuLocalPoseNew(const BoundingBox &box)//5
{
  local_pose_.resize(6, 0);
  range_z_.resize(2, 0);
  local_pose_[0] = std::min(box.size.x, box.size.y);
  local_pose_[1] = std::max(box.size.x, box.size.y);
  local_pose_[2] = box.size.z;

  Point3f center = box.center;
  local_pose_[3] = center.getLength();
  local_pose_[4] = atan2f(center.y, center.x);
  local_pose_[5] = fabs(box.angle);//TODO: maybe the range is not correct

  range_z_[0] = center.z - box.size.z * 0.5f + estimate_lidar_height_;
  range_z_[1] = center.z + box.size.z * 0.5f + estimate_lidar_height_;
}

void RoboFeature::normalHistorgram(NormalPointCloudConstPtr in_cloud_ptr)
{
  /**
   * 说明：
   * x-y平面上yaw角统计0-180度范围，每30度一个bin，共6个，共线反向认为是相同的，所以负角度+180
   * z方向pitch角共90度，3个bin，共线反向认为是相同的
   */
  normal_his_.resize(9, 0);
  std::vector<int> hist;
  hist.resize(9, 0);

  float ang_step = M_PI / 6.f + 1e-6;

  int valid_cnt = 0;

  for(int i = 0; i < in_cloud_ptr->size(); i++)
  {
    const pcl::PointXYZINormal &tmp = in_cloud_ptr->points[i];
    if(isInvalidNormalPoint(tmp)) continue;

    float ang_x = atan2f(tmp.normal_y, tmp.normal_x);
    ang_x = ang_x > 0 ? ang_x : ang_x + M_PI;

    float ang_z = fabsf(acosf(fabsf(tmp.normal_z)));

    int index_x = ang_x / ang_step;
    int index_z = ang_z / ang_step;

    hist[6 + index_z]++;
    hist[index_x]++;

    valid_cnt++;
  }

  for(int i = 0; i < hist.size(); i++)
    normal_his_[i] = hist[i] / static_cast<float>(valid_cnt + 1e-6);
}

//TODO: 此处统计normal是否需要去掉无效normal的影响？会摊平均值
void RoboFeature::normalDevDescriptor(NormalPointCloudConstPtr in_cloud_ptr)
{
  normal_dev_.resize(1, 0);
  float sum = 0;
  int valid_cnt = 0;
  for(int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    const pcl::PointXYZINormal &tmp = in_cloud_ptr->points[i];
    if(isInvalidNormalPoint(tmp)) continue;

    sum += in_cloud_ptr->points[i].curvature;
    valid_cnt++;
  }

  normal_dev_[0] = sum / static_cast<float>(valid_cnt + 1e-6);
}

void RoboFeature::calcuInertiaTensor(NormalPointCloudConstPtr cloud) //与安装位置相关
{
  inertia_tensor_.resize(6, 0);
  n_inertia_tensor_.resize(6, 0);

  int valid_cnt = 0;
  for(int pp = 0; pp < cloud->points.size(); ++pp)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[pp];
    if(isInvalidPoint(tmp)) continue;

    cv::Point3f pts(cloud->points[pp].x, cloud->points[pp].y, cloud->points[pp].z);
    inertia_tensor_[0] += (pts.x) * (pts.x) + (pts.y) * (pts.y);
    inertia_tensor_[1] -= (pts.x) * (pts.y);
    inertia_tensor_[2] -= (pts.x) * (pts.z);
    inertia_tensor_[3] += (pts.x) * (pts.x) + (pts.z) * (pts.z);
    inertia_tensor_[4] -= (pts.y) * (pts.z);
    inertia_tensor_[5] += (pts.y) * (pts.y) + (pts.z) * (pts.z);

    valid_cnt++;
  }

#if USE_NORMAL_STATISTIC
  int valid_n_cnt = 0;
  for(int pp = 0; pp < cloud->points.size();++pp)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[pp];
    if(isInvalidNormalPoint(tmp)) continue;

    cv::Point3f pts_n(cloud->points[pp].normal_x, cloud->points[pp].normal_y, cloud->points[pp].normal_z);
    n_inertia_tensor_[0] += (pts_n.x) * (pts_n.x) + (pts_n.y)*(pts_n.y);
    n_inertia_tensor_[1] -= (pts_n.x) * (pts_n.y);
    n_inertia_tensor_[2] -= (pts_n.x) * (pts_n.z);
    n_inertia_tensor_[3] += (pts_n.x) * (pts_n.x) + (pts_n.z)*(pts_n.z);
    n_inertia_tensor_[4] -= (pts_n.y) * (pts_n.z);
    n_inertia_tensor_[5] += (pts_n.y) * (pts_n.y) + (pts_n.z)*(pts_n.z);

    valid_n_cnt++;
  }
#endif

  for(int i = 0; i < inertia_tensor_.size(); i++)
  {
    inertia_tensor_[i] /= static_cast<float>(valid_cnt + 1e-6);
#if USE_NORMAL_STATISTIC
    n_inertia_tensor_[i] /= static_cast<float>(valid_n_cnt + 1e-6);
#endif
  }
}

void RoboFeature::calcuCovMat(NormalPointCloudConstPtr cloud)
{
  cov_mat_.resize(6, 0);
  n_cov_mat_.resize(6, 0);

  if(cloud->empty())
    return;

  float x_avr = 0;
  float y_avr = 0;
  float z_avr = 0;

  int valid_cnt = 0;
  for(int pp = 0; pp < cloud->points.size(); ++pp)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[pp];
    if(isInvalidPoint(tmp)) continue;

    x_avr += cloud->points[pp].x;
    y_avr += cloud->points[pp].y;
    z_avr += cloud->points[pp].z;

    valid_cnt++;
  }

  x_avr /= static_cast<float>(valid_cnt + 1e-6);
  y_avr /= static_cast<float>(valid_cnt + 1e-6);
  z_avr /= static_cast<float>(valid_cnt + 1e-6);

  for(int pp = 0; pp < cloud->points.size(); ++pp)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[pp];
    if(isInvalidPoint(tmp)) continue;

    cov_mat_[0] += (cloud->points[pp].x - x_avr) * (cloud->points[pp].x - x_avr);//cov(x,x)
    cov_mat_[1] += (cloud->points[pp].x - x_avr) * (cloud->points[pp].y - y_avr);//cov(x,y)
    cov_mat_[2] += (cloud->points[pp].x - x_avr) * (cloud->points[pp].z - z_avr);//cov(x,z)
    cov_mat_[3] += (cloud->points[pp].y - y_avr) * (cloud->points[pp].y - y_avr);//cov(y,y)
    cov_mat_[4] += (cloud->points[pp].y - y_avr) * (cloud->points[pp].z - z_avr);//cov(y,z)
    cov_mat_[5] += (cloud->points[pp].z - z_avr) * (cloud->points[pp].z - z_avr);//cov(z,z)
  }

#if USE_NORMAL_STATISTIC

  float x_avr_n = 0;
  float y_avr_n = 0;
  float z_avr_n = 0;

  int valid_n_cnt = 0;
  for(int pp = 0; pp < cloud->points.size(); ++pp)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[pp];
    if(isInvalidNormalPoint(tmp)) continue;

    x_avr_n += cloud->points[pp].normal_x;
    y_avr_n += cloud->points[pp].normal_y;
    z_avr_n += cloud->points[pp].normal_z;

    valid_n_cnt++;
  }

  x_avr_n /= static_cast<float>(valid_n_cnt + 1e-6);
  y_avr_n /= static_cast<float>(valid_n_cnt + 1e-6);
  z_avr_n /= static_cast<float>(valid_n_cnt + 1e-6);

  for(int pp = 0; pp < cloud->points.size(); ++pp)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[pp];
    if(isInvalidNormalPoint(tmp)) continue;

    n_cov_mat_[0] +=
        (cloud->points[pp].normal_x - x_avr_n) * (cloud->points[pp].normal_x - x_avr_n);//cov(x,x)
    n_cov_mat_[1] +=
        (cloud->points[pp].normal_x - x_avr_n) * (cloud->points[pp].normal_y - y_avr_n);//cov(x,y)
    n_cov_mat_[2] +=
        (cloud->points[pp].normal_x - x_avr_n) * (cloud->points[pp].normal_z - z_avr_n);//cov(x,z)
    n_cov_mat_[3] +=
        (cloud->points[pp].normal_y - y_avr_n) * (cloud->points[pp].normal_y - y_avr_n);//cov(y,y)
    n_cov_mat_[4] +=
        (cloud->points[pp].normal_y - y_avr_n) * (cloud->points[pp].normal_z - z_avr_n);//cov(y,z)
    n_cov_mat_[5] +=
        (cloud->points[pp].normal_z - z_avr_n) * (cloud->points[pp].normal_z - z_avr_n);//cov(z,z)
  }
#endif


  for(int i = 0; i < 6; ++i)
  {
    cov_mat_[i] /= static_cast<float>(valid_cnt + 1e-6);
#if USE_NORMAL_STATISTIC
    n_cov_mat_[i] /= static_cast<float>(valid_n_cnt + 1e-6);
#endif
  }
}

void RoboFeature::calcuCovEigenvalue(const std::vector<float> &cov_mat, std::vector<float> &feature)
{
  feature.resize(4, 0);
  Eigen::Matrix3f C;
  int index = 0;
  for(int i = 0; i < 3; i++)
  {
    for(int j = i; j < 3; j++)
    {
      C(i, j) = cov_mat[index++];
      C(j, i) = C(i, j);
    }
  }

  Eigen::EigenSolver<Eigen::Matrix3f> es(C);
  Eigen::Matrix3f D = es.pseudoEigenvalueMatrix();
  Eigen::Matrix3f V = es.pseudoEigenvectors();

  float eigenvalue_sum = 0;
  for(int aa = 0; aa < 3; ++aa)
  {
    feature[aa] = D(aa, aa);
    eigenvalue_sum += feature[aa];
  }
  feature[3] = eigenvalue_sum;

#if 0 //Todo: need normalization ?
  eigenvalue_sum += 1e-9;
  for(int nn = 0; nn <3 ; nn++)
  {
      feature[nn] = feature[nn] / eigenvalue_sum;
  }
#endif
}

void RoboFeature::calcuMidCov(NormalPointCloudPtr cloud)
{
  mid_cov_.resize(6, 0);
  n_mid_cov_.resize(6, 0);
  cv::Point3f mid_pt1;
  int size = cloud->points.size();
  std::vector<float> xcord(size);
  std::vector<float> ycord(size);
  std::vector<float> zcord(size);

  int valid_cnt = 0;
  for(int i = 0; i < size; i++)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[i];
    if(isInvalidPoint(tmp)) continue;

    xcord[valid_cnt] = (cloud->points[i].x);
    ycord[valid_cnt] = (cloud->points[i].y);
    zcord[valid_cnt] = (cloud->points[i].z);
    valid_cnt++;
  }

  xcord.resize(valid_cnt);
  ycord.resize(valid_cnt);
  zcord.resize(valid_cnt);

  std::sort(xcord.begin(), xcord.end());
  std::sort(ycord.begin(), ycord.end());
  std::sort(zcord.begin(), zcord.end());

  //return the middle value
  mid_pt1.x = xcord[size / 2];
  mid_pt1.y = ycord[size / 2];
  mid_pt1.z = zcord[size / 2];

  for(int i = 0; i < size; i++)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[i];
    if(isInvalidPoint(tmp)) continue;

    mid_cov_[0] += ((cloud->points[i].x) - mid_pt1.x) * ((cloud->points[i].y) - mid_pt1.y);//xy
    mid_cov_[1] += ((cloud->points[i].x) - mid_pt1.x) * ((cloud->points[i].z) - mid_pt1.z);//xz
    mid_cov_[2] += ((cloud->points[i].y) - mid_pt1.y) * ((cloud->points[i].z) - mid_pt1.z);//yz
    mid_cov_[3] += ((cloud->points[i].x) - mid_pt1.x) * ((cloud->points[i].x) - mid_pt1.x);//xx
    mid_cov_[4] += ((cloud->points[i].y) - mid_pt1.y) * ((cloud->points[i].y) - mid_pt1.y);//yy
    mid_cov_[5] += ((cloud->points[i].z) - mid_pt1.z) * ((cloud->points[i].z) - mid_pt1.z);//zz
  }

#if USE_NORMAL_STATISTIC
  cv::Point3f mid_pt2;
  std::vector<float> xcord_n(size);
  std::vector<float> ycord_n(size);
  std::vector<float> zcord_n(size);

  int valid_n_cnt=0;
  for (int i = 0; i < size; i++){
    const pcl::PointXYZINormal &tmp = cloud->points[i];
    if (isInvalidNormalPoint(tmp)) continue;

    xcord_n[valid_n_cnt] = (cloud->points[i].normal_x);
    ycord_n[valid_n_cnt] = (cloud->points[i].normal_y);
    zcord_n[valid_n_cnt] = (cloud->points[i].normal_z);
    valid_n_cnt++;
  }

  xcord_n.resize(valid_n_cnt);
  ycord_n.resize(valid_n_cnt);
  zcord_n.resize(valid_n_cnt);

  std::sort(xcord_n.begin(), xcord_n.end());
  std::sort(ycord_n.begin(), ycord_n.end());
  std::sort(zcord_n.begin(), zcord_n.end());

  mid_pt2.x = xcord_n[valid_n_cnt / 2];
  mid_pt2.y = ycord_n[valid_n_cnt / 2];
  mid_pt2.z = zcord_n[valid_n_cnt / 2];

  for(int i = 0; i < size; i++)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[i];
    if (isInvalidNormalPoint(tmp)) continue;

    n_mid_cov_[0] +=
        ((cloud->points[i].normal_x) - mid_pt2.x) * ((cloud->points[i].normal_y) - mid_pt2.y);//xy
    n_mid_cov_[1] +=
        ((cloud->points[i].normal_x) - mid_pt2.x) * ((cloud->points[i].normal_z) - mid_pt2.z);//xz
    n_mid_cov_[2] +=
        ((cloud->points[i].normal_y) - mid_pt2.y) * ((cloud->points[i].normal_z) - mid_pt2.z);//yz
    n_mid_cov_[3] +=
        ((cloud->points[i].normal_x) - mid_pt2.x) * ((cloud->points[i].normal_x) - mid_pt2.x);//xx
    n_mid_cov_[4] +=
        ((cloud->points[i].normal_y) - mid_pt2.y) * ((cloud->points[i].normal_y) - mid_pt2.y);//yy
    n_mid_cov_[5] +=
        ((cloud->points[i].normal_z) - mid_pt2.z) * ((cloud->points[i].normal_z) - mid_pt2.z);//zz
  }
#endif

  for(int aa = 0; aa < 6; ++aa)
  {
    mid_cov_[aa] /= static_cast<float>(valid_cnt + 1e-6);
#if USE_NORMAL_STATISTIC
    n_mid_cov_[aa] /= static_cast<float>(valid_n_cnt + 1e-6);
#endif
  }
}

void RoboFeature::calcuIntensityStatistic(NormalPointCloudPtr cloud)//10//TODO: feature need redesign
{
  intensity_statistic_.resize(3, 0);//max_intensity, mean_intensity, dev_intensity
  float intensityMean = 0, intensityDev = 0;
  float max_intensity = -1e3;

  int valid_cnt = 0;
  for(int i = 0; i < cloud->points.size(); i++)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[i];
    if(isInvalidPoint(tmp)) continue;

    intensityMean += tmp.intensity;

    if (tmp.intensity>max_intensity)
      max_intensity = tmp.intensity;

    valid_cnt++;
  }

  intensityMean /= static_cast<float>(valid_cnt + 1e-6);

  for(int i = 0; i < cloud->points.size(); i++)
  {
    const pcl::PointXYZINormal &tmp = cloud->points[i];
    if(isInvalidPoint(tmp)) continue;

    intensityDev += (tmp.intensity - intensityMean) * (tmp.intensity - intensityMean);
  }
  intensityDev = sqrtf(intensityDev) / static_cast<float>(valid_cnt + 1e-6);

  intensity_statistic_[0] = max_intensity;
  intensity_statistic_[1] = intensityMean;
  intensity_statistic_[2] = intensityDev;
}

bool RoboFeature::isInvalidPoint(const pcl::PointXYZINormal &pts)
{
  if(pcl_isnan(pts.x) || pcl_isnan(pts.y) || pcl_isnan(pts.z) || pcl_isnan(pts.intensity))
    return true;

  return false;
}

bool RoboFeature::isInvalidNormalPoint(const pcl::PointXYZINormal &normal)
{
  if(pcl_isnan(normal.normal_x) || pcl_isnan(normal.normal_y) || pcl_isnan(normal.normal_z)
     || pcl_isnan(normal.x) || pcl_isnan(normal.y) || pcl_isnan(normal.z) || pcl_isnan(normal.intensity))
    return true;

  return false;
}
}
