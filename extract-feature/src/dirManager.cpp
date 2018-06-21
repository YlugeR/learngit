
#include "dirManager.h"
dirManager::dirManager() {

}

dirManager::~dirManager() {

}

void dirManager::getDirAndName(const std::string &dataDir, std::vector<std::string>& path_vec) {
  std::vector<boost::filesystem::path> allSubDir;
  boost::filesystem::path path1(dataDir.c_str());
  assert(boost::filesystem::is_directory(path1));

  //get sub dirs
  if (boost::filesystem::is_directory(path1)) {
    std::copy(boost::filesystem::directory_iterator(path1),boost::filesystem::directory_iterator(),std::back_inserter(allSubDir));
    std::sort(allSubDir.begin(),allSubDir.end());

    for (std::vector<boost::filesystem::path>::iterator it(allSubDir.begin());it!=allSubDir.end();++it) {
        path_vec.push_back((*it).string());
    }
  }
}

void dirManager::getNameInDir(const std::string &dataDir, std::vector<std::string>& file_vec) {
  std::vector<boost::filesystem::path> allSubDir;
  boost::filesystem::path path1(dataDir.c_str());
  assert(boost::filesystem::is_directory(path1));

  //get sub dirs
  if (boost::filesystem::is_directory(path1)) {
    std::copy(boost::filesystem::directory_iterator(path1),boost::filesystem::directory_iterator(),std::back_inserter(allSubDir));
    std::sort(allSubDir.begin(),allSubDir.end());

    for (std::vector<boost::filesystem::path>::iterator it(allSubDir.begin());it!=allSubDir.end();++it) {
        file_vec.push_back((*it).filename().string());
    }
  }
}


