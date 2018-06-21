
#ifndef CLASSIFYPROJECT_DIRMANAGER_H
#define CLASSIFYPROJECT_DIRMANAGER_H

#include <iostream>
#include "boost/filesystem.hpp"
class dirManager {
 public:
  dirManager();
  ~dirManager();

  void getDirAndName(const std::string &dataDir, std::vector<std::string>& path_vec);
  void getNameInDir(const std::string &dataDir, std::vector<std::string>& file_vec);
};

#endif //CLASSIFYPROJECT_DIRMANAGER_H
