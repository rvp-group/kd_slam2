#include "file_utils.h"
#include <sys/types.h>
#include <dirent.h>

bool myMkdir(const std::string& dir_name) {
  int status = mkdir(dir_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (status && errno!=EEXIST) {
    return false;
  }
  return true;
}

std::string rawFilename(const std::string& s) {
  size_t pos_start=s.find_last_of('/');
  if (pos_start==string::npos){
    pos_start=0;
  } else {
    ++pos_start;
  }
  size_t pos_end=s.find_last_of('.');
  if (pos_end==string::npos) {
    return std::string();
  }
  return s.substr(pos_start,pos_end-pos_start);
}

bool isTest (const std::string& name) {
  return name.find("_test")!=string::npos;
}

bool listDir(std::set<std::string>& files,
             const std::string& dir_name,
             const std::string& extension) {
  
  DIR *dir = opendir (dir_name.c_str());
  if (dir==NULL)
    return false;

  int ext_l=extension.length();
  struct dirent *ent;
  while ((ent = readdir (dir)) != NULL) {
      std::string filename(ent->d_name);
      if (ext_l) {
        int f_l=filename.length();
        if (f_l<=ext_l)
          continue;
        std::string f_ext=filename.substr(f_l-ext_l, ext_l);
        if (f_ext!=extension)
          continue;
      }
      files.insert(filename);
    }
  closedir (dir);
  return true;
}

std::string gtName(const std::string& ev_name){
  std::string result=ev_name;
  int l=result.length();
  result[l-6]='g';
  result[l-5]='t';
  return result;
}
