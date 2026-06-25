#pragma  once
#include <string>
#include <iostream>
#include <fstream>
#include <set>
#include <sys/stat.h>

using namespace std;

std::string rawFilename(const std::string& s);
std::string stripPath(const std::string& s);

bool listDir(std::set<std::string>& files,
             const std::string& dir_name,
             const std::string& extension);

std::string gtName(const std::string& ev_name);

bool isTest (const std::string& name);

bool myMkdir(const std::string& dir_name);
