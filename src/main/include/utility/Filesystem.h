#pragma once

#include <sys/stat.h>
#include <string>
#include <dirent.h>
#include <limits.h>

bool IsExistingRegularFile(std::string filename);
time_t GetLastModifiedTimestamp(std::string filename);
std::string GetFirstModifiedFile(std::string directory);
std::string GetFirstDirectory(std::string directory);