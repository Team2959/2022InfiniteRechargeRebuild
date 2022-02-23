#include "utility/Filesystem.h"

bool IsExistingRegularFile(std::string filename)
{
    struct stat info;
    if(stat(filename.c_str(), &info) != 0)
    {
        return false;
    }
    return S_ISREG(info.st_mode);
}

time_t GetLastModifiedTimestamp(std::string filename)
{
    struct stat info;
    if(stat(filename.c_str(), &info) != 0)
    {
        return -1;
    }
    return info.st_mtim.tv_sec;
}

std::string GetFirstModifiedFile(std::string directory)
{
    DIR *dir;
    struct dirent *ent;
    std::string oldestFilename{};
    time_t oldestTimestamp = LONG_MAX;
    if ((dir = opendir(directory.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if(ent->d_type == DT_REG) {
                std::string absoluteFilename = directory + "/" + ent->d_name;
                time_t timestamp = GetLastModifiedTimestamp(absoluteFilename);
                if(timestamp < oldestTimestamp)
                {
                    oldestFilename = std::string(ent->d_name);
                    oldestTimestamp = timestamp;
                }
            }
        }
        closedir(dir);
    }
    return oldestFilename;
}

std::string GetFirstDirectory(std::string directory)
{
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(directory.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if(ent->d_type == DT_DIR) {
                std::string name = std::string{ent->d_name};
                if(name != ".." && name != ".") {
                    return name;
                }
            }
        }
        closedir(dir);
    }
    return std::string{};
}

