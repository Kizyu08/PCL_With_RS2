#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>

namespace common {
    std::string genArgument(const std::string& argName, const std::string& help,
        const std::string& modelName, const std::string& zooFile,
        char key = ' ', std::string defaultVal = "");

    std::string findFile(const std::string& filename);

    std::string genPreprocArguments(const std::string& modelName, const std::string& zooFile);

}


