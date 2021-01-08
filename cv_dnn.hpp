#pragma once
#include <fstream>
#include <sstream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "common.hpp"

namespace libstest {
    class cv_dnn {
    private:
        float confThreshold = 0.5;
        float nmsThreshold = 0.4;
        float scale = 0.00392;

        int inpWidth = 416;
        int inpHeight = 416;
        int backend = 0;
        int target = 0;

        cv::Scalar mean = 0; //parser.get<Scalar>("mean");
        bool swapRB = true;
        std::vector<std::string> classes;

         std::string modelPath = "C:\\lib\\darknet\\x64\\cfg\\yolov3-tiny.cfg";
         std::string configPath = "I:\\Software\\_DEV\\yolo\\tukaeta\\yolov3-tiny.weights";
         std::string file = "C:\\lib\\darknet\\x64\\data\\coco.names";
        /*std::string modelPath = "C:\\Users\\onodera\\Documents\\githubRepos\\tukaeta\\yolov3-tiny.weights";
        std::string configPath = "C:\\Users\\onodera\\Documents\\githubRepos\\tukaeta\\yolov3-tiny.cfg";
        std::string file = "C:\\Users\\onodera\\Documents\\githubRepos\\tukaeta\\coco.names";*/

        cv::Mat blob;

        // dnn network
        cv::dnn::Net net;

        std::vector<cv::String> outNames;

    public:
        cv_dnn();
        void exec(cv::Mat& input, cv::Mat& out, std::vector<cv::Rect>&, std::vector<int>& classIds, std::mutex& imgMtx);

    private:
        void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& out, cv::dnn::Net& net, std::vector<cv::Rect>& boxes, std::vector<int>& classIds);
        void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
        void callback(int pos, void* userdata);

        std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);
    };

}