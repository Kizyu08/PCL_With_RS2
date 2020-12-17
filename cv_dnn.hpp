#pragma once
#include <fstream>
#include <sstream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "common.hpp"

class cv_dnn {
private:
    float confThreshold = 0.5;
    float nmsThreshold = 0.4;
    float scale = 0.00392;

    int inpWidth = 416;
    int inpHeight = 416;
    int backend = 0;
    int target = 0;

    Scalar mean = 0; //parser.get<Scalar>("mean");
    bool swapRB = true;
    std::vector<std::string> classes;

    std::string modelPath = "C:\\lib\\darknet\\x64\\cfg\\yolov3-tiny.cfg";
    std::string configPath = "I:\\Software\\_DEV\\yolo\\tukaeta\\yolov3-tiny.weights";
    std::string file = "C:\\lib\\darknet\\x64\\data\\coco.names";

    Mat frame, blob;

    // dnn network
    Net net;

    std::vector<String> outNames;

public:
	cv_dnn();
    void exec(Mat* input, Mat* out);

private:
    void postprocess(Mat& frame, const std::vector<Mat>& out, Net& net);
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
    void callback(int pos, void* userdata);

    std::vector<String> getOutputsNames(const Net& net);
};