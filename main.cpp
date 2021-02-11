#include "main.h"

int main()
{
    //rs2pcl();
    //rs2pcd(0, 0,1280, 720);
    //rs2pcd_rab();
    //rs2cvSample();
    //rabSample();

    //ppd.detector();
    startDetector(GetIniPath().c_str());

    int w = 0;
    int h = 0;
    while (w < 1) {
        getImage();
        w = getWidth();
        h = getHeight();
        cv::waitKey(10);
    }

    int size = getDataSize();
    //std::cout << "datasize:" << size << std::endl;
    

    while (true) {
        getImage();
        cv::imshow("nanika", image);
        unsigned char* imagedata_ptr = new unsigned char[size];
        getImageData(imagedata_ptr);
        auto imagedata = *imagedata_ptr;

        getBoxes();
        auto count = getBoxesCount();
        if (count > 0) {
            float* data;
            data = new float[count * 6];
            data = getBoxesData();
            std::cout << "count: " << count << std::endl;
            std::cout << "pos x: " << data[0] << "pos y: " << data[1] << "pos z: " << data[2] << std::endl;
            delete[] data;
        }
        cv::waitKey(10);
    }
    return 0;
}

UNITYEXPORT void UNITYCALLCONV startDetector(const char* iniPath)
{
    auto iniPathStr = std::string(iniPath);
    ppd.start_detector_thread(iniPathStr);
}

UNITYEXPORT void UNITYCALLCONV stopDetector()
{
    ppd.stop_detector_thread();
}

UNITYEXPORT void UNITYCALLCONV getImage()
{
    ppd.get_texture(image);
    //cv::imshow("image", image);
}

UNITYEXPORT int UNITYCALLCONV getWidth()
{
    return image.cols;
}

UNITYEXPORT int UNITYCALLCONV getHeight()
{
    return image.rows;
}

UNITYEXPORT int UNITYCALLCONV getDataSize()
{
    return image.total() * image.elemSize();
}

UNITYEXPORT void UNITYCALLCONV getImageData(unsigned char* data)
{
    std::memcpy(data, image.data, image.total() * image.elemSize());
}

UNITYEXPORT void UNITYCALLCONV getBoxes()
{
    ppd.get_boxes(boxes);
}

UNITYEXPORT int UNITYCALLCONV getBoxesCount()
{
    return boxes.size();
}

UNITYEXPORT float* UNITYCALLCONV getBoxesData()
{
    int size = boxes.size();
    boxData = new float[size * 6];

    for (int i = 0; i < size; i++) {
        int j = i * 6;
        boxData[j + 0] = boxes[i].x;
        boxData[j + 1] = boxes[i].y;
        boxData[j + 2] = boxes[i].z;
        boxData[j + 3] = boxes[i].xLength;
        boxData[j + 4] = boxes[i].yLength;
        boxData[j + 5] = boxes[i].zLength;
    }
    return boxData;
    //std::memcpy(dataPtr, data, size);
}

std::string GetIniPath()
{
    char dir[MAX_PATH];
    ::GetModuleFileNameA(NULL, dir, MAX_PATH);  // 実行ファイルのパスを取得
    char* pdest = strrchr(dir, '\\');          // 実行ファイルのパスから
    pdest[1] = '\0';                           // 実行ファイル名だけ切り取る
    return strcat(dir, "config.ini");                // iniファイル名を付け足す。
}