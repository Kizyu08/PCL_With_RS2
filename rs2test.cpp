#include "cv_dnn.hpp"
#include "libstest.hpp"
#include "points_position_detector.h"

using namespace libstest;

int main()
{
    //rs2pcl();
    //rs2pcd(0, 0,1280, 720);
    //rs2pcd_rab();
    //rs2cvSample();
    //rabSample();

    points_position_detector ppd;
    ppd.get_targets_position();
    return 0;
}