#pragma once
#include "baslerClass.hpp"
#include "HighSpeedProjector.h"
#include "CameraCalibrate.h"
#include "ProjectorCalibrate.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <conio.h>
#include <thread>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <experimental/filesystem>

enum class CalibrationMode
{
    CAMERA = 0,
    PROJECTOR = 1,
    CAMERA_PROJECTOR = 2
};

class Calibration
{
public:
    Calibration();
    ~Calibration();
    cv::Size CheckerBoardSize;
    float CheckerWidthLength;
    float CheckerHightLength;
    cv::Size imageSize;
    int ModeFlag;

    CameraCalibrate cameraCalibration;

};