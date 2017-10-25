#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

class Calibrate_data
{
public:
    std::vector< cv::Point2f > coner_vec;
    std::vector< cv::Point3f > world_point_vec;
    bool pattern_found;
    cv::Mat pattern_image;
};
class CameraCalibrate
{
private:
    std::vector< Calibrate_data > calibrate_point;
    cv::Size CheckerBoardSize;
    float widthLength, heightLength;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    fs::path folder_path;
    std::vector< cv::Mat > rvecs, tvecs;
    std::size_t frame_number;
    int calibrate_flag; /* キャリブレーション関数実行時の設定 */

public:
    CameraCalibrate( cv::Size const &CheckerBoardSize, float const widthLength, float const heightLength );
    ~CameraCalibrate();
    bool calcrate_coner( cv::Mat const &image ); /* チェスボードのコーナー点を検出する */
    bool calcrate_coner_high_accuracy( bool const isInitialized = false, float const f = 0.0f, float const pixel_size = 0.0f, cv::Mat const &imageSiz = cv::Mat() ); /* 高精度にチェスボードのコーナー点を検出する */
    void calcrate_intrinsec_param(); /* 内部パラメタを計算する */
    void show_reprojection_image(); /* 再投影した画像を出力する */
    bool out_put_parameter(); /* 求めた内部パラメタをファイルに書き出す */
    void out_put_image(); /* 内部パラメタの計算に使用した画像をファイルに書き出す */
    void delete_one_frame(); /* コーナー点を検出した画像の一番最新のものを削除 */
    std::vector< cv::Point2f > get_coner_points( unsigned int const index = 0 ); /* 計算したコーナー点を取得する.indexに指定がない場合は最新のチェスボードのコーナー点を取得 */
    unsigned int get_frame_number();
    cv::Mat get_cameraMatrix();
    cv::Mat get_distCoeffs();

private:
    cv::Point2d distortNorm( cv::Point2d p, cv::Mat1d K, cv::Mat1d D, cv::Mat1d R );
    bool calcrate_coner(Calibrate_data &calibrate_data, cv::Mat const &image);
    bool calcrate_coner_high_accuracy( Calibrate_data &calibrate_data );
    void calcrate_intrinsec_param(std::vector< Calibrate_data > const &calibrate_datas, cv::Mat &CameraMatrix, cv::Mat &distCoeffs, std::vector< cv::Mat > &rvecs, std::vector< cv::Mat > &tvecs);
    void show_reprojection_image(std::vector< Calibrate_data > const &calibrate_data, const std::vector< cv::Mat > &rvec, const std::vector< cv::Mat > &tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    void out_put_image(std::vector < Calibrate_data > const &calibrate_data);
    void set_intrinsec_param( float const fx, float const fy, float const uc, float const vc );
    void set_intrinsec_param( cv::Mat const &cameraMatrix ); /* 内部パラメタの初期値を与える */
};