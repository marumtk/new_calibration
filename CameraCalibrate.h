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
    int calibrate_flag; /* �L�����u���[�V�����֐����s���̐ݒ� */

public:
    CameraCalibrate( cv::Size const &CheckerBoardSize, float const widthLength, float const heightLength );
    ~CameraCalibrate();
    bool calcrate_coner( cv::Mat const &image ); /* �`�F�X�{�[�h�̃R�[�i�[�_�����o���� */
    bool calcrate_coner_high_accuracy( bool const isInitialized = false, float const f = 0.0f, float const pixel_size = 0.0f, cv::Mat const &imageSiz = cv::Mat() ); /* �����x�Ƀ`�F�X�{�[�h�̃R�[�i�[�_�����o���� */
    void calcrate_intrinsec_param(); /* �����p�����^���v�Z���� */
    void show_reprojection_image(); /* �ē��e�����摜���o�͂��� */
    bool out_put_parameter(); /* ���߂������p�����^���t�@�C���ɏ����o�� */
    void out_put_image(); /* �����p�����^�̌v�Z�Ɏg�p�����摜���t�@�C���ɏ����o�� */
    void delete_one_frame(); /* �R�[�i�[�_�����o�����摜�̈�ԍŐV�̂��̂��폜 */
    std::vector< cv::Point2f > get_coner_points( unsigned int const index = 0 ); /* �v�Z�����R�[�i�[�_���擾����.index�Ɏw�肪�Ȃ��ꍇ�͍ŐV�̃`�F�X�{�[�h�̃R�[�i�[�_���擾 */
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
    void set_intrinsec_param( cv::Mat const &cameraMatrix ); /* �����p�����^�̏����l��^���� */
};