#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <algorithm>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

class Calibrate_data_p
{
public:
    std::vector< cv::Point2f > coner_vec_c;
    std::vector< cv::Point2f > coner_vec_p;
    std::vector< cv::Point3f > world_point_vec;
    bool pattern_found;
    cv::Mat pattern_image;
    cv::Mat decode_image_x; // 32bit�̃f�R�[�h�摜
    cv::Mat decode_image_y; // 32bit�̃f�R�[�h�摜
    cv::Mat decode_image_x_8uc1; // 8bit�̃f�R�[�h�摜
    cv::Mat decode_image_y_8uc1; // 8bit�̃f�R�[�h�摜
    std::vector< cv::Mat > gray_code_image_x; // �f�R�[�h�O�̓�l�������摜
    std::vector< cv::Mat > gray_code_image_y; // �f�R�[�h�O�̓�l�������摜
};

class ProjectorCalibrate
{
private:
    cv::Size CheckerBoardSize;
    float widthLength;
    float heightLength;

    std::vector< Calibrate_data_p > calibrate_data;
    cv::Mat projectorMatrix;
    cv::Mat distCoeffs;
    fs::path folder_path;
    std::vector< cv::Mat > rvecs, tvecs;
    std::vector< cv::Mat > gray_posi_x, gray_posi_y, gray_nega_x, gray_nega_y;
    cv::Size imageSize_p;
    cv::Mat gray_code_x;
    cv::Mat gray_code_y;
    int calibrate_flag; /* �L�����u���[�V�����֐����s���̐ݒ� */
    std::size_t frame_number;
public:
    ProjectorCalibrate(cv::Size const &CheckerBoardSize, float const widthLength, float const heightLength, cv::Size const &projector_imageSize );
    ProjectorCalibrate::~ProjectorCalibrate();
    void calcrate_intrinsec_param(); /* �����p�����^���v�Z���� */
    void show_reprojection_image(); /* �ē��e�����摜���o�͂��� */
    bool out_put_parameter(); /* ���߂������p�����^���t�@�C���ɏ����o�� */
    void out_put_image(); /* �����p�����^�̌v�Z�Ɏg�p�����摜���t�@�C���ɏ����o�� */
    void delete_one_frame(); /* �R�[�i�[�_�����o�����摜�̈�ԍŐV�̂��̂��폜 */
    void set_intrinsec_param( float const fx, float const fy, float const uc, float const vc ); /* �����p�����^�̏����l��^���� */
    void get_gray_code( std::vector< cv::Mat > &pos_x, std::vector< cv::Mat > &pos_y, std::vector< cv::Mat > &nega_x, std::vector< cv::Mat > &nega_y ); /* �������ꂽ�O���C�R�[�h�摜���擾���� */
    bool offline_mode( cv::Mat const &camera_image, std::vector< cv::Mat > const &image_x, std::vector< cv::Mat > const &image_y);
    bool decode_gray_code( std::vector< cv::Mat > const &gray_cap_img_posi_x, std::vector< cv::Mat > const &gray_cap_img_posi_y, std::vector< cv::Mat > const &gray_cap_img_nega_x, std::vector< cv::Mat > const &gray_cap_img_nega_y, std::vector< cv::Point2f > const &coner_vec_c = std::vector< cv::Point2f >( 0 ) ); /* �O���C�R�[�h�摜���f�R�[�h���� */
    void update_coner( std::vector< cv::Point2f > const &coner, unsigned int const update_index  );
    unsigned int get_frame_number();
    cv::Mat get_projectorMatrix();
    cv::Mat get_distCoeffs();
    fs::path get_folder_path();
    std::vector< Calibrate_data_p > calcrate_minimize_intrinsec_param( unsigned int const loop );
private:
    // void ini
    bool calcrate_coner( Calibrate_data_p &calibrate_data, cv::Mat const &image ); /* �J�����̃`�F�X�{�[�h�̃R�[�i�[�_�����o���� */
    void create_binary_code( std::vector< cv::Mat > &gray_image_posi_x, std::vector< cv::Mat > &gray_image_posi_y, std::vector< cv::Mat > &gray_image_nega_x, std::vector< cv::Mat > &gray_image_nega_y, cv::Size const &projetion_imageSize );
    void create_gray_code(std::vector< cv::Mat > &gray_image_posi_x, std::vector< cv::Mat > &gray_image_posi_y, std::vector< cv::Mat > &gray_image_nega_x, std::vector< cv::Mat > &gray_image_nega_y, cv::Size const &projetion_imageSize, unsigned int const max_bit);
    void create_diff_image( std::vector< cv::Mat > &diff_images, std::vector< cv::Mat > const &posi_images, std::vector< cv::Mat > const &nega_images, unsigned int const threshold );
    bool decode_gray_code( Calibrate_data_p &calibrate_data, std::vector< cv::Mat > const &posi_images, std::vector< cv::Mat > const &nega_images, unsigned int const binary_threshold, bool const direction, bool const isMedianFilter = true );
    void create_gray_map( cv::Mat &gray_map, std::vector< cv::Mat > const &diff_images );
    void get_projector_image_coner_points( Calibrate_data_p &calibrate_data );
    void calcrate_intrinsec_param(std::vector< Calibrate_data_p > const &calibrate_datas, cv::Mat &CameraMatrix, cv::Mat &distCoeffs, std::vector< cv::Mat > &rvecs, std::vector< cv::Mat > &tvecs, cv::Size const &imageSize );
    void show_reprojection_image(std::vector< Calibrate_data_p > const &calibrate_data, const std::vector< cv::Mat > &rvec, const std::vector< cv::Mat > &tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    void out_put_image(std::vector < Calibrate_data_p > const &calibrate_data);
    void draw_projector_coner_point( Calibrate_data_p const &calibrate_data, cv::Mat const &gray_code_image );
    void set_intrinsec_param(cv::Mat const &cameraMatrix); /* �����p�����^�̏����l��^���� */
    unsigned int gray_to_binary( unsigned int const gray_code_value );
    unsigned int binary_to_gray( unsigned int const binary_code_value );
    double check_reprojection_error( std::vector< cv::Point3f > const &worldPoints, std::vector< cv::Point2f > const &conerPoints, cv::Mat const &intrinsecMatrix, cv::Mat const &distCoeffs, cv::Mat const &rvec, cv::Mat const &tvec );
    double calcrate_minimize_intrinsec_param(cv::Mat &projMatrix, cv::Mat &distCoeffs, unsigned int const vector_size, std::vector< Calibrate_data_p > &get_calibrate_datas = std::vector< Calibrate_data_p >( 0 ) );
};