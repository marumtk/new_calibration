#include "CameraCalibrate.h"

CameraCalibrate::CameraCalibrate( cv::Size const &CheckerBoardSize, float const widthLength, float const heightLength )
    :calibrate_flag( 0 ),
    frame_number( 0 )
{
    CameraCalibrate::CheckerBoardSize = CheckerBoardSize;
    CameraCalibrate::widthLength = widthLength;
    CameraCalibrate::heightLength = heightLength;
    auto time = std::to_string(std::chrono::duration_cast< std::chrono::seconds >(std::chrono::system_clock::now().time_since_epoch()).count());
    folder_path = ("output_" + time);
    fs::create_directories(folder_path);
}

CameraCalibrate::~CameraCalibrate()
{
}

bool CameraCalibrate::calcrate_coner(cv::Mat const &image)
{
    Calibrate_data tmp;
    auto isDetect = calcrate_coner( tmp, image );
    if( isDetect )
    {
        calibrate_point.emplace_back( tmp );
        frame_number = calibrate_point.size();
        std::cout << " 現在の成功枚数 : " << calibrate_point.size() << std::endl;
    }
    return isDetect;
}

bool CameraCalibrate::calcrate_coner_high_accuracy( bool const isInitialized, float const f, float const pixel_size, cv::Mat const &image )
{
    if( isInitialized )
    {
        Calibrate_data tmp;
        float fx = 0, fy = 0;
        float uc = 0, vc = 0;
        fx = f*1000.0f / pixel_size;
        fy = f*1000.0f / pixel_size; 
        uc = image.cols/2;
        vc = image.rows/2;
        set_intrinsec_param( fx, fy, uc, vc );
        this->distCoeffs = cv::Mat( cv::Size( 1, 5 ), CV_32FC1, cv::Scalar::all( 0.0 ) );
        if( !calcrate_coner( tmp, image ) ) return false;
        auto isDetect = calcrate_coner_high_accuracy( tmp );
        if (isDetect)
        {
            calibrate_point.emplace_back(tmp);
            frame_number = calibrate_point.size();
            std::cout << " 現在の成功枚数 : " << calibrate_point.size() << std::endl;
        }
        return isDetect;
    }
    else
    {
        auto const MAX = 2;
        for( int loop = 0; loop < MAX; loop++ )
        {
            for( auto &tmp : calibrate_point )
            {
                calcrate_coner_high_accuracy( tmp );
            }
            calcrate_intrinsec_param();
        }
    }
    return true;
}
void CameraCalibrate::calcrate_intrinsec_param()
{
    std::cout << " 内部パラメタを計算しています... " << std::endl;
    calcrate_intrinsec_param( calibrate_point, cameraMatrix, distCoeffs, rvecs, tvecs );
    std::cout << " 計算完了しました " << std::endl;
}

void CameraCalibrate::show_reprojection_image()
{
    show_reprojection_image( calibrate_point, rvecs, tvecs, cameraMatrix, distCoeffs );
}

void CameraCalibrate::out_put_image()
{
    out_put_image( calibrate_point );
}

bool CameraCalibrate::calcrate_coner( Calibrate_data &calibrate_data, cv::Mat const &checker_image )
{
    // 初期化
    Calibrate_data tmp;
    tmp.coner_vec.resize(0);
    tmp.world_point_vec.resize(0);
    tmp.pattern_found = false;
    tmp.pattern_image = checker_image.clone();
    
    // コーナー検出
    std::vector<cv::Point2f> imagePointsBuf;
    bool isDetected = false;
    isDetected = cv::findChessboardCorners( tmp.pattern_image, CheckerBoardSize, imagePointsBuf);
    if (isDetected) 
    {
        cv::cornerSubPix( tmp.pattern_image, imagePointsBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        tmp.coner_vec = imagePointsBuf;
        tmp.pattern_found = true;
    }
    cv::Mat show = tmp.pattern_image.clone();
    cv::cvtColor( show, show, cv::COLOR_GRAY2BGR );
    cv::drawChessboardCorners(show, CheckerBoardSize, (cv::Mat)(imagePointsBuf), isDetected);
    cv::imshow( "result", show );

    // 世界座標系の取得
    for( int i = 0; i < CheckerBoardSize.width * CheckerBoardSize.height; i++ )
    {
        // ここのhightがちゃんとint計算として切り捨てられて正しくでるか？
        cv::Point3f worldPoint( static_cast< float >( i % CheckerBoardSize.width * widthLength ), static_cast< float >( i / CheckerBoardSize.width * heightLength ), 0.0f );
        tmp.world_point_vec.emplace_back( worldPoint );
    }

    // 成功したか失敗したか
    if( tmp.pattern_found )
    {
        calibrate_data = tmp;
        return true;
    }
    else
    {
        std::cout << " コーナー検出失敗 " << std::endl;
        return false;
    }
}

cv::Point2d CameraCalibrate::distortNorm(cv::Point2d p, cv::Mat1d K, cv::Mat1d D, cv::Mat1d R) 
{
    double x = (p.x - K(0, 2)) / K(0, 0);
    double y = (p.y - K(1, 2)) / K(1, 1);
    cv::Mat1d X = (cv::Mat_<double>(3, 1) << x, y, 1);
    cv::Mat1d X2 = R.inv() * X;

    double x2 = X2(0) / X2(2);
    double y2 = X2(1) / X2(2);
    double r2 = x2*x2 + y2*y2;

    // レンズ歪みの効果を追加
    double k1 = D(0);
    double k2 = D(1);
    double p1 = D(2);
    double p2 = D(3);
    double k3 = D(4);
    double x3 = x2 * (1 + k1 * r2 + k2 * r2*r2 + k3 * r2*r2*r2) + 2 * p1 * x2 * y2 + p2 * (r2 + 2 * x2*x2);
    double y3 = y2 * (1 + k1 * r2 + k2 * r2*r2 + k3 * r2*r2*r2) + p1 * (r2 + 2 * y2*y2) + 2 * p2 * x2 * y2;

    cv::Point2d ret;
    ret.x = x3 * K(0, 0) + K(0, 2);
    ret.y = y3 * K(1, 1) + K(1, 2);
    return ret;
}

bool CameraCalibrate::calcrate_coner_high_accuracy( Calibrate_data &calibrate_data )
{
    float const normGridSize = 60.0f;
    // 正規化画像作成前の準備として歪み補正
    std::vector< cv::Point2f > imagePointsOriginUndist;
    cv::undistortPoints( calibrate_data.coner_vec, imagePointsOriginUndist, cameraMatrix, distCoeffs );
    // cv::undistortPointsでPを指定しない場合には以下の処理が必要
    cv::Mat_< double > const cm = cameraMatrix;
    for (int i = 0; i < (int)imagePointsOriginUndist.size(); i++) {
        imagePointsOriginUndist[i].x = static_cast<float>(imagePointsOriginUndist[i].x * cm(0, 0) + cm(0, 2));
        imagePointsOriginUndist[i].y = static_cast<float>(imagePointsOriginUndist[i].y * cm(1, 1) + cm(1, 2));
    }
    // 正規化画像作成用のホモグラフィー行列・平行化用回転行列計算
    std::vector<cv::Point2f> imagePointsNorm;
    for (int i = 0; i < calibrate_data.coner_vec.size(); i++) {
        imagePointsNorm.push_back(cv::Point2f(( calibrate_data.world_point_vec[i].x / (float)widthLength + 1) * (float)normGridSize, ( calibrate_data.world_point_vec[i].y / (float)heightLength + 1) * (float)normGridSize));
    }
    cv::Mat1d H = cv::findHomography(imagePointsOriginUndist, imagePointsNorm);
    cv::Mat1d R = cameraMatrix.inv() * H * cameraMatrix;

    // 校正用画像正規化
    cv::Mat normImg, map1, map2;
    cv::Size normSize((CheckerBoardSize.width + 1) * normGridSize, (CheckerBoardSize.height + 1) * normGridSize);
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, normSize, CV_32FC1, map1, map2);
    cv::remap( calibrate_data.pattern_image, normImg, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // グレースケール・明るさ補正 
    cv::Mat1b gray;
    gray = normImg.clone();
    cv::cvtColor(normImg, normImg, CV_GRAY2BGR);
    if( !cv::findChessboardCorners(normImg, CheckerBoardSize, imagePointsNorm) ) return false;
    cv::cornerSubPix(gray, imagePointsNorm, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    cv::drawChessboardCorners(normImg, CheckerBoardSize, (cv::Mat)(imagePointsNorm), true);

    // 元の画像に射影しなおす
    for (int i = 0; i < (int)imagePointsNorm.size(); i++)
        calibrate_data.coner_vec[i] = distortNorm( imagePointsNorm[i], cameraMatrix, distCoeffs, R );

    cv::imshow("norm", normImg);
    cv::waitKey(1);
    return true;
}

void CameraCalibrate::calcrate_intrinsec_param( std::vector< Calibrate_data > const &calibrate_datas, cv::Mat &CameraMatrix, cv::Mat &distCoeffs, std::vector< cv::Mat > &rvecs, std::vector< cv::Mat > &tvecs )
{
    cv::Mat cameraMatrix_tmp;
    cv::Mat distCoeffs_tmp;
    std::vector< cv::Mat > rvecs_tmp, tvecs_tmp;
    std::vector< std::vector< cv::Point3f > > worldPoints;
    std::vector< std::vector< cv::Point2f > > coner_vec;
    for( auto const &tmp : calibrate_datas )
    {
        worldPoints.emplace_back( tmp.world_point_vec );
        coner_vec.emplace_back( tmp.coner_vec );
    }  
    cv::calibrateCamera(worldPoints, coner_vec, cv::Size( calibrate_datas[0].pattern_image.cols, calibrate_datas[0].pattern_image.rows ), cameraMatrix_tmp, distCoeffs_tmp, rvecs_tmp, tvecs_tmp, calibrate_flag );
    CameraMatrix = cameraMatrix_tmp.clone();
    distCoeffs = distCoeffs_tmp.clone();
    tvecs = tvecs_tmp;
    rvecs = rvecs_tmp;
}


void CameraCalibrate::show_reprojection_image( std::vector< Calibrate_data > const &calibrate_data, const std::vector< cv::Mat > &rvec, const std::vector< cv::Mat > &tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs )
{
    std::vector< std::vector< cv::Point2f > > imagePoint_vec(tvec.size());
    for (int i = 0; i < calibrate_data.size(); i++)
    {
        cv::projectPoints( calibrate_data[i].world_point_vec, rvec[i], tvec[i], cameraMatrix, distCoeffs, imagePoint_vec[i]);
        auto show = calibrate_data[i].pattern_image.clone();
        cv::cvtColor( show, show, cv::COLOR_GRAY2BGR );
        for (int j = 0; j < imagePoint_vec[i].size(); j++)
        {
            cv::circle(show, imagePoint_vec[i][j], 3, cv::Scalar(0, 255, 0));
        }
        cv::imshow("show", show);
        cv::waitKey(0);
    }
}

bool CameraCalibrate::out_put_parameter()
{
    {
        std::ofstream fs_camera(folder_path / "cameraMatrix.dat");
        assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3);
        cv::Mat_< double > cm = cameraMatrix;
        fs_camera <<
            cm(0, 0) << " " << cm(0, 1) << " " << cm(0, 2) << "\n" <<
            cm(1, 0) << " " << cm(1, 1) << " " << cm(1, 2) << "\n" <<
            cm(2, 0) << " " << cm(2, 1) << " " << cm(2, 2) << "\n";
        fs_camera << std::flush;
    }
    {
        std::ofstream fs_coeffs(folder_path / "cameraCoeffs.dat");
        assert(distCoeffs.rows == 1);
        for (auto i = 0; i < distCoeffs.cols; ++i)
        {
            fs_coeffs << (i == 0 ? "" : " ") << distCoeffs.at< double >(i);
        }
        fs_coeffs << std::endl;
    }
    return true;
}

void CameraCalibrate::out_put_image( std::vector < Calibrate_data > const &calibrate_data )
{
    for( int i = 0; i < calibrate_data.size(); i++ )
    {
        std::string filename = folder_path.string() + "/camera_img_" + std::to_string( i ) + ".png" ;
        cv::imwrite( filename, calibrate_data[i].pattern_image );
    }
}

void CameraCalibrate::delete_one_frame()
{
    calibrate_point.pop_back();
    frame_number = calibrate_point.size();
    std::cout << " 現在の成功枚数 : " << calibrate_point.size() << std::endl;
}

std::vector< cv::Point2f > CameraCalibrate::get_coner_points( unsigned int const index )
{
    assert( calibrate_point.size() != 0 );
    unsigned int tmp = calibrate_point.size() - 1u;
    if( index != 0 ) tmp = index;
    return calibrate_point[ tmp ].coner_vec;
}

void CameraCalibrate::set_intrinsec_param(float const fx, float const fy, float const uc, float const vc)
{
    cv::Mat1d cameraMatrix_tmp( 3, 3, 0.0f );
    cameraMatrix_tmp( 0, 0 ) = fx;
    cameraMatrix_tmp( 1, 1 ) = fy;
    cameraMatrix_tmp( 0, 2 ) = uc;
    cameraMatrix_tmp( 1, 2 ) = vc;
    cameraMatrix_tmp( 2, 2 ) = 1.0f;
    std::cout << cameraMatrix_tmp << std::endl;
    set_intrinsec_param( cameraMatrix_tmp );
}

void CameraCalibrate::set_intrinsec_param( cv::Mat const &cameraMatrix )
{
    calibrate_flag |= CV_CALIB_USE_INTRINSIC_GUESS;
    this->cameraMatrix = cameraMatrix;
}

unsigned int CameraCalibrate::get_frame_number()
{
    return frame_number;
}

cv::Mat CameraCalibrate::get_cameraMatrix()
{
    return this->cameraMatrix;
}

cv::Mat CameraCalibrate::get_distCoeffs()
{
    return this->distCoeffs;
}