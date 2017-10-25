#include "ProjectorCalibrate.h"

ProjectorCalibrate::ProjectorCalibrate( cv::Size const &CheckerBoardSize, float const widthLength, float const heightLength, cv::Size const &projector_imageSize )
    :calibrate_flag( 0 ),
    imageSize_p( projector_imageSize ),
    calibrate_data( 0 ),
    gray_code_x( projector_imageSize, CV_32FC1, cv::Scalar::all( 0 ) ),
    gray_code_y( projector_imageSize, CV_32FC1, cv::Scalar::all( 0 ) ),
    frame_number( 0 )
{
    ProjectorCalibrate::CheckerBoardSize = CheckerBoardSize;
    ProjectorCalibrate::widthLength = widthLength;
    ProjectorCalibrate::heightLength = heightLength;
    auto time = std::to_string(std::chrono::duration_cast< std::chrono::seconds >(std::chrono::system_clock::now().time_since_epoch()).count());
    folder_path = ("output_" + time);
    fs::create_directories(folder_path);
    // 内部パラメータを推定する.
    //calibrate_flag |= CV_CALIB_USE_INTRINSIC_GUESS;
    // 中心点を固定する。ただmapamokでは基本はやらないっぽい？.
    // flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
    // プロジェクターは円周方向の歪み係数が少ないらしい。実際にやってみると確かに少ないので、いっそこれをオンにして0にしたほうがよさそう
    calibrate_flag |= CV_CALIB_ZERO_TANGENT_DIST;
    // やはりないとだめらしい.
    calibrate_flag |= CV_CALIB_FIX_ASPECT_RATIO;
    // mapamokで歪みの大きなプロジェクターの場合は入れるべきとあったが、Qumiの場合ないとかなりずれる.
    calibrate_flag |= CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3;
    // 投影用のグレイコード画像の生成
    // create_binary_code( gray_posi_x, gray_posi_y, gray_nega_x, gray_nega_y, imageSize_p );
    create_gray_code( gray_posi_x, gray_posi_y, gray_nega_x, gray_nega_y, imageSize_p, 10u );
    std::vector< cv::Mat > diff_image_x, diff_image_y;
    create_diff_image( diff_image_x, gray_posi_x, gray_nega_x, 100 );
    create_gray_map( this->gray_code_x, diff_image_x );
    create_diff_image(diff_image_y, gray_posi_y, gray_nega_y, 100 );
    create_gray_map(this->gray_code_y, diff_image_y);
}

ProjectorCalibrate::~ProjectorCalibrate()
{
}

void ProjectorCalibrate::get_gray_code( std::vector< cv::Mat > &pos_x, std::vector< cv::Mat > &pos_y, std::vector< cv::Mat > &nega_x, std::vector< cv::Mat > &nega_y )
{
    pos_x = this->gray_posi_x;
    pos_y = this->gray_posi_y;
    nega_x = this->gray_nega_x;
    nega_y = this->gray_nega_y;
}

// 差分画像の生成
void ProjectorCalibrate::create_diff_image( std::vector< cv::Mat > &diff_images, std::vector< cv::Mat > const &posi_images, std::vector< cv::Mat > const &nega_images, unsigned int const threshold )
{
    assert( (posi_images.size() != 0) && (nega_images.size() != 0) );
    auto const width = posi_images[0].cols;
    auto const height = posi_images[0].rows;
    for (int i = 0; i < posi_images.size(); i++)
    {
        cv::Mat tmp(cv::Size(width, height), CV_8UC1);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                auto posi_val = static_cast< int >(posi_images[i].at< unsigned char >(y, x));
                auto nega_val = static_cast< int >(nega_images[i].at< unsigned char >(y, x));
                if (posi_val - nega_val > static_cast< int >( threshold ) )
                {
                    tmp.at< unsigned char >(y, x) = 255;
                }
                else
                {
                    tmp.at< unsigned char >(y, x) = 0;
                }
            }
        }
        diff_images.emplace_back(tmp.clone());
    }
}

void ProjectorCalibrate::draw_projector_coner_point( Calibrate_data_p const &calibrate_data, cv::Mat const &gray_code_image )
{
    cv::Mat show;
    auto const max_size = (imageSize_p.width > imageSize_p.height) ? imageSize_p.width : imageSize_p.height;
    show = gray_code_image.clone()*255/max_size;
    show.convertTo( show, CV_8UC1 );
    cv::cvtColor( show, show, cv::COLOR_GRAY2BGR );
    for( auto const &tmp : calibrate_data.coner_vec_p )
    {
        cv::circle( show, tmp, 2, cv::Scalar( 0, 0, 255 ) );
    }
    cv::imshow( "projector_coner", show );
    cv::waitKey( 1 );
}

bool ProjectorCalibrate::offline_mode( cv::Mat const &camera_image, std::vector< cv::Mat > const &diff_image_x, std::vector< cv::Mat > const &diff_image_y)
{
    Calibrate_data_p tmp;
    if (!calcrate_coner(tmp, camera_image))
    {
        std::cout << " カメラでコーナーが検出できませんでした " << std::endl;
        return false;
    }
    cv::Mat gray_map_x(cv::Size(diff_image_x[0].cols, diff_image_x[0].rows), CV_32FC1, cv::Scalar(-1.0f));
    cv::Mat gray_map_y(cv::Size(diff_image_y[0].cols, diff_image_y[0].rows), CV_32FC1, cv::Scalar(-1.0f));
    for( int y = 0; y < diff_image_x[0].rows; y++ )
    {
        for( int x = 0; x < diff_image_x[0].cols; x++ )
        {
            if( diff_image_x[0].at< unsigned char >( y, x ) > 0 )
            {
                gray_map_x.at< int >( y, x ) = 0;
            }
            if (diff_image_y[0].at< unsigned char >(y, x) > 0)
            {
                gray_map_y.at< int >(y, x) = 0;
            }
        }
    }
    create_gray_map( gray_map_x, diff_image_x );
    create_gray_map( gray_map_y, diff_image_y );
    tmp.decode_image_x = std::move( gray_map_x );
    tmp.decode_image_y = std::move( gray_map_y );
    cv::Mat image_x, image_y;
   image_x = tmp.decode_image_x.clone() * 255 / 1024;
   image_y = tmp.decode_image_y.clone() * 255 / 1024;
    tmp.decode_image_x_8uc1 = image_x.clone();
    tmp.decode_image_y_8uc1 = image_y.clone();
    get_projector_image_coner_points(tmp);
    draw_projector_coner_point(tmp, gray_code_x);
    calibrate_data.emplace_back(tmp);
    frame_number = calibrate_data.size();
    std::cout << " 現在の成功枚数 : " << calibrate_data.size() << std::endl;
    return true;
}

bool ProjectorCalibrate::decode_gray_code( std::vector< cv::Mat > const &gray_cap_img_posi_x, std::vector< cv::Mat > const &gray_cap_img_posi_y, std::vector< cv::Mat > const &gray_cap_img_nega_x, std::vector< cv::Mat > const &gray_cap_img_nega_y, std::vector< cv::Point2f > const &coner_vec_c )
{
    auto const max_size = (imageSize_p.width > imageSize_p.height) ? imageSize_p.width : imageSize_p.height;
    Calibrate_data_p tmp;
    if( coner_vec_c.size() == 0)
    {
        auto const image = gray_cap_img_posi_x[ 0 ].clone(); //msbでカメラのコーナー検出
        if( !calcrate_coner( tmp, image ) )
        {
            std::cout << " カメラでコーナーが検出できませんでした " << std::endl;
            return false;
        }
    }
    else
    {
        tmp.coner_vec_c = coner_vec_c;
        tmp.pattern_image = gray_cap_img_posi_x[ 0 ].clone();
        // 世界座標系の取得
        for (int i = 0; i < CheckerBoardSize.width * CheckerBoardSize.height; i++)
        {
            // ここのhightがちゃんとint計算として切り捨てられて正しくでるか？
            cv::Point3f worldPoint(static_cast< float >(i % CheckerBoardSize.width * widthLength), static_cast< float >(i / CheckerBoardSize.width * heightLength), 0.0f);
            tmp.world_point_vec.emplace_back(worldPoint);
        }
    }
	//デコード時の輝度のしきい値を指定（通常は40）
    auto flag = decode_gray_code( tmp, gray_cap_img_posi_x, gray_cap_img_nega_x, 40u, true, false ); // xをデコード
    flag &= decode_gray_code( tmp, gray_cap_img_posi_y, gray_cap_img_nega_y, 40u, false, false ); // yをデコード
    //auto flag = decode_gray_code(tmp, gray_cap_img_posi_x, gray_cap_img_nega_x, 40u, true ); // xをデコード
    //flag &= decode_gray_code(tmp, gray_cap_img_posi_y, gray_cap_img_nega_y, 40u, false ); // yをデコード

    if( !flag )
    {
        std::cout << " グレイコード法が動作しませんでした " << std::endl;
        return false;
    }
    cv::Mat show_x, show_y;
    show_x = tmp.decode_image_x.clone()*255/max_size;
    show_y = tmp.decode_image_y.clone()*255/max_size;
    show_x.convertTo(show_x, CV_8UC1);
    show_y.convertTo(show_y, CV_8UC1);
    cv::imshow("decode_x", show_x);
    cv::imshow("decode_y", show_y);
    tmp.decode_image_x_8uc1 = show_x.clone();
    tmp.decode_image_y_8uc1 = show_y.clone();
    cv::waitKey(1);
    get_projector_image_coner_points( tmp );
    draw_projector_coner_point(tmp, gray_code_x);
    calibrate_data.emplace_back( tmp );
    frame_number = calibrate_data.size();
    std::cout << " 現在の成功枚数 : " << calibrate_data.size() << std::endl;
}
void ProjectorCalibrate::create_gray_map( cv::Mat &gray_map, std::vector< cv::Mat > const &diff_images )
{
    auto const max_size = (imageSize_p.width > imageSize_p.height) ? imageSize_p.width : imageSize_p.height;
    auto const max_bit = static_cast< unsigned int >(log2(max_size));
    for (int i = 1; i < diff_images.size(); i++) // 単色白の画像を抜くので 1 から始まる
    {
        for (int y = 0; y < gray_map.rows; y++)
        {
            for (int x = 0; x < gray_map.cols; x++)
            {
                if (gray_map.at< float >(y, x) > -1.0f)
                {
                    if (diff_images[i].at< unsigned char >(y, x))
                        gray_map.at< float >(y, x) += static_cast< float >(1 << (max_bit - i));
                }
            }
        }
    }
}


bool ProjectorCalibrate::decode_gray_code( Calibrate_data_p &calibrate_data, std::vector< cv::Mat > const &posi_images, std::vector< cv::Mat > const &nega_images, unsigned int const binary_threshold, bool const direction, bool const isMedianFilter )
{
    std::vector< cv::Mat > diff_images;
    const unsigned int median_karnel_size = 5u;
    create_diff_image( diff_images, posi_images, nega_images, binary_threshold );
    //for( int i = 0; i < diff_images.size(); i++ )
    //{
    //    cv::imshow( "diff", diff_images[i] );
    //    cv::waitKey( 0 );
    //}
    auto const msb_img = diff_images[ 0 ].clone();
    cv::Mat gray_map( cv::Size( msb_img.cols, msb_img.rows ), CV_32FC1, cv::Scalar( -1.0f ) );
    std::size_t positive_pixel_count = 0u;
    auto const max_size = ( imageSize_p.width > imageSize_p.height ) ? imageSize_p.width : imageSize_p.height;
    auto const max_bit = static_cast< unsigned int >(log2(max_size));
    
    for( int y = 0; y < msb_img.rows; y++ )
    {
        auto const pos_msb = msb_img.ptr< unsigned char >( y );
        for( int x = 0; x < msb_img.cols; x++ )
        {
            if( pos_msb[ x ] )
            {
                gray_map.at< float >( y, x ) += 1.0f;
                positive_pixel_count++;
            }
        }
    }
    // プロジェクタで投影した映像がカメラ画像内で100分の1に達していない場合は検出できていないと判断
    if( positive_pixel_count < static_cast< std::size_t >( msb_img.rows * msb_img.cols / 100 ) )
        return false;
    create_gray_map( gray_map, diff_images );
    if( isMedianFilter )
        cv::medianBlur( gray_map, gray_map, median_karnel_size );
    if( direction )
    {
        calibrate_data.decode_image_x = gray_map.clone();
        calibrate_data.gray_code_image_x = diff_images;
    }
    else
    {
        calibrate_data.decode_image_y = gray_map.clone(); 
        calibrate_data.gray_code_image_y = diff_images;
    }
    return true;
}

void ProjectorCalibrate::update_coner(std::vector< cv::Point2f > const &coner, unsigned int const update_index )
{
    calibrate_data[ update_index ].coner_vec_c = coner;
    get_projector_image_coner_points( calibrate_data[ update_index ] );
    draw_projector_coner_point( calibrate_data[ update_index ], gray_code_x );
}


void ProjectorCalibrate::get_projector_image_coner_points( Calibrate_data_p &calibrate_data )
{
    auto decode_to_pixel = [ & ]( float const x, float const y )
    {
        auto const max_size = imageSize_p.width > imageSize_p.height ? imageSize_p.width : imageSize_p.height;
        // auto const pointx = max_size - static_cast< int >( x );
        auto const pointx = static_cast< int >( 1024 - x ); // 左上原点(opencvの画像座標系)の想定.グレイコードが左→右に明るくなるコードを投影
        // auto const pointy = max_size - static_cast< int >( y ); // 左上原点(opencvの画像座標系)の想定.グレイコードが下→上に明るくなるコードを投影しているからmax_sizeから引く
        auto const pointy = static_cast< int >( y );
        return cv::Point2f( pointx, pointy );
    };
    std::vector< cv::Point2f > coner_tmp_p;
    for( auto const &coner_tmp : calibrate_data.coner_vec_c )
    {
        auto const x = calibrate_data.decode_image_x.at< float >( coner_tmp.y, coner_tmp.x );
        auto const y = calibrate_data.decode_image_y.at< float >( coner_tmp.y, coner_tmp.x );
        // auto pixel = decode_to_pixel( x, y );
        auto pixel = decode_to_pixel( static_cast< float >( gray_to_binary( static_cast< unsigned int >( x ) ) ), static_cast< float >( gray_to_binary( static_cast< unsigned int >( y ) ) ) );
        coner_tmp_p.emplace_back( std::move( pixel ) );
    }
    calibrate_data.coner_vec_p = coner_tmp_p;
}

void ProjectorCalibrate::calcrate_intrinsec_param()
{
    std::cout << " 内部パラメタを計算しています... " << std::endl;
    calcrate_intrinsec_param(calibrate_data, projectorMatrix, distCoeffs, rvecs, tvecs, imageSize_p );
    std::cout << projectorMatrix << std::endl;
    std::cout << " 計算完了しました " << std::endl;
}

void ProjectorCalibrate::show_reprojection_image()
{
    show_reprojection_image(calibrate_data, rvecs, tvecs, projectorMatrix, distCoeffs);
}

void ProjectorCalibrate::out_put_image()
{
    out_put_image(calibrate_data);
}

bool ProjectorCalibrate::calcrate_coner(Calibrate_data_p &calibrate_data, cv::Mat const &checker_image)
{
    // 初期化
    Calibrate_data_p tmp;
    tmp.coner_vec_c.resize(0);
    tmp.world_point_vec.resize(0);
    tmp.pattern_found = false;
    tmp.pattern_image = checker_image.clone();

    // コーナー検出
    std::vector<cv::Point2f> imagePointsBuf;
    bool isDetected = false;
    isDetected = cv::findChessboardCorners(tmp.pattern_image, CheckerBoardSize, imagePointsBuf);
    if (isDetected)
    {
        cv::cornerSubPix(tmp.pattern_image, imagePointsBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        tmp.coner_vec_c = imagePointsBuf;
        tmp.pattern_found = true;
    }
    cv::Mat show = tmp.pattern_image.clone();
    cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
    cv::drawChessboardCorners(show, CheckerBoardSize, (cv::Mat)(imagePointsBuf), isDetected);
    cv::imshow("result", show);

    // 世界座標系の取得
    for (int i = 0; i < CheckerBoardSize.width * CheckerBoardSize.height; i++)
    {
        // ここのhightがちゃんとint計算として切り捨てられて正しくでるか？
        cv::Point3f worldPoint(static_cast< float >(i % CheckerBoardSize.width * widthLength), static_cast< float >(i / CheckerBoardSize.width * heightLength), 0.0f);
        tmp.world_point_vec.emplace_back(worldPoint);
    }

    // 成功したか失敗したか
    if (tmp.pattern_found)
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

void ProjectorCalibrate::calcrate_intrinsec_param(std::vector< Calibrate_data_p > const &calibrate_datas, cv::Mat &ProjectorMatrix, cv::Mat &distCoeffs, std::vector< cv::Mat > &rvecs, std::vector< cv::Mat > &tvecs, cv::Size const &imageSize )
{
    cv::Mat distCoeffs_tmp;
    cv::Mat projectorMatrix_tmp = this->projectorMatrix.clone();
    std::vector< cv::Mat > rvecs_tmp, tvecs_tmp;
    std::vector< std::vector< cv::Point3f > > worldPoints;
    std::vector< std::vector< cv::Point2f > > coner_vec;
    for (auto const &tmp : calibrate_datas)
    {
        worldPoints.emplace_back(tmp.world_point_vec);
        coner_vec.emplace_back(tmp.coner_vec_p);
    }
    cv::calibrateCamera( worldPoints, coner_vec, imageSize, projectorMatrix_tmp, distCoeffs_tmp, rvecs_tmp, tvecs_tmp, calibrate_flag);
    ProjectorMatrix = projectorMatrix_tmp.clone();
    distCoeffs = distCoeffs_tmp.clone();
    tvecs = tvecs_tmp;
    rvecs = rvecs_tmp;
}


void ProjectorCalibrate::show_reprojection_image(std::vector< Calibrate_data_p > const &calibrate_data, const std::vector< cv::Mat > &rvec, const std::vector< cv::Mat > &tvec, const cv::Mat& ProjectorMatrix, const cv::Mat& distCoeffs)
{
    std::vector< std::vector< cv::Point2f > > imagePoint_vec(tvec.size());
    for (int i = 0; i < calibrate_data.size(); i++)
    {
        cv::projectPoints(calibrate_data[i].world_point_vec, rvec[i], tvec[i], ProjectorMatrix, distCoeffs, imagePoint_vec[i]);
        auto const max_size = (imageSize_p.width > imageSize_p.height) ? imageSize_p.width : imageSize_p.height;
        cv::Mat show = gray_code_x.clone() * 255 / max_size;
        show.convertTo(show, CV_8UC1);
        cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
        for (int j = 0; j < imagePoint_vec[i].size(); j++)
        {
            cv::circle(show, calibrate_data[i].coner_vec_p[j], 3, cv::Scalar( 0, 0, 255 ) );
            cv::circle(show, imagePoint_vec[i][j], 3, cv::Scalar(0, 255, 0));
        }
        cv::imshow("show", show);
        cv::waitKey(0);
    }
}

bool ProjectorCalibrate::out_put_parameter()
{
    {
        std::ofstream fs_Projector(folder_path / "projectorMatrix.dat");
        assert(projectorMatrix.rows == 3 && projectorMatrix.cols == 3);
        cv::Mat_< double > pm = projectorMatrix.clone();
        fs_Projector <<
            pm(0, 0) << " " << pm(0, 1) << " " << pm(0, 2) << "\n" <<
            pm(1, 0) << " " << pm(1, 1) << " " << pm(1, 2) << "\n" <<
            pm(2, 0) << " " << pm(2, 1) << " " << pm(2, 2) << "\n";
        fs_Projector << std::flush;
    }
    {
        std::ofstream fs_coeffs(folder_path / "projectorCoeffs.dat");
        assert(distCoeffs.rows == 1);
        for (auto i = 0; i < distCoeffs.cols; ++i)
        {
            fs_coeffs << (i == 0 ? "" : " ") << distCoeffs.at< double >(i);
        }
        fs_coeffs << std::endl;
    }
    return true;
}

void ProjectorCalibrate::out_put_image(std::vector < Calibrate_data_p > const &calibrate_data)
{
    fs::path diff_image_x_folder_path(folder_path.string() + "/diff_x");
    fs::path diff_image_y_folder_path(folder_path.string() + "/diff_y");
    fs::create_directories(diff_image_x_folder_path);
    fs::create_directories(diff_image_y_folder_path);
    for (int i = 0; i < calibrate_data.size(); i++)
    {
        std::string filename = folder_path.string() + "/cap_img_" + std::to_string(i) + ".png";
        if( calibrate_data[i].pattern_image.data != 0 ) cv::imwrite(filename, calibrate_data[i].pattern_image);
        filename = folder_path.string() + "/decode_x" + std::to_string(i) + ".png";
        cv::imwrite( filename, calibrate_data[i].decode_image_x_8uc1 );
        filename = folder_path.string() + "/decode_y" + std::to_string(i) + ".png";
        cv::imwrite(filename, calibrate_data[i].decode_image_y_8uc1);
        for( int j = 0; j < calibrate_data[i].gray_code_image_x.size(); j++ )
        {
            auto file_name_x = diff_image_x_folder_path.string() + "/" + std::to_string( i ) + "_" + std::to_string( j ) + ".png";
            auto file_name_y = diff_image_y_folder_path.string() + "/" + std::to_string( i ) + "_" + std::to_string( j ) + ".png";
            cv::imwrite( file_name_x, calibrate_data[i].gray_code_image_x[j] );
            cv::imwrite( file_name_y, calibrate_data[i].gray_code_image_y[j] );
        }
    }
}

void ProjectorCalibrate::delete_one_frame()
{
    calibrate_data.pop_back();
    frame_number = calibrate_data.size();
    std::cout << " 現在の成功枚数 : " << calibrate_data.size() << std::endl;
}

void ProjectorCalibrate::set_intrinsec_param(cv::Mat const &projectorMatrix)
{
    calibrate_flag |= CV_CALIB_USE_INTRINSIC_GUESS;
    this->projectorMatrix = projectorMatrix.clone();
    std::cout << this->projectorMatrix << std::endl;
}

void ProjectorCalibrate::set_intrinsec_param( float const fx, float const fy, float const uc, float const vc )
{
    cv::Mat1d cameraMatrix_tmp(3, 3, 0.0f);
    cameraMatrix_tmp(0, 0) = fx;
    cameraMatrix_tmp(1, 1) = fy;
    cameraMatrix_tmp(0, 2) = uc;
    cameraMatrix_tmp(1, 2) = vc;
    cameraMatrix_tmp(2, 2) = 1.0f;
    set_intrinsec_param( cameraMatrix_tmp );
}

void ProjectorCalibrate::create_binary_code( std::vector< cv::Mat > &gray_image_posi_x, std::vector< cv::Mat > &gray_image_posi_y, std::vector< cv::Mat > &gray_image_nega_x, std::vector< cv::Mat > &gray_image_nega_y, cv::Size const &projection_imageSize )
{
    gray_image_nega_x.clear();
    gray_image_nega_y.clear();
    gray_image_posi_x.clear();
    gray_image_posi_y.clear();
    auto const max_size = projection_imageSize.width > projection_imageSize.height ? projection_imageSize.width : projection_imageSize.height;
    auto const max_bit = static_cast< unsigned int >( std::log2( static_cast< float >(max_size) ) ) + 1u; // 単色白の画像を投影する分、 1 枚多くなる
    for (int i = 0; i < max_bit; i++)
    {
        auto code_divide = 1u << i;
        auto const y_divide = max_size / code_divide;
        auto const x_divide = max_size / code_divide;
        cv::Mat tmp_posi_y( projection_imageSize, CV_8UC1);
        cv::Mat tmp_posi_x( projection_imageSize, CV_8UC1);
        cv::Mat tmp_nega_y( projection_imageSize, CV_8UC1);
        cv::Mat tmp_nega_x( projection_imageSize, CV_8UC1);
        bool y_flag = true;
        for (int y = 0; y < projection_imageSize.height; y++)
        {
            bool x_flag = true;
            if ((y%y_divide == 0) && (y != 0))
            {
                y_flag = y_flag ? false : true;
            }
            for (int x = 0; x < projection_imageSize.width; x++)
            {
                if ((x%x_divide == 0) && (x != 0))
                {
                    x_flag = x_flag ? false : true;
                }

                if (y_flag == true)
                {
                    tmp_posi_y.at< unsigned char >(y, x) = 255;
                    tmp_nega_y.at< unsigned char >(y, x) = 0;
                }
                else
                {
                    tmp_posi_y.at< unsigned char >(y, x) = 0;
                    tmp_nega_y.at< unsigned char >(y, x) = 255;
                }
                if (x_flag == true)
                {
                    tmp_posi_x.at< unsigned char >(y, x) = 255u;
                    tmp_nega_x.at< unsigned char >(y, x) = 0u;
                }
                else
                {
                    tmp_posi_x.at< unsigned char >(y, x) = 0u;
                    tmp_nega_x.at< unsigned char >(y, x) = 255u;
                }

            }
        }
        //cv::imshow( "tmp_posi_y", tmp_posi_y );
        //cv::imshow( "tmp_posi_x", tmp_posi_x );
        //cv::imshow( "tmp_nega_y", tmp_nega_y );
        //cv::imshow( "tmp_nega_x", tmp_nega_x );
        gray_image_posi_y.emplace_back(tmp_posi_y.clone());
        gray_image_nega_y.emplace_back(tmp_nega_y.clone());
        gray_image_posi_x.emplace_back(tmp_posi_x.clone());
        gray_image_nega_x.emplace_back(tmp_nega_x.clone());
        //cv::waitKey( 0 );

    }
}

void ProjectorCalibrate::create_gray_code(std::vector< cv::Mat > &gray_image_posi_x, std::vector< cv::Mat > &gray_image_posi_y, std::vector< cv::Mat > &gray_image_nega_x, std::vector< cv::Mat > &gray_image_nega_y, cv::Size const &projection_imageSize, unsigned int const max_bit )
{
    cv::Mat gray_image_tmp_x( projection_imageSize, CV_32SC1 );
    cv::Mat gray_image_tmp_y(projection_imageSize, CV_32SC1);
    cv::Mat binary_image_tmp_x( projection_imageSize, CV_32SC1 );
    cv::Mat binary_image_tmp_y( projection_imageSize, CV_32SC1 );
    std::vector< cv::Mat > gray_image_vec_x, gray_image_vec_y;
    std::vector< cv::Mat > gray_image_vec_inv_x, gray_image_vec_inv_y;
    gray_image_vec_x.emplace_back( projection_imageSize, CV_8UC1, cv::Scalar::all( 255 ) );
    gray_image_vec_y.emplace_back(projection_imageSize, CV_8UC1, cv::Scalar::all(255));
    gray_image_vec_inv_x.emplace_back( ~gray_image_vec_x[0] );
    gray_image_vec_inv_y.emplace_back( ~gray_image_vec_y[0] );
    auto width = projection_imageSize.width / ( 1 << max_bit );
    auto height = projection_imageSize.height / ( 1 << max_bit );
    if( height == 0 && width == 0 )
    {
        std::runtime_error(" 指定したbitが大きすぎます ");
    }
    else if( height == 0 || width == 0 )
    {
        if( height > width )
            width = height;
        else
            height = width;
    }
    int count_y = 0;
    for( int y = 0; y < projection_imageSize.height; y++ )
    { 
        if( (y!=0) && ( (y%height) == 0 ) ) count_y++;
        auto const pos_x_b = binary_image_tmp_x.ptr< int >( y );
        auto const pos_y_b = binary_image_tmp_y.ptr< int >( y );
        auto const pos_x_g = gray_image_tmp_x.ptr< int >( y );
        auto const pos_y_g = gray_image_tmp_y.ptr< int >( y );
        int count_x = 0;
        for( int x = 0; x < projection_imageSize.width; x++ )
        {
            if( (x!=0) && ( (x%width) == 0 ) ) count_x++;
            pos_x_b[ x ] = count_x;
            pos_y_b[ x ] = count_y;
            pos_x_g[ x ] = static_cast< int >( binary_to_gray( static_cast< unsigned int >( count_x ) ) );
            pos_y_g[ x ] = static_cast< int >( binary_to_gray( static_cast< unsigned int >( count_y ) ) );
        }
    }
    /*
    cv::Mat tmp1 = gray_image_tmp_x.clone()*256/1024;
    tmp1.convertTo( tmp1, CV_8UC1 );
    cv::imshow( "x", tmp1 );
    cv::waitKey( 0 );
    */
	//グレイコードの輝度を指定（通常は255）
    for( int bit = max_bit-1; bit >= 0; bit-- )
    {
        cv::Mat image_tmp_x( projection_imageSize, CV_8UC1, cv::Scalar::all( 0 ) );
        cv::Mat image_tmp_y( projection_imageSize, CV_8UC1, cv::Scalar::all( 0 ) );
        for( int y = 0; y < projection_imageSize.height; y++ )
        {
            auto const pos_x = image_tmp_x.ptr< unsigned char >( y );
            auto const pos_y = image_tmp_y.ptr< unsigned char >( y );
            auto const mask_x = gray_image_tmp_x.ptr< int >( y );
            auto const mask_y = gray_image_tmp_y.ptr< int >( y );
            for( int x = 0; x < projection_imageSize.width; x++ )
            {
                if( mask_x[ x ] & (1u << bit) )
                    pos_x[ x ] = 255u;
                if( mask_y[ x ] & (1u << bit) )
                    pos_y[ x ] = 255u;
            }
        }
        cv::imshow( "x", image_tmp_x );
        cv::imshow( "y", image_tmp_y );
        cv::waitKey( 0 );
        gray_image_vec_inv_x.emplace_back( ~image_tmp_x.clone() );
        gray_image_vec_inv_y.emplace_back( ~image_tmp_y.clone() );
        gray_image_vec_x.emplace_back( std::move( image_tmp_x ) );
        gray_image_vec_y.emplace_back( std::move( image_tmp_y ) );
    }
    gray_image_posi_x = std::move( gray_image_vec_x );
    gray_image_posi_y = std::move( gray_image_vec_y );
    gray_image_nega_x = std::move( gray_image_vec_inv_x );
    gray_image_nega_y = std::move( gray_image_vec_inv_y );
    
}

unsigned int ProjectorCalibrate::binary_to_gray( unsigned int const binary_code_value )
{
    unsigned int tmp = 0;
    tmp |= binary_code_value;
    return tmp^( tmp>>1 );
}

unsigned int ProjectorCalibrate::gray_to_binary( unsigned int const gray_code_value )
{
    unsigned int tmp1 = 0;
    tmp1 |= gray_code_value;
    unsigned int tmp2 = 0;
    tmp2 |= gray_code_value;
    while( tmp2 )
    {
        tmp2>>=1;
        tmp1^=tmp2;
    }
    return tmp1;
}

unsigned int ProjectorCalibrate::get_frame_number()
{
    return frame_number;
}

double ProjectorCalibrate::check_reprojection_error(std::vector< cv::Point3f > const &worldPoints, std::vector< cv::Point2f > const &conerPoints, cv::Mat const &intrinsecMatrix, cv::Mat const &distCoeffs, cv::Mat const &rvec, cv::Mat const &tvec)
{
    std::vector< cv::Point2f > imagePoints;
    cv::projectPoints( worldPoints, rvec, tvec, intrinsecMatrix, distCoeffs, imagePoints );
    double sum = 0.0f;
    for( int i = 0; i < conerPoints.size(); i++ )
    {
        auto deltaX = conerPoints[ i ].x - imagePoints[ i ].x;
        auto deltaY = conerPoints[ i ].y - imagePoints[ i ].y;
        auto distance = sqrt( deltaX*deltaX + deltaY*deltaY );
        sum += distance;
    }
    return sum/conerPoints.size();
}

double ProjectorCalibrate::calcrate_minimize_intrinsec_param( cv::Mat &projMatrix, cv::Mat &distCoeffs, unsigned int const vector_size, std::vector< Calibrate_data_p > &get_calibrate_datas )
{
    std::vector< Calibrate_data_p > tmp_vec;
    tmp_vec = this->calibrate_data;
    std::random_shuffle( tmp_vec.begin(), tmp_vec.end() );
    for( ;; )
    {
        if( tmp_vec.size() <= vector_size )
            break;
        tmp_vec.pop_back();
    }
    cv::Mat projMatrix_tmp, distCoeffs_tmp;
    std::vector< cv::Mat > rvecs, tvecs;
    calcrate_intrinsec_param( tmp_vec, projMatrix_tmp, distCoeffs_tmp, rvecs, tvecs, imageSize_p );
    double error_sum = 0;
    for( int i = 0; i < tmp_vec.size(); i++ )
    {
        error_sum += check_reprojection_error( tmp_vec[i].world_point_vec, tmp_vec[i].coner_vec_p, projMatrix_tmp, distCoeffs_tmp, rvecs[i], tvecs[i] );
    }
    projMatrix = std::move( projMatrix_tmp );
    distCoeffs = std::move( distCoeffs_tmp );
    get_calibrate_datas = tmp_vec;
    return error_sum / tmp_vec.size();
}

std::vector< Calibrate_data_p > ProjectorCalibrate::calcrate_minimize_intrinsec_param(unsigned int const loop)
{
    std::vector< Calibrate_data_p > best_calibrate_dataset;
    cv::Mat projMatrix_tmp;
    cv::Mat distCoeff_tmp;
    double error_min = 10000.0f; 
    for( int i = 0; i < loop; i++ )
    {
        cv::Mat tmp1, tmp2;
        std::vector< Calibrate_data_p > calibrate_dataset;
        auto error_tmp = calcrate_minimize_intrinsec_param( tmp1, tmp2, 20u, calibrate_dataset );
        if( error_tmp < error_min )
        {
            error_min = error_tmp;
            projMatrix_tmp = std::move( tmp1 );
            distCoeff_tmp = std::move( tmp2 );
            best_calibrate_dataset = calibrate_dataset;
        }
    }
    std::cout << "pixel誤差 : " << error_min << std::endl;
    this->distCoeffs = std::move( distCoeff_tmp );
    this->projectorMatrix = std::move( projMatrix_tmp );
    return best_calibrate_dataset;
}

cv::Mat ProjectorCalibrate::get_projectorMatrix()
{
    return this->projectorMatrix;
}

cv::Mat ProjectorCalibrate::get_distCoeffs()
{
    return this->distCoeffs;
}

fs::path ProjectorCalibrate::get_folder_path()
{
    return this->folder_path;
}
