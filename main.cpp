#include "baslerClass.hpp"
#include "HighSpeedProjector.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <conio.h>
#include <thread>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <experimental/filesystem>
#include "CameraCalibrate.h"
#include "ProjectorCalibrate.h"
namespace fs = std::experimental::filesystem;

#define ONLINE_CALIBRATION
#define PROJECTOR_CALIBRATEION
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480


#ifndef _DEBUG
#pragma comment( lib, "opencv_world310.lib" )
#else
#pragma comment( lib, "opencv_world310d.lib" )
#endif


//#define ONLINE_CALIBRATION
//#define PROJECTOR_CALIBRATEION

// 何番目から必要な画像が撮像できているかをチェックする
unsigned int check_image_number( std::vector< cv::Mat > const &capture_image, bool manual )
{
    unsigned int index = 0u;
    if( !manual )
    {
        // 必ず一枚目は違うことを想定して書いている
        for( int i = 0; i < capture_image.size()-1; i++ )
        {
            auto tmp1 = capture_image[i].clone();
            auto tmp2 = capture_image[i+1].clone();
            cv::Mat dist( cv::Size( capture_image[0].cols, capture_image[0].rows ), CV_8UC1 );
            cv::absdiff( tmp2, tmp1, dist );
            std::size_t sum = 0;
            for( int y = 0; y < dist.rows; y++ )
            {
                auto const pos = dist.ptr< unsigned char >( y );
                for( int x = 0; x < dist.cols; x++ )
                {
                    if( pos[ x ] > 40 )
                    sum += 1;
                }
            }
            if( sum > ( dist.rows*dist.cols/20 ) )
            {
                index = i+1;
                break;
            }
        }
    }
    else
    {

        for( int i = 0; i < capture_image.size(); i++ )
        {
                cv::imshow(" show ", capture_image[i]);
                auto key = cv::waitKey(0);
                if( key == 'y' )
                {
                    std::cout << "index : " << i << " を選択しました " << std::endl;
                    index = i;
                    break;
                }
                else if( key == 'n' )
                {
                    std::cout << " 次の画像を表示します " << std::endl;
                    continue;
                }
        }
    }
    return index;
}


// 再投影(カメラ)
void projection_test_c(const std::vector< std::vector< cv::Point3f > > &objectPoints, const std::vector< cv::Mat > &rvec, const std::vector< cv::Mat > &tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, fs::path const &folder_path)
{
    std::vector< cv::Mat > projection_image_vec(tvec.size());
    std::vector< std::vector< cv::Point2f > > imagePoint_vec(tvec.size());
    for (int i = 0; i < tvec.size(); i++)
    {
        cv::projectPoints(objectPoints[i], rvec[i], tvec[i], cameraMatrix, distCoeffs, imagePoint_vec[i]);
        std::string fs = folder_path.string() + ("/img_" + std::to_string(i) + ".png");
        auto show = cv::imread(fs);
        for (int j = 0; j < imagePoint_vec[i].size(); j++)
        {
            cv::circle(show, imagePoint_vec[i][j], 3, cv::Scalar(0, 0, 255));
        }
        cv::imshow("show", show);
        cv::waitKey(0);
    }
}

#ifdef ONLINE_CALIBRATION;

#ifndef PROJECTOR_CALIBRATEION
int main()
{
 
    basler basler1;
    basler1.connect( 0 );
    basler1.setParam(paramTypeCamera::paramInt::WIDTH, CAMERA_WIDTH);
    basler1.setParam(paramTypeCamera::paramInt::HEIGHT, CAMERA_HEIGHT);
    basler1.setParam(paramTypeBasler::AcquisitionMode::TriggerMode);
    basler1.setParam(paramTypeBasler::CaptureType::MonocroGrab);
    // basler1.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
    // basler1.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame);
    basler1.setParam(paramTypeBasler::GrabStrategy::OneByOne);
    basler1.setParam(paramTypeBasler::Param::ExposureTime, 8000);
    basler1.setParam(paramTypeCamera::paramFloat::GAIN, 0.0f);
    
	basler basler2;
	basler2.connect(1);
	basler2.setParam(paramTypeCamera::paramInt::WIDTH, CAMERA_WIDTH);
	basler2.setParam(paramTypeCamera::paramInt::HEIGHT, CAMERA_HEIGHT);
	basler2.setParam(paramTypeBasler::AcquisitionMode::TriggerMode);
	basler2.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	basler2.setParam(paramTypeBasler::GrabStrategy::OneByOne);
	basler2.setParam(paramTypeBasler::Param::ExposureTime, 8000);
	basler2.setParam(paramTypeCamera::paramFloat::GAIN, 0.0f);

   
    cv::Size const checkerBoardSize(7, 5); // 格子点の数
    cv::Size img_size;
    float const firstLength = 25.0f, secondLength = 25.0f; // チェッカーボードのそれぞれの長さ
    // カメラキャリブレーションクラスの動作チェック
    CameraCalibrate camera_calib( checkerBoardSize, firstLength, secondLength );
    basler1.start();
    cv::Mat image( cv::Size( CAMERA_WIDTH, CAMERA_HEIGHT), CV_8UC1 );

    int count = 0;
    for( ;; )
    {
        basler1.captureFrame( image.data );
        cv::imshow( "cap", image );
        auto key = cv::waitKey( 1 );
        if (key == 27) break; // ESC
        switch (key)
        {
            case 'c':
            {
                std::string filename = "C:/Users/k2vision/Desktop/maruyama/new_calib_ver2/GrayCode/GrayCode/output_1507627098";
                filename += "/camera_img_" + std::to_string( count ) + ".png";
                cv::Mat cap = cv::imread( filename, 0 );
                count++;
                std::cout << count << std::endl;
                //cv::Mat cap = image.clone();
                // camera_calib.calcrate_coner_high_accuracy( true, 12.5f, 5.86f, cap );
                camera_calib.calcrate_coner( cap );
                break;
            }
            case 's':
            {
                camera_calib.calcrate_intrinsec_param();
                camera_calib.calcrate_coner_high_accuracy( false );
                camera_calib.out_put_parameter();
                camera_calib.out_put_image();
                camera_calib.show_reprojection_image();
                break;
            }
            case 'd' :
            {
                camera_calib.delete_one_frame();
                break;
            }
        }
    }
    basler1.stop();
    basler1.disconnect();
    return 0;
}


#else
 
int main()
{
    // 投影映像の生成
    std::vector< cv::Mat > gray_image_posi_x, gray_image_posi_y, gray_image_nega_x, gray_image_nega_y;

    basler basler1;
    basler1.connect( 0 );
    basler1.setParam(paramTypeCamera::paramInt::WIDTH, CAMERA_WIDTH);
    basler1.setParam(paramTypeCamera::paramInt::HEIGHT, CAMERA_HEIGHT);
    basler1.setParam(paramTypeBasler::AcquisitionMode::TriggerMode);
    basler1.setParam(paramTypeBasler::CaptureType::MonocroGrab);
    basler1.setParam(paramTypeBasler::GrabStrategy::OneByOne);
    basler1.setParam(paramTypeBasler::Param::ExposureTime, 8000);
    basler1.setParam(paramTypeCamera::paramFloat::GAIN, 0.0f);

	basler basler2;
	basler2.connect(1);
	basler2.setParam(paramTypeCamera::paramInt::WIDTH, CAMERA_WIDTH);
	basler2.setParam(paramTypeCamera::paramInt::HEIGHT, CAMERA_HEIGHT);
	basler2.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	basler2.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	basler2.setParam(paramTypeBasler::GrabStrategy::OneByOne);
	basler2.setParam(paramTypeBasler::Param::ExposureTime, 8000);
	basler2.setParam(paramTypeCamera::paramFloat::GAIN, 0.0f);

    cv::Size const checkerBoardSize(9, 6); // 格子点の数
    cv::Size img_size;
    float const firstLength = 25.0f, secondLength = 25.0f; // チェッカーボードのそれぞれの長さ

    // 投影と撮像
    HighSpeedProjector hsproj;
    std::vector< unsigned long > bit_sequence = { 0, 1, 2, 3, 4, 5, 6, 7 };
    std::vector< int > led_adjust = {40, 30, 30, 30, 0, 0, 0, 0 };
    // hsproj.set_projection_mode( PM::FLIP );
    hsproj.set_projection_mode( PM::PATTERN_EMBED );
    hsproj.set_parameter_value( PP::BIT_SEQUENCE, bit_sequence );
    hsproj.set_parameter_value( PP::PROJECTOR_LED_ADJUST, led_adjust );
    hsproj.set_parameter_value( PP::FRAME_RATE, 120u );
    hsproj.set_parameter_value( PP::BUFFER_MAX_FRAME, 40u );
    hsproj.set_parameter_value( PP::PATTERN_OFFSET, 0u );
    cv::Mat dammy_image( cv::Size( 1024, 768 ), CV_8UC1, cv::Scalar::all( 0 ) );
    CameraCalibrate camera_calib( checkerBoardSize, firstLength, secondLength );
    ProjectorCalibrate projector_calib( checkerBoardSize, firstLength, secondLength, cv::Size( 1024, 768 ) );
    // 初期化を使用するかどうか
    // projector_calib.set_intrinsec_param( 1896.0f, 1896.0f, 497.0f, 832.4f );
    projector_calib.get_gray_code( gray_image_posi_x, gray_image_posi_y, gray_image_nega_x, gray_image_nega_y );
    

	bool flag_basler2 = false; // basler2で撮像を行うためのフラグ
	
	std::thread thr1(
		[&]()
	{
		bool first_flag = true;
		std::cout << std::endl;
		std::cout << "スレッド1スタート" << std::endl;
		// 実際に正しくできているかの確認
		cv::imshow("dist", dammy_image);
		for (;;)
		{
			auto key = cv::waitKey(33);
			if (key == 27) break; // ESC
			switch (key)
			{
			case 'c':
			{
				// posi_x → nega_x → posi_y → nega_y
				// の順番で投影
				std::vector< cv::Mat > cap_image(500);
				for (auto i = 0; i < cap_image.size(); i++)
				{
					cap_image[i].create(cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), CV_8UC1);
				}
				auto cap_count = 0;
				basler1.start();
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
				if (first_flag)
				{
					hsproj.connect_projector();
					first_flag = false;
					flag_basler2 = true;
				}
				for (auto i = 0; i < gray_image_posi_x.size(); i++)
				{
					hsproj.send_image_8bit(gray_image_posi_x[i].data);
					basler1.captureFrame(cap_image[cap_count].data);
					cap_count++;
				}
				for (auto i = 0; i < gray_image_nega_x.size(); i++)
				{
					hsproj.send_image_8bit(gray_image_nega_x[i].data);
					basler1.captureFrame(cap_image[cap_count].data);
					cap_count++;
				}
				for (auto i = 0; i < gray_image_posi_y.size(); i++)
				{
					hsproj.send_image_8bit(gray_image_posi_y[i].data);
					basler1.captureFrame(cap_image[cap_count].data);
					cap_count++;
				}
				for (auto i = 0; i < gray_image_nega_y.size(); i++)
				{
					hsproj.send_image_8bit(gray_image_nega_y[i].data);
					basler1.captureFrame(cap_image[cap_count].data);
					cap_count++;
				}
				hsproj.send_image_8bit(dammy_image.data);
				for (;; )
				{
					basler1.captureFrame(cap_image[cap_count].data);
					cap_count++;
					if (cap_count > 250) break;
				}
				basler1.stop();
				auto index = check_image_number(cap_image, false);
				std::vector< cv::Mat > posi_x, posi_y, nega_x, nega_y;
				auto frame_number = gray_image_posi_x.size();
				int count = 0;
				for (int i = 0; i < frame_number * 4; i++)
				{
					if (((i % (frame_number)) == 0) && (i != 0))
						count++;
					if (count == 0)
					{
						posi_x.emplace_back(cap_image[index + i]);
					}
					else if (count == 1)
					{
						nega_x.emplace_back(cap_image[index + i]);
					}
					else if (count == 2)
					{
						posi_y.emplace_back(cap_image[index + i]);
					}
					else
					{
						nega_y.emplace_back(cap_image[index + i]);
					}
				}
				//if( !camera_calib.calcrate_coner( posi_x[ 0 ] ) )
				if (!camera_calib.calcrate_coner(cap_image[index - 1u]))
				{
					std::cout << "カメラ失敗" << std::endl;
					continue;
				}
				else
				{
					auto coner_vec = camera_calib.get_coner_points();
					if (!projector_calib.decode_gray_code(posi_x, posi_y, nega_x, nega_y, coner_vec))
					{
						camera_calib.delete_one_frame();
						std::cout << "プロジェクタ失敗" << std::endl;
					}
				}
				break;
			}
			case 'd':
			{
				camera_calib.delete_one_frame();
				projector_calib.delete_one_frame();
				break;
			}
			case 's':
			{
				camera_calib.calcrate_intrinsec_param();
				camera_calib.calcrate_coner_high_accuracy();
				camera_calib.out_put_image();
				camera_calib.out_put_parameter();
				projector_calib.calcrate_intrinsec_param();
				projector_calib.out_put_parameter();
				projector_calib.out_put_image();
				projector_calib.show_reprojection_image();
				flag_basler2 = false;
				break;
			}
			case 'w':
			{
				// カメラの再検出結果を反映
				/*
				unsigned int const frame_number = camera_calib.get_frame_number();
				for( int i = 0; i < frame_number; i++ )
				{
					auto coner_vec = camera_calib.get_coner_points( i );
					projector_calib.update_coner( coner_vec, i );
				}
				*/
				std::cout << "最適なデータセット模索中..." << std::endl;
				auto calibrate_dataset = projector_calib.calcrate_minimize_intrinsec_param(100u);
				projector_calib.out_put_parameter();
				std::cout << "計算完了しました" << std::endl;
				// ステレオキャリブレーションのコードを書く
				std::cout << "ステレオキャリブレーションを行います..." << std::endl;
				std::vector< std::vector< cv::Point3f > > world_points_vec;
				std::vector< std::vector< cv::Point2f > > coner_points_vec_c, coner_points_vec_p;
				for (auto const &calib_data : calibrate_dataset)
				{
					world_points_vec.emplace_back(calib_data.world_point_vec);
					coner_points_vec_c.emplace_back(calib_data.coner_vec_c);
					coner_points_vec_p.emplace_back(calib_data.coner_vec_p);
				}
				auto cameraMatrix = camera_calib.get_cameraMatrix();
				auto distCoeffs_c = camera_calib.get_distCoeffs();
				//distCoeffs_c.at< double >(4) = 0;
				auto projectorMatrix = projector_calib.get_projectorMatrix();
				auto distCoeffs_p = projector_calib.get_distCoeffs();
				cv::Mat R, T, E, F;
				cv::stereoCalibrate(world_points_vec, coner_points_vec_c, coner_points_vec_p, cameraMatrix, distCoeffs_c, projectorMatrix, distCoeffs_p, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), R, T, E, F, CV_CALIB_FIX_INTRINSIC);
				std::cout << R << std::endl;
				std::cout << T << std::endl;
				{
					std::ofstream fs_Projector(projector_calib.get_folder_path() / "R.dat");
					assert(R.rows == 3 && R.cols == 3);
					cv::Mat_< double > r = R.clone();
					fs_Projector <<
						r(0, 0) << " " << r(0, 1) << " " << r(0, 2) << "\n" <<
						r(1, 0) << " " << r(1, 1) << " " << r(1, 2) << "\n" <<
						r(2, 0) << " " << r(2, 1) << " " << r(2, 2) << "\n";
					fs_Projector << std::flush;
				}
				{
					std::ofstream fs_coeffs(projector_calib.get_folder_path() / "T.dat");
					for (auto i = 0; i < T.rows; ++i)
					{
						fs_coeffs << (i == 0 ? "" : " ") << T.at< double >(i);
					}
					fs_coeffs << std::endl;
				}
				std::cout << "計算完了しました" << std::endl;
			}
			}

		}
	}
	);

	cv::Mat cam_image(cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), CV_8UC1, cv::Scalar::all(0));
	std::thread thr2(
		[&]()
	{
		std::cout << std::endl;
		std::cout << "スレッド2スタート" << std::endl;
		basler2.start();

		for (;;)
		{
			basler2.captureFrame(cam_image.data);
		}
	}
	);

	std::thread thr3(
		[&]()
	{
		std::cout << std::endl;
		std::cout << "スレッド3スタート" << std::endl;

		for (;;)
		{
			if (flag_basler2) {
				cv::imshow("camera_image(movie)", cam_image);
				cvWaitKey(1);
			}
		}
	}
	);

	// スレッドの立ち上がり待機
	std::this_thread::sleep_for(std::chrono::seconds(1));

	thr1.join();
	thr2.join();
	thr3.join();

	basler2.stop();
    return 0;

}
#endif

#else


int main()
{
    cv::Size const checkerBoardSize(9, 6); // 格子点の数
    cv::Size img_size;
    float const firstLength = 25.0f, secondLength = 25.0f; // チェッカーボードのそれぞれの長さ

                                                           // 投影と撮像

    CameraCalibrate camera_calib(checkerBoardSize, firstLength, secondLength);
    ProjectorCalibrate projector_calib(checkerBoardSize, firstLength, secondLength, cv::Size(1024, 768));

    cv::Mat dammy_image( cv::Size( 1024, 768 ), CV_8UC1, cv::Scalar::all( 0 ) );
    cv::imshow("dist", dammy_image);
    std::string filename = "C:/Users/k2vision/Desktop/maruyama/new_calib_ver2/GrayCode/GrayCode/output_1508418716/";
    std::size_t count = 0;
    unsigned int const image_number = 10u;
    for (;;)
    {
        auto key = cv::waitKey(33);
        if (key == 27) break; // ESC
        switch (key)
        {
        case 'c':
        {
        
            cv::Mat image_c =cv::imread( filename+"camera_img_"+std::to_string(count)+".png", 0 );
            if (!camera_calib.calcrate_coner(image_c))
            {
                std::cout << "カメラ失敗" << std::endl;
                continue;
            }
            else
            {
                std::vector< cv::Mat > images_x, images_y;
                for( int i = 0; i < image_number; i++ )
                {
                    cv::Mat image_x = cv::imread(filename + "diff_x/" + std::to_string(count) + "_" + std::to_string( i ) + ".png", 0);
                    cv::Mat image_y = cv::imread(filename + "diff_y/" + std::to_string(count) + "_" + std::to_string( i ) + ".png", 0);
                    images_x.emplace_back( image_x.clone() );
                    images_y.emplace_back( image_y.clone() );

                }

                auto coner_vec = camera_calib.get_coner_points();
                if (!projector_calib.offline_mode( image_c, images_x, images_y ))
                {
					std::cout << "プロジェクタ失敗" << std::endl;
                    camera_calib.delete_one_frame();
                }
                count++;
            }
            break;
        }
        case 'd':
        {
            camera_calib.delete_one_frame();
            projector_calib.delete_one_frame();
            break;
        }
        case 's':
        {
            camera_calib.calcrate_intrinsec_param();
            camera_calib.calcrate_coner_high_accuracy();
            camera_calib.out_put_image();
            camera_calib.out_put_parameter();
            projector_calib.calcrate_intrinsec_param();
            projector_calib.out_put_parameter();
            projector_calib.out_put_image();
            projector_calib.show_reprojection_image();
            break;
        }
        case 'w':
        {
            std::cout << "最適なデータセット模索中..." << std::endl;
            auto calibrate_dataset = projector_calib.calcrate_minimize_intrinsec_param(100u);
            projector_calib.out_put_parameter();
            std::cout << "計算完了しました" << std::endl;
            // ステレオキャリブレーションのコードを書く
            std::cout << "ステレオキャリブレーションを行います..." << std::endl;
            std::vector< std::vector< cv::Point3f > > world_points_vec;
            std::vector< std::vector< cv::Point2f > > coner_points_vec_c, coner_points_vec_p;
            for (auto const &calib_data : calibrate_dataset)
            {
                world_points_vec.emplace_back(calib_data.world_point_vec);
                coner_points_vec_c.emplace_back(calib_data.coner_vec_c);
                coner_points_vec_p.emplace_back(calib_data.coner_vec_p);
            }
			//std::cout << coner_points_vec_c[0] << std::endl;
			//std::cout << coner_points_vec_p[0] << std::endl;
            auto cameraMatrix = camera_calib.get_cameraMatrix();
            std::cout << " cameraMatrix = " << cameraMatrix << std::endl;
            auto distCoeffs_c = camera_calib.get_distCoeffs();
            std::cout << " distCoeffs_c = " << distCoeffs_c << std::endl;
            auto projectorMatrix = projector_calib.get_projectorMatrix();
            std::cout << " projectorMatirx = " << projectorMatrix << std::endl;
            auto distCoeffs_p = projector_calib.get_distCoeffs();
            std::cout << " distCoeffs_p = " << distCoeffs_p << std::endl;
            cv::Mat R, T, E, F;
            cv::stereoCalibrate(world_points_vec, coner_points_vec_c, coner_points_vec_p, cameraMatrix, distCoeffs_c, projectorMatrix, distCoeffs_p, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), R, T, E, F, CV_CALIB_FIX_INTRINSIC);
            std::cout << R << std::endl;
            std::cout << T << std::endl;
            {
                std::ofstream fs_Projector(projector_calib.get_folder_path() / "R.dat");
                assert(R.rows == 3 && R.cols == 3);
                cv::Mat_< double > r = R.clone();
                fs_Projector <<
                    r(0, 0) << " " << r(0, 1) << " " << r(0, 2) << "\n" <<
                    r(1, 0) << " " << r(1, 1) << " " << r(1, 2) << "\n" <<
                    r(2, 0) << " " << r(2, 1) << " " << r(2, 2) << "\n";
                fs_Projector << std::flush;
            }
            {
                std::ofstream fs_coeffs(projector_calib.get_folder_path() / "T.dat");
                for (auto i = 0; i < T.rows; ++i)
                {
                    fs_coeffs << (i == 0 ? "" : " ") << T.at< double >(i);
                }
                fs_coeffs << std::endl;
            }
            std::cout << "計算完了しました" << std::endl;
        }
        }

    }
    return 0;
}



#endif