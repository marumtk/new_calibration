///
/// @file		baslerClass.cpp
/// @brief		baslerClass baslerカメラ
/// @author		Hikaru Amano
/// @date		2017/8/10 作成
/// @date		2017/8/22 最終更新
/// @version	1.0.1
///
#include "baslerClass.hpp"

unsigned int basler::camera_count = 0;
unsigned int basler::camera_number = 0;
DeviceInfoList_t basler::devices;

/// 初期化. カメラ台数確認等
basler::basler() : Camera(CAM_WIDTH, CAM_HEIGHT, CAM_FPS),
grab_strategy(GrabStrategy_OneByOne),
frameNumber(0u)
{
	PylonInitialize();
	CTlFactory& tlFactoryTmp = CTlFactory::GetInstance();
	tlFactory = &tlFactoryTmp;
	if (camera_number == 0)
	{
		camera_number = tlFactory->EnumerateDevices(devices);
		if (camera_number == 0) std::runtime_error("No camera present.");
		std::cout << "Basler Init Cam : " << camera_number << std::endl;
	}
	camera_count++;
	formatConverter.OutputPixelFormat = PixelType_RGB8packed;

	// デバッグ用
	debug_flag.trigger_delay = 0.0f;
	debug_flag.acquisition_mode = static_cast< int >(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	//debug_flag.fast_mode = static_cast< int >(paramTypeBasler::FastMode::SensorReadoutModeNormal);
	debug_flag.grab_strategy = static_cast< int >(paramTypeBasler::GrabStrategy::OneByOne);
	debug_flag.cap_type = static_cast< int >(paramTypeBasler::CaptureType::ColorGrab);
}

basler::~basler()
{
	PylonTerminate();
}

void basler::parameter_all_print()
{
	std::cout << " Width : " << width << std::endl;
	std::cout << " Height : " << height << std::endl;
	std::cout << " fps : " << fps << std::endl;
	std::cout << " gain : " << gain << std::endl;
	std::cout << " Exposure time : " << debug_flag.exposure_time << std::endl;
	std::cout << " Trigger delay : " << debug_flag.trigger_delay << std::endl;
	if (debug_flag.acquisition_mode == static_cast< int >(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate))
	{
		std::cout << " Acquisition mode : " << "FrameRate" << std::endl;
	}
	else
	{
		std::cout << " Acquisition mode : " << "Trigger" << std::endl;
	}
	if (debug_flag.fast_mode == static_cast< int >(paramTypeBasler::FastMode::SensorReadoutModeNormal))
	{
		std::cout << " Fast mode : " << "Normal" << std::endl;
	}
	else
	{
		std::cout << " Fast mode : " << "Fast" << std::endl;
	}
	if (debug_flag.grab_strategy == static_cast< int >(paramTypeBasler::GrabStrategy::OneByOne))
	{
		std::cout << " Grab strategy : " << "OneByOne" << std::endl;
	}
	else
	{
		std::cout << " Grab strategy : " << "LatestOnly" << std::endl;
	}
	std::cout << " Capture type : ";
	if (debug_flag.cap_type == static_cast< int >(paramTypeBasler::CaptureType::ColorGrab))
	{
		std::cout << "Color" << std::endl;
	}
	else if (debug_flag.cap_type == static_cast< int >(paramTypeBasler::CaptureType::MonocroGrab))
	{
		std::cout << "Mono" << std::endl;
	}
	else if (debug_flag.cap_type == static_cast< int >(paramTypeBasler::CaptureType::BayerGrab))
	{
		std::cout << "Bayer" << std::endl;
	}
}

/// 個別カメラ接続?
void basler::connect(int id)
{
	if (id < camera_number)
	{
		Cameras = new Camera_t(tlFactory->CreateDevice(devices[id]));
	}
	else
	{
		std::runtime_error("number of camera is over flow.");
	}
	std::cout << "Using device " << Cameras->GetDeviceInfo().GetModelName() << std::endl;
	Cameras->Open();
	Cameras->AcquisitionFrameRateEnable.SetValue(true);
	//Cameras->SensorReadoutMode.SetValue(Basler_UsbCameraParams::SensorReadoutMode_Normal);
	Cameras->AcquisitionFrameRate.SetValue(CAM_FPS);
	Cameras->Width.SetValue(CAM_WIDTH);
	Cameras->Height.SetValue(CAM_HEIGHT);
	// デバッグ用
	debug_flag.exposure_time = Cameras->ExposureTime.GetValue();
}

/// カメラ終了処理
void basler::disconnect()
{
	Cameras->Close();
}
/// パラメタ設定後 撮像開始 or 撮像待機
void basler::start()
{
	Cameras->StartGrabbing(grab_strategy);
}
/// 撮像終了(あれば)
void basler::stop()
{
	Cameras->StopGrabbing();
}

/// パラメタの設定
void basler::setParam(const paramTypeCamera::paramInt &pT, const int param)
{
	switch (pT)
	{
	case paramTypeCamera::paramInt::WIDTH:
		if (Cameras->Width.GetMax() >= param) width = param;
		else
		{
			std::cout << " WIDTH :: Max以上の設定値を与えているのでMaxに設定します" << std::endl;
			width = Cameras->Width.GetMax();
		}
		Cameras->Width.SetValue(width);
		break;
	case paramTypeCamera::paramInt::HEIGHT:
		if (Cameras->Height.GetMax() >= param) height = param;
		else
		{
			std::cout << " HEIGHT :: Max以上の設定値を与えているのでMaxに設定します" << std::endl;
			height = Cameras->Height.GetMax();
		}
		Cameras->Height.SetValue(param);
		break;
		//case paramType::FPS  :
		//case paramType::GAIN :
		//	std::runtime_error( " paramの引数を明示的なfloat型として与えてください " );
		//	break;
	}
}
void basler::setParam(const paramTypeCamera::paramFloat &pT, const float param)
{
	switch (pT)
	{
	case paramTypeCamera::paramFloat::FPS:
		if (Cameras->AcquisitionFrameRateEnable.GetValue() == true)
		{
			if (Cameras->ResultingFrameRate.GetValue() > param) fps = param;
			else
			{
				std::cout << " FPS :: Max以上の設定値を与えているのでMaxに設定します" << std::endl;
				fps = Cameras->ResultingFrameRate.GetValue();
			}
		}
		else
		{
			std::cout << "trigger modeとの併用はできません" << std::endl;
		}
		break;
	case paramTypeCamera::paramFloat::GAIN:
		if (Cameras->Gain.GetMax() < param)
		{
			std::cout << " GAIN :: Max以上の設定値を与えているのでMaxに設定します" << std::endl;
			gain = Cameras->Gain.GetMax();
		}
		else
		{
			gain = param;
		}
		Cameras->Gain.SetValue(param);
		break;
		//case paramType::WIDTH  :
		//case paramType::HEIGHT :
		//	std::runtime_error( " paramの引数を明示的なint型として与えてください " );
		//	break;
	}
}
void basler::setParam(const paramTypeBasler::Param &pT, const float param)
{
	switch (pT)
	{
	case paramTypeBasler::Param::ExposureTime:
		if (param > (1000000.0f / fps - 50.0f) && fps < 1000.0f) std::runtime_error("撮像レートに対して露光時間が長すぎます");
		Cameras->ExposureTime.SetValue(param);
		debug_flag.exposure_time = param;
		break;
	case paramTypeBasler::Param::TriggerDelay:
		if (Cameras->TriggerMode.GetIntValue() != Basler_UsbCameraParams::TriggerMode_On) std::runtime_error("trigger modeを先にONにしてください");
		Cameras->TriggerDelay.SetValue(param);
		debug_flag.trigger_delay = param;
		break;
	}
}
void basler::setParam(const paramTypeBasler::AcquisitionMode &pT)
{
	if (pT == paramTypeBasler::AcquisitionMode::TriggerMode)
	{
		Cameras->AcquisitionFrameRateEnable.SetValue(false);
		Cameras->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line3);
		Cameras->TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
		Cameras->TriggerMode.SetIntValue(Basler_UsbCameraParams::TriggerMode_On);
		debug_flag.acquisition_mode = static_cast< int >(paramTypeBasler::AcquisitionMode::TriggerMode);
	}
	else if (pT == paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate)
	{
		Cameras->TriggerMode.SetIntValue(Basler_UsbCameraParams::TriggerMode_Off);
		Cameras->AcquisitionFrameRateEnable.SetValue(true);
		debug_flag.acquisition_mode = static_cast< int >(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	}
}
void basler::setParam(const paramTypeBasler::FastMode &pT)
{
	if (pT == paramTypeBasler::FastMode::SensorReadoutModeFast)
	{
		Cameras->SensorReadoutMode.SetValue(Basler_UsbCameraParams::SensorReadoutMode_Fast);
		debug_flag.fast_mode = static_cast< int >(paramTypeBasler::FastMode::SensorReadoutModeFast);
	}
	else if (pT == paramTypeBasler::FastMode::SensorReadoutModeNormal)
	{
		Cameras->SensorReadoutMode.SetValue(Basler_UsbCameraParams::SensorReadoutMode_Normal);
		debug_flag.fast_mode = static_cast< int >(paramTypeBasler::FastMode::SensorReadoutModeNormal);
	}
}
void basler::setParam(const paramTypeBasler::CaptureType &pT)
{
	switch (pT)
	{
	case paramTypeBasler::CaptureType::ColorGrab:
		formatConverter.OutputPixelFormat = PixelType_RGB8packed;
		capture_type_flag = paramTypeBasler::CaptureType::ColorGrab;
		debug_flag.cap_type = static_cast< int >(paramTypeBasler::CaptureType::ColorGrab);
		break;
	case paramTypeBasler::CaptureType::MonocroGrab:
		formatConverter.OutputPixelFormat = PixelType_Mono8;
		capture_type_flag = paramTypeBasler::CaptureType::MonocroGrab;
		debug_flag.cap_type = static_cast< int >(paramTypeBasler::CaptureType::MonocroGrab);
		break;
	case paramTypeBasler::CaptureType::BayerGrab:
		formatConverter.OutputPixelFormat = PixelType_BayerGR8;
		capture_type_flag = paramTypeBasler::CaptureType::BayerGrab;
		debug_flag.cap_type = static_cast< int >(paramTypeBasler::CaptureType::BayerGrab);
		break;
	}
}
void basler::setParam(const paramTypeBasler::GrabStrategy &pT)
{
	switch (pT)
	{
	case paramTypeBasler::GrabStrategy::OneByOne:
		grab_strategy = GrabStrategy_OneByOne;
		debug_flag.grab_strategy = static_cast< int >(paramTypeBasler::GrabStrategy::OneByOne);
		break;
	case paramTypeBasler::GrabStrategy::LatestOnlyFrame:
		grab_strategy = GrabStrategy_LatestImageOnly;
		debug_flag.grab_strategy = static_cast< int >(paramTypeBasler::GrabStrategy::LatestOnlyFrame);
		break;
	}
}

/// パラメタの取得
//virtual void getParam(const paramType &pT, void* param) = 0;
int basler::getParam(const paramTypeCamera::paramInt &pT)
{
	int tmp;
	switch (pT)
	{
	case paramTypeCamera::paramInt::WIDTH:
		tmp = width;
		break;
	case paramTypeCamera::paramInt::HEIGHT:
		tmp = height;
		break;
	}
	return tmp;
}

float basler::getParam(const paramTypeCamera::paramFloat &pT)
{
	float tmp;
	switch (pT)
	{
	case paramTypeCamera::paramFloat::FPS:
		tmp = fps;
		break;
	case paramTypeCamera::paramFloat::GAIN:
		tmp = gain;
		break;
	}
	return tmp;
}
float basler::getParam(const paramTypeBasler::Param &pT)
{
	float tmp;
	switch (pT)
	{
	case paramTypeBasler::Param::ExposureTime:
		tmp = Cameras->ExposureTime.GetValue();
		break;
	case paramTypeBasler::Param::TriggerDelay:
		tmp = Cameras->TriggerDelay.GetValue();
		break;
	}
	return tmp;
}

/// 画像の取得(単眼)
void basler::captureFrame(void* data)
{
	if (Cameras->IsGrabbing())
	{
		Cameras->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
		if (ptrGrabResult->GrabSucceeded())
		{
			formatConverter.Convert(pylonImage, ptrGrabResult);
			switch (capture_type_flag)
			{
			case paramTypeBasler::CaptureType::ColorGrab:
				memcpy(data, pylonImage.GetBuffer(), width*height * 3);
				break;
			case paramTypeBasler::CaptureType::MonocroGrab:
			case paramTypeBasler::CaptureType::BayerGrab:
				memcpy(data, pylonImage.GetBuffer(), width*height);
				break;
			}
			frameNumber = ptrGrabResult->GetBlockID();
		}
	}
}