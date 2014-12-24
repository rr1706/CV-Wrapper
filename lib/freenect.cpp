#include "freenect.hpp"
#include <cmath>

myMutex::myMutex() {
	pthread_mutex_init( &m_mutex, NULL );
}
void myMutex::lock() {
	pthread_mutex_lock( &m_mutex );
}
void myMutex::unlock() {
	pthread_mutex_unlock( &m_mutex );
}

ColorFreenectDevice::ColorFreenectDevice(freenect_context *_ctx, int _index)
: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
m_new_depth_frame(false), depthMat(cv::Size(640,480),CV_16UC1),
rgbMat(cv::Size(640,480), CV_8UC3, cv::Scalar(0)),
ownMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0)) {
	
	for( unsigned int i = 0 ; i < 2048 ; i++) {
		float v = i/2048.0;
		v = std::pow(v, 3)* 6;
		m_gamma[i] = v*6*256;
	}
}

// Do not call directly even in child
void ColorFreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
	m_rgb_mutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	rgbMat.data = rgb;
	m_new_rgb_frame = true;
	m_rgb_mutex.unlock();
};

// Do not call directly even in child
void ColorFreenectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
	m_depth_mutex.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	depthMat.data = (uchar*) depth;
	m_new_depth_frame = true;
	m_depth_mutex.unlock();
}

bool ColorFreenectDevice::getVideo(cv::Mat& output) {
	m_rgb_mutex.lock();
	if(m_new_rgb_frame) {
		cv::cvtColor(rgbMat, output, CV_RGB2BGR);
		m_new_rgb_frame = false;
		m_rgb_mutex.unlock();
		return true;
	} else {
		m_rgb_mutex.unlock();
		return false;
	}
}

bool ColorFreenectDevice::getDepth(cv::Mat& output) {
	m_depth_mutex.lock();
	if(m_new_depth_frame) {
		depthMat.copyTo(output);
		m_new_depth_frame = false;
		m_depth_mutex.unlock();
		return true;
	} else {
		m_depth_mutex.unlock();
		return false;
	}
}

IRFreenectDevice::IRFreenectDevice(freenect_context *_ctx, int _index)
: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
m_buffer_rgb(FREENECT_VIDEO_IR_8BIT), m_gamma(2048), m_new_rgb_frame(false),
m_new_depth_frame(false), depthMat(cv::Size(640,480),CV_16UC1),
rgbMat(cv::Size(640,480), CV_8UC1, cv::Scalar(0)),
ownMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0)) {
	
	for( unsigned int i = 0 ; i < 2048 ; i++) {
		float v = i/2048.0;
		v = std::pow(v, 3)* 6;
		m_gamma[i] = v*6*256;
	}
	setVideoFormat(FREENECT_VIDEO_IR_8BIT);
}

// Do not call directly even in child
void IRFreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
	m_rgb_mutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	rgbMat.data = rgb;
	m_new_rgb_frame = true;
	m_rgb_mutex.unlock();
};

// Do not call directly even in child
void IRFreenectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
	m_depth_mutex.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	depthMat.data = (uchar*) depth;
	m_new_depth_frame = true;
	m_depth_mutex.unlock();
}

bool IRFreenectDevice::getVideo(cv::Mat& output) {
	m_rgb_mutex.lock();
	if(m_new_rgb_frame) {
		rgbMat.copyTo(output);
		m_new_rgb_frame = false;
		m_rgb_mutex.unlock();
		return true;
	} else {
		m_rgb_mutex.unlock();
		return false;
	}
}

bool IRFreenectDevice::getDepth(cv::Mat& output) {
	m_depth_mutex.lock();
	if(m_new_depth_frame) {
		depthMat.copyTo(output);
		m_new_depth_frame = false;
		m_depth_mutex.unlock();
		return true;
	} else {
		m_depth_mutex.unlock();
		return false;
	}
}


ColorFreenectDevice& fauxDevice()
{
	ColorFreenectDevice* dev = NULL;
	return *dev;
}

KinectCapture::KinectCapture(rgb_init rgb)
: ir(false), device(freenect.createDevice<ColorFreenectDevice>(rgb.id))
{
	device.startVideo();
	device.startDepth();
}
KinectCapture::KinectCapture(ir_init ir)
: ir(true), device(freenect.createDevice<IRFreenectDevice>(ir.id))
{
	device.startVideo();
	device.startDepth();
}

// These two following methods are here because they can't be deleted like inherited constructors can :)

bool KinectCapture::open(const std::string&)
{
	throw std::runtime_error("KinectCapture can only be initialized with constructor.");
}

bool KinectCapture::open(int i)
{
	throw std::runtime_error("KinectCapture can only be initialized with constructor.");
}

bool KinectCapture::isOpened() const
{
	return true;
}

void KinectCapture::release()
{
}

bool KinectCapture::grab()
{
	cv::Mat d;
	if (ir) {
		static_cast<IRFreenectDevice&>(device).getVideo(this->rgb);
		static_cast<IRFreenectDevice&>(device).getDepth(d);
	} else {
		static_cast<ColorFreenectDevice&>(device).getVideo(this->rgb);
		static_cast<ColorFreenectDevice&>(device).getDepth(d);
	}
	d.convertTo(this->depth, CV_8UC1, 255.0/2048.0);
	return true;
}

bool KinectCapture::retrieve(cv::Mat &image, int i)
{
	switch (i) {
		case 0:
			image = this->rgb;
			return true;
		case 1:
			image = this->depth;
			return true;
		default:
			return false;
	}
}

KinectCapture& KinectCapture::operator>>(cv::Mat& image)
{
	this->grab();
	image = this->rgb;
	return *this;
}

bool KinectCapture::read(cv::Mat &image)
{
	image = this->rgb;
	return true;
}

bool KinectCapture::set(int propId, double value)
{
	throw std::runtime_error("unsupported");
}

double KinectCapture::get(int propId)
{
	return 0.0;
}

