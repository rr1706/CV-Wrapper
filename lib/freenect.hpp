//#ifdef __APPLE__
//#include <libfreenect/libfreenect.hpp>
//#else
#include <libfreenect.hpp>
//#endif
#include <vector>
#include <string>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class myMutex {
public:
	myMutex();
	void lock();
	void unlock();
private:
	pthread_mutex_t m_mutex;
};


class ColorFreenectDevice : public Freenect::FreenectDevice {
public:
	ColorFreenectDevice(freenect_context *_ctx, int _index);
	void VideoCallback(void* _rgb, uint32_t timestamp);
	void DepthCallback(void* _depth, uint32_t timestamp);
	bool getVideo(cv::Mat& output);
	bool getDepth(cv::Mat& output);
private:
	std::vector<uint8_t> m_buffer_depth;
	std::vector<uint8_t> m_buffer_rgb;
	std::vector<uint16_t> m_gamma;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;
	cv::Mat depthMat;
	cv::Mat rgbMat;
	cv::Mat ownMat;
	myMutex m_rgb_mutex;
	myMutex m_depth_mutex;
};

class IRFreenectDevice : public Freenect::FreenectDevice {
public:
	IRFreenectDevice(freenect_context *_ctx, int _index);
	void VideoCallback(void* _rgb, uint32_t timestamp);
	void DepthCallback(void* _depth, uint32_t timestamp);
	bool getVideo(cv::Mat& output);
	bool getDepth(cv::Mat& output);
private:
	std::vector<uint8_t> m_buffer_depth;
	std::vector<uint8_t> m_buffer_rgb;
	std::vector<uint16_t> m_gamma;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;
	cv::Mat depthMat;
	cv::Mat rgbMat;
	cv::Mat ownMat;
	myMutex m_rgb_mutex;
	myMutex m_depth_mutex;
};

struct rgb_init {
	int id;
};

struct ir_init {
	int id;
};

class KinectCapture : public cv::VideoCapture
{
public:
	KinectCapture() = delete;
	KinectCapture(int) = delete;
	KinectCapture(std::string&) = delete;
	KinectCapture(rgb_init rgb);
	KinectCapture(ir_init ir);
	
	virtual bool open(const cv::string &filename);
	virtual bool open(int device);
	virtual bool isOpened() const;
	void release();
	
	bool grab();
	// channel 0 is rgb/ir, 1 is depth
	bool retrieve(cv::Mat &image, int channel);
	KinectCapture& operator>>(cv::Mat& image);
	bool read(cv::Mat &image);
	
	bool set(int propId, double value);
	double get(int propId);
	bool ir;
protected:
	Freenect::Freenect freenect;
	Freenect::FreenectDevice& device;
	cv::Mat rgb, depth;
};

