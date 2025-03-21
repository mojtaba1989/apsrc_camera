#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apsrc_msgs/ElapsedTime.h>
#include <apsrc_msgs/RecInfo.h>

class webcam_node
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Publisher lapsed_time_pub_, info_pub_;
	cv::VideoCapture cap_;
	apsrc_msgs::ElapsedTime msg_;
	apsrc_msgs::RecInfo info_msg_;
	cv::VideoWriter writer_;
	ros::Timer timer_;
	std::string topic_;
	
	
	// Frame
	cv::Mat frame_;
	int frame_width_;
	int frame_height_;
	int frame_rate_;
	uint64_t time_lapse_ref_;

	// Parameters
	std::string camera_name_;
	int device_id_;
	int width_;
	int height_;
	int fps_;
	std::string dir_;
	bool rotate_ = false;
  
public:
	webcam_node()
	{
		nh_ = ros::NodeHandle();
		pnh_ = ros::NodeHandle("~");
		load_param();

		lapsed_time_pub_ = nh_.advertise<apsrc_msgs::ElapsedTime>(generateTopic("elapsed_time"), 1, true);
		info_pub_ = nh_.advertise<apsrc_msgs::RecInfo>(generateTopic("info"), 1, true);
		cap_.open(device_id_);
		
		if (!cap_.isOpened())
		{
			ROS_ERROR("Failed to open camera");
			return;
		}

		ROS_INFO("Camera opened successfully");
		config();

		frame_height_ =static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
		frame_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
		frame_rate_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));
		
		ROS_INFO("Original settings: (%dx%d)@%d", frame_width_, frame_height_, frame_rate_);
		msg_.header.frame_id = "base_link";
		msg_.header.seq = 0;
	}

	void run()
	{
		if (!ros::ok())
		{
			ROS_ERROR("ROS is not ok");
			return;
		}

		if (!cap_.isOpened())
		{
			ROS_ERROR("Camera is not opened");
			return;
		}

		cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
		cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
		cap_.set(cv::CAP_PROP_FPS, fps_);

		std::string filename = generateFilename();
	
		writer_.open(filename, cv::VideoWriter::fourcc('m','p','4','v'), fps_, cv::Size(width_, height_));
		
		frame_height_ =static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
		frame_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
		frame_rate_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));
		ROS_INFO("Target settings: (%dx%d)@%d", frame_width_, frame_height_, frame_rate_);
		info_msg_.header.frame_id = "base_link";
		info_msg_.header.seq = 0;
		info_msg_.header.stamp = ros::Time::now();
		info_msg_.width = frame_width_;
		info_msg_.height = frame_height_;
		info_msg_.fps = frame_rate_;
		info_msg_.encoding = "mp4";
		info_msg_.camera_name = camera_name_;
		info_msg_.devide_id = device_id_;
		info_msg_.file_path = filename;
		info_pub_.publish(info_msg_);

		if (!writer_.isOpened())
		{
			ROS_ERROR("Video writer is not opened");
			return;
		}
		
		ROS_INFO("Video writer opened successfully, Recording...");
		timer_ = nh_.createTimer(ros::Duration(1/fps_), std::bind(&webcam_node::timerCallback, this));
	}

	void timerCallback()
	{
		cap_ >> frame_;
		if (msg_.header.seq == 0)
		{
			time_lapse_ref_ = ros::Time::now().toNSec();
		}

		if (frame_.empty())
		{
			ROS_ERROR("frame is empty");
			return;
		}
		
		if (rotate_)
		{
			cv::rotate(frame_, frame_, cv::ROTATE_180);
		}
		uint64_t time_lapse = ros::Time::now().toNSec() - time_lapse_ref_;
		std::string timestamp_text = std::to_string(msg_.header.seq);
		cv::putText(frame_, timestamp_text, cv::Point(5,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
		writer_.write(frame_);
		msg_.header.stamp = ros::Time::now();
		msg_.elapsed_time = time_lapse;
		lapsed_time_pub_.publish(msg_);
		msg_.header.seq++;
	}

	void load_param()
	{
		pnh_.param<std::string>("camera_name", camera_name_, "");
		pnh_.param("device_id", device_id_, 0);
		pnh_.param("width", width_, 640);
		pnh_.param("height", height_, 480);
		pnh_.param("fps", fps_, 15);
		pnh_.param<std::string>("dir", dir_, "/tmp");
		pnh_.param("rotate", rotate_, false);
		ROS_INFO("Parameters loaded");
	}

	void config()
	{
		int brightness;
		int contrast;
		int saturation;
		int sharpness;
		int backlight_compensation;
		int exposure_auto;
		int exposure;
		int focus_absolute;
		int focus_auto;
		pnh_.param("brightness", brightness, -1);
		pnh_.param("contrast", contrast, -1);
		pnh_.param("saturation", saturation, -1);
		pnh_.param("sharpness", sharpness, -1);
		pnh_.param("backlight_compensation", backlight_compensation, -1);
		pnh_.param("exposure_auto", exposure_auto, -1);
		pnh_.param("exposure", exposure, -1);
		pnh_.param("focus_absolute", focus_absolute, -1);
		pnh_.param("focus_auto", focus_auto, -1);

		if (brightness != -1)
		{
			cap_.set(cv::CAP_PROP_BRIGHTNESS, brightness);
			ROS_INFO("Brightness: %d", brightness);
		}
		if (contrast != -1)
		{
			cap_.set(cv::CAP_PROP_CONTRAST, contrast);
			ROS_INFO("Contrast: %d", contrast);
		}
		if (saturation != -1)
		{
			cap_.set(cv::CAP_PROP_SATURATION, saturation);
			ROS_INFO("Saturation: %d", saturation);
		}
		if (sharpness != -1)
		{
			cap_.set(cv::CAP_PROP_SHARPNESS, sharpness);
			ROS_INFO("Sharpness: %d", sharpness);
		}
		if (backlight_compensation != -1)
		{
			cap_.set(cv::CAP_PROP_BACKLIGHT, backlight_compensation);
			ROS_INFO("Backlight compensation: %d", backlight_compensation);
		}
		if (exposure_auto != -1)
		{
			cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, exposure_auto);
			ROS_INFO("Exposure auto: %d", exposure_auto);
		}
		if (exposure != -1)
		{
			cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
			ROS_INFO("Exposure: %d", exposure);
		}
		if (focus_auto != -1)
		{
			cap_.set(cv::CAP_PROP_AUTOFOCUS, focus_auto);
			ROS_INFO("Focus auto: %d", focus_auto);
		}
		if (focus_absolute != -1)
		{
			cap_.set(cv::CAP_PROP_FOCUS, focus_absolute);
			ROS_INFO("Focus absolute: %d", focus_absolute);
		}
		ROS_INFO("Camera settings configured");
	}

	std::string generateFilename() {
    ros::Time current_time = ros::Time::now();
		std::stringstream ss;
		struct tm t;
		time_t raw_time = current_time.sec;
		gmtime_r(&raw_time, &t);
		ss << camera_name_ << "_" 
				<< 1900 + t.tm_year << "_"
				<< std::setw(2) << std::setfill('0') << 1 + t.tm_mon << "_"
				<< std::setw(2) << std::setfill('0') << t.tm_mday << "_"
				<< std::setw(2) << std::setfill('0') << t.tm_hour << "_"
				<< std::setw(2) << std::setfill('0') << t.tm_min << "_"
				<< std::setw(2) << std::setfill('0') << t.tm_sec
				<< ".mp4";
		
		std::string filename = dir_ + "/" + ss.str();
		ROS_INFO("Filename: %s", filename.c_str());
		return filename;
	}

	std::string generateTopic(std::string topic)
	{
		std::stringstream ss;
		ss << "/" << camera_name_;
		if (topic == "info")
		{
			ss << "/info";
		} else if (topic == "elapsed_time")
		{
			ss << "/elapsed_time";
		}
		return ss.str();
	}

	~webcam_node()
	{
		cap_.release();
		writer_.release();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "webcam_node");
	webcam_node webcam;
	webcam.run();
	ros::spin();
	return 0;
}