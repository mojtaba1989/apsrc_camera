#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <apsrc_msgs/msg/elapsed_time.hpp>
#include <apsrc_msgs/msg/rec_info.hpp>

#include <chrono>


class webcam_node : public rclcpp::Node
{
private:
	rclcpp::Publisher<apsrc_msgs::msg::ElapsedTime>::SharedPtr lapsed_time_pub_;
	rclcpp::Publisher<apsrc_msgs::msg::RecInfo>::SharedPtr info_pub_;
	cv::VideoCapture cap_;
	apsrc_msgs::msg::ElapsedTime msg_;
	apsrc_msgs::msg::RecInfo info_msg_;
	cv::VideoWriter writer_;
	rclcpp::TimerBase::SharedPtr timer_;
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
	webcam_node() : Node("webcam_node")
	{
		load_param();
		lapsed_time_pub_ = this->create_publisher<apsrc_msgs::msg::ElapsedTime>(generateTopic("elapsed_time"), 1);
		auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
		info_pub_ = this->create_publisher<apsrc_msgs::msg::RecInfo>(generateTopic("info"), qos);
		cap_.open(device_id_, cv::CAP_V4L2);
		
		if (!cap_.isOpened())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
		config();

		frame_height_ =static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
		frame_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
		frame_rate_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));
		
		RCLCPP_INFO(this->get_logger(), "Original settings: (%dx%d)@%d", frame_width_, frame_height_, frame_rate_);
		msg_.header.frame_id = "base_link";
		msg_.seq = 0;
	}

	void run()
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "ROS is not ok");
			return;
		}

		if (!cap_.isOpened())
		{
			RCLCPP_ERROR(this->get_logger(), "Camera is not opened");
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
		RCLCPP_INFO(this->get_logger(), "Target settings: (%dx%d)@%d", frame_width_, frame_height_, frame_rate_);
		info_msg_.header.frame_id = "base_link";
		info_msg_.header.stamp = this->now();
		info_msg_.width = frame_width_;
		info_msg_.height = frame_height_;
		info_msg_.fps = frame_rate_;
		info_msg_.encoding = "mp4";
		info_msg_.camera_name = camera_name_;
		info_msg_.devide_id = device_id_;
		info_msg_.file_path = filename;
		info_pub_->publish(info_msg_);

		if (!writer_.isOpened())
		{
			RCLCPP_ERROR(this->get_logger(), "Video writer is not opened");
			return;
		}
		
		RCLCPP_INFO(this->get_logger(), "Video writer opened successfully, Recording...");
		auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));
		timer_ = this->create_wall_timer(period_ms, std::bind(&webcam_node::timerCallback, this));
	}

	void timerCallback()
	{
		cap_ >> frame_;
		if (msg_.seq == 0)
		{
			time_lapse_ref_ = static_cast<uint64_t>(this->now().nanoseconds());
		}

		if (frame_.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "frame is empty");
			return;
		}
		
		if (rotate_)
		{
			cv::rotate(frame_, frame_, cv::ROTATE_180);
		}
		uint64_t time_lapse = static_cast<uint64_t>(this->now().nanoseconds()) - time_lapse_ref_;
		std::string timestamp_text = std::to_string(msg_.seq);
		cv::putText(frame_, timestamp_text, cv::Point(5,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
		writer_.write(frame_);
		msg_.header.stamp = this->now();
		msg_.elapsed_time = time_lapse;
		lapsed_time_pub_->publish(msg_);
		msg_.seq++;
	}

	void load_param()
	{
		this->declare_parameter<std::string>("camera_name", "");
		this->declare_parameter("device_id", 0);
		this->declare_parameter("width", 640);
		this->declare_parameter("height", 480);
		this->declare_parameter("fps", 15);
		this->declare_parameter<std::string>("dir", "/tmp");
		this->declare_parameter("rotate", false);
		camera_name_ = this->get_parameter("camera_name").as_string();
		device_id_ = this->get_parameter("device_id").as_int();
		width_ = this->get_parameter("width").as_int();
		height_ = this->get_parameter("height").as_int();
		fps_ = this->get_parameter("fps").as_int();
		dir_ = this->get_parameter("dir").as_string();
		rotate_ = this->get_parameter("rotate").as_bool();
		RCLCPP_INFO(this->get_logger(), "Parameters loaded");
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
		this->declare_parameter("brightness", -1);
		brightness = this->get_parameter("brightness").as_int();
		this->declare_parameter("contrast", -1);
		contrast = this->get_parameter("contrast").as_int();
		this->declare_parameter("saturation", -1);
		saturation = this->get_parameter("saturation").as_int();
		this->declare_parameter("sharpness", -1);
		sharpness = this->get_parameter("sharpness").as_int();
		this->declare_parameter("backlight_compensation", -1);
		backlight_compensation = this->get_parameter("backlight_compensation").as_int();
		this->declare_parameter("exposure_auto", -1);
		exposure_auto = this->get_parameter("exposure_auto").as_int();
		this->declare_parameter("exposure", -1);
		exposure = this->get_parameter("exposure").as_int();
		this->declare_parameter("focus_absolute", -1);
		focus_absolute = this->get_parameter("focus_absolute").as_int();
		this->declare_parameter("focus_auto", -1);
		focus_auto = this->get_parameter("focus_auto").as_int();

		if (brightness != -1)
		{
			cap_.set(cv::CAP_PROP_BRIGHTNESS, brightness);
			RCLCPP_INFO(this->get_logger(), "Brightness: %d", brightness);
		}
		if (contrast != -1)
		{
			cap_.set(cv::CAP_PROP_CONTRAST, contrast);
			RCLCPP_INFO(this->get_logger(), "Contrast: %d", contrast);
		}
		if (saturation != -1)
		{
			cap_.set(cv::CAP_PROP_SATURATION, saturation);
			RCLCPP_INFO(this->get_logger(), "Saturation: %d", saturation);
		}
		if (sharpness != -1)
		{
			cap_.set(cv::CAP_PROP_SHARPNESS, sharpness);
			RCLCPP_INFO(this->get_logger(), "Sharpness: %d", sharpness);
		}
		if (backlight_compensation != -1)
		{
			cap_.set(cv::CAP_PROP_BACKLIGHT, backlight_compensation);
			RCLCPP_INFO(this->get_logger(), "Backlight compensation: %d", backlight_compensation);
		}
		if (exposure_auto != -1)
		{
			cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, exposure_auto);
			RCLCPP_INFO(this->get_logger(), "Exposure auto: %d", exposure_auto);
		}
		if (exposure != -1)
		{
			cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
			RCLCPP_INFO(this->get_logger(), "Exposure: %d", exposure);
		}
		if (focus_auto != -1)
		{
			cap_.set(cv::CAP_PROP_AUTOFOCUS, focus_auto);
			RCLCPP_INFO(this->get_logger(), "Focus auto: %d", focus_auto);
		}
		if (focus_absolute != -1)
		{
			cap_.set(cv::CAP_PROP_FOCUS, focus_absolute);
			RCLCPP_INFO(this->get_logger(), "Focus absolute: %d", focus_absolute);
		}
		RCLCPP_INFO(this->get_logger(), "Camera settings configured");
	}

	std::string generateFilename() {
		rclcpp::Time current_time = rclcpp::Time(this->now());
		std::stringstream ss;
		struct tm t;
		time_t raw_time = current_time.seconds();
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
		RCLCPP_INFO(this->get_logger(), "Filename: %s", filename.c_str());
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
	rclcpp::init(argc, argv);
	auto webcam = std::make_shared<webcam_node>();
	webcam->run();
	rclcpp::spin(webcam);
	rclcpp::shutdown();
	return 0;
}