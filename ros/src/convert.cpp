#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;
ros::Subscriber sub;

void load_radar(cv::Mat raw_data, std::vector<int64_t> &timestamps, std::vector<float> &azimuths,
    std::vector<bool> &valid, cv::Mat &fft_data) {
    int encoder_size = 5600;
    int N = raw_data.rows;
    timestamps = std::vector<int64_t>(N, 0);
    azimuths = std::vector<float>(N, 0);
    valid = std::vector<bool>(N, true);
    int range_bins = 3360;
    fft_data = cv::Mat::zeros(N, range_bins, CV_32F);
    for (int i = 0; i < N; ++i) {
        uchar* byteArray = raw_data.ptr<uchar>(i);
        timestamps[i] = *((int64_t *)(byteArray));
        azimuths[i] = *((uint16_t *)(byteArray + 8)) * 2 * M_PI / float(encoder_size);
        valid[i] = byteArray[10] == 255;
        for (int j = 0; j < range_bins; j++) {
            fft_data.at<float>(i, j) = (float)*(byteArray + 11 + j) / 255.0;
        }
    }
}

void radar_polar_to_cartesian(std::vector<float> &azimuths, cv::Mat &fft_data, float radar_resolution,
    float cart_resolution, int cart_pixel_width, bool interpolate_crossover, cv::Mat &cart_img) {

    float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
    if (cart_pixel_width % 2 == 0)
        cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;

    cv::Mat map_x = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
    cv::Mat map_y = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

#pragma omp parallel for collapse(2)
    for (int j = 0; j < map_y.cols; ++j) {
        for (int i = 0; i < map_y.rows; ++i) {
            map_y.at<float>(i, j) = -1 * cart_min_range + j * cart_resolution;
        }
    }
#pragma omp parallel for collapse(2)
    for (int i = 0; i < map_x.rows; ++i) {
        for (int j = 0; j < map_x.cols; ++j) {
            map_x.at<float>(i, j) = cart_min_range - i * cart_resolution;
        }
    }
    cv::Mat range = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
    cv::Mat angle = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

    float azimuth_step = azimuths[1] - azimuths[0];
#pragma omp parallel for collapse(2)
    for (int i = 0; i < range.rows; ++i) {
        for (int j = 0; j < range.cols; ++j) {
            float x = map_x.at<float>(i, j);
            float y = map_y.at<float>(i, j);
            float r = (sqrt(pow(x, 2) + pow(y, 2)) - radar_resolution / 2) / radar_resolution;
            if (r < 0)
                r = 0;
            range.at<float>(i, j) = r;
            float theta = atan2f(y, x);
            if (theta < 0)
                theta += 2 * M_PI;
            angle.at<float>(i, j) = (theta - azimuths[0]) / azimuth_step;
        }
    }
    if (interpolate_crossover) {
        cv::Mat a0 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
        cv::Mat aN_1 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
        for (int j = 0; j < fft_data.cols; ++j) {
            a0.at<float>(0, j) = fft_data.at<float>(0, j);
            aN_1.at<float>(0, j) = fft_data.at<float>(fft_data.rows-1, j);
        }
        cv::vconcat(aN_1, fft_data, fft_data);
        cv::vconcat(fft_data, a0, fft_data);
        angle = angle + 1;
    }
    cv::remap(fft_data, cart_img, range, angle, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

void callback(const sensor_msgs::ImageConstPtr & msg) {
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	cv::Mat raw_data = cv_ptr->image;
	// Extract fft_data and relevant meta data
	std::vector<int64_t> timestamps;
	std::vector<float> azimuths;
	std::vector<bool> valid;
	cv::Mat polar_img;
	load_radar(raw_data, timestamps, azimuths, valid, polar_img);
	cv::Mat cart_img;
    float cart_resolution = 0.25;
    int cart_pixel_width = 1000;
    bool interpolate_crossover = true;
    float radar_resolution = 0.0596;
    radar_polar_to_cartesian(azimuths, polar_img, radar_resolution, cart_resolution, cart_pixel_width,
        interpolate_crossover, cart_img);
	// Convert to cartesian
    cv::Mat vis;
    cart_img.convertTo(vis, CV_8U, 255.0);
	cv_bridge::CvImage out_msg;
	out_msg.encoding = "mono8";
	out_msg.image = vis;
	pub.publish(out_msg.toImageMsg());
}

int main(int32_t argc, char** argv) {
	ros::init(argc, argv, "convert");
	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::Image>("/Navtech/Cartesian", 4);
	sub = nh.subscribe("/talker1/Navtech/Polar", 2, &callback);
	ros::spin();
	return 0;
}