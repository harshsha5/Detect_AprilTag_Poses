#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void setTagLocations(float x_det, float y_det, float z_det)
  {
	  //TODO: Update tag locations
    // Optimize this using eigen for matrix multiplication and save the subscribed P matrix from the Camera info topic only once 
    float D11 = 656.4037475585938;
    float D12 = 0.0;
    float D13 = 329.7264464396903;
    float D14 = 0.0;
    float D21 = 0.0;
    float D22 = 655.7791748046875;
    float D23 = 261.0592521875151;
    float D24 = 0.0;
    float D31 = 0.0;
    float D32 = 0.0;
    float D33 = 1.0;
    float D34 = 0.0;
    x_loc = (D11*x_det + D12*y_det + D13*z_det + D14)/(D31*x_det + D32*y_det + D33*z_det + D34);
    y_loc = (D21*x_det + D22*y_det + D23*z_det + D24)/(D31*x_det + D32*y_det + D33*z_det + D34);
    x_arr.push_back(x_loc);
    y_arr.push_back(y_loc);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	//TODO: Draw circles at tag locations on image. 
  cv::circle(cv_ptr->image, cv::Point(x_loc, y_loc), 5, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // TODO:Output modified video stream
    // Convert the modified frames into sensor_msgs::Image message and publish it using image_pub
    image_pub_.publish(cv_ptr->toImageMsg());
  }

private:
  float x_loc ,y_loc;
  std::vector<float> x_arr;
  std::vector<float> y_arr;
};
