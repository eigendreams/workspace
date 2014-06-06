#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  HOGDescriptor hog;

public:
  ImageConverter() : it_(nh_) {
    image_pub_ = it_.advertise("salida_deteccion", 1);
    image_sub_ = it_.subscribe("/usb_cam1/image_raw", 1, &ImageConverter::imageCb, this);
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  }

  ~ImageConverter() { }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // pyrDown(cv_ptr->image,cv_ptr->image);
    vector<Rect> found, found_filt;
    hog.detectMultiScale(cv_ptr->image, found, 0, Size(8, 8), Size(32, 32), 1.05, 2);
    
    size_t i, j;
    for( i = 0; i < found.size(); i++ ) {
        Rect r = found[i];
        for( j = 0; j < found.size(); j++ )
            if( j != i && (r & found[j]) == r)
                break;
        if( j == found.size() )
            found_filt.push_back(r);
    }
    for( i = 0; i < found_filt.size(); i++ ) {
        Rect r = found_filt[i];
        r.x += cvRound(r.width * 0.1);
        r.width = cvRound(r.width * 0.8);
        r.y += cvRound(r.height * 0.07);
        r.height = cvRound(r.height * 0.8);
        rectangle(cv_ptr->image, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);
    }
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
