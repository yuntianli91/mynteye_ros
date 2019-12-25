// mynteye
#include <mynteye/api/api.h>
#include <mynteye/logger.h>
// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
// opencv
#include <opencv2/opencv.hpp>
// C++ standard
#include <stdio.h>
#include <mutex>
#include <memory>
#include <atomic>
// namespace
using namespace std;
MYNTEYE_USE_NAMESPACE
// ros publisher
image_transport::Publisher cam0_pub, cam1_pub;
ros::Publisher imu0_pub;
// others
ros::Time imu_start_time;
ros::Time img_start_time_left, img_start_time_right;
// =========================== functions ==================================== //
bool setParameters(const shared_ptr<API> &mynt_api, ros::NodeHandle &nh);
// =========================== main function ================================ //
int main(int argc, char** argv){
    // ---- ros node init
    ros::init(argc, argv, "mynteye_node");
    ros::NodeHandle nh("~");
    
    image_transport::ImageTransport it(nh);
    cam0_pub = it.advertise("/mynteye_node/cam0/image_raw", 100);
    cam1_pub = it.advertise("/mynteye_node/cam1/image_raw", 100);

    imu0_pub = nh.advertise<sensor_msgs::Imu>("/mynteye_node/imu0/data_raw", 1000);
    
    // ---- MYNTEYE API init
    auto &&mynt_api = API::Create(argc, argv);
    if(!mynt_api){
        ROS_ERROR("Fail to create MYNTEYE API ! Node terminated.");
        return -1;
    }
    
    // ---- set parameters ---- //
    setParameters(mynt_api, nh);
    // ---- config sensor callback functions ---- //
    // Get left image 
    std::atomic_uint left_count(0);
    mynt_api->SetStreamCallback(Stream::LEFT, [&left_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        if(left_count == 0){img_start_time_left = ros::Time::now() - ros::Duration(data.img->timestamp / 1e6);}
        
        std_msgs::Header img_header;
        sensor_msgs::ImagePtr img_msg;
        img_header.frame_id = "cam0";
        img_header.seq = left_count++;
        img_header.stamp = img_start_time_left + ros::Duration(data.img->timestamp / 1e6);

        img_msg = cv_bridge::CvImage(img_header, "mono8", data.frame).toImageMsg();
        // cv::imshow("left_img", data.frame);
        // cv::waitKey(1);
        cam0_pub.publish(img_msg);
    });
    // Get right image 
    std::atomic_uint right_count(0);
    mynt_api->SetStreamCallback(Stream::RIGHT, [&right_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        if(right_count == 0){img_start_time_right = ros::Time::now() - ros::Duration(data.img->timestamp / 1e6);}
        
        std_msgs::Header img_header;
        sensor_msgs::ImagePtr img_msg;
        img_header.frame_id = "cam1";
        img_header.seq = right_count++;
        img_header.stamp = img_start_time_right + ros::Duration(data.img->timestamp / 1e6);

        img_msg = cv_bridge::CvImage(img_header, "mono8", data.frame).toImageMsg();
        cam1_pub.publish(img_msg);
    });
    // Get motion data from callback
    std::atomic_uint imu_count(0);
    mynt_api->SetMotionCallback([&imu_count](const api::MotionData &data){
        CHECK_NOTNULL(data.imu);
        
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "imu0";
        imu_msg.header.seq = imu_count++;
        imu_msg.header.stamp = ros::Time(data.imu->timestamp / 1e6); // us -> s

        imu_msg.linear_acceleration.x = data.imu->accel[0];
        imu_msg.linear_acceleration.y = data.imu->accel[1];
        imu_msg.linear_acceleration.z = data.imu->accel[2];
        
        imu_msg.angular_velocity.x = data.imu->gyro[0];
        imu_msg.angular_velocity.y = data.imu->gyro[1];
        imu_msg.angular_velocity.z = data.imu->gyro[2];

        imu0_pub.publish(imu_msg);
      });

    // ---- start data stream ---- //
    mynt_api->Start(Source::ALL);
    while(true){
      // q or Q to quit
      system("stty echo");
      char key = getchar();
      if(key == 'q' || key == 'Q'){break;}
      
      ros::spinOnce();
    }
    mynt_api->Stop(Source::ALL);
    return 0;
}

bool setParameters(const shared_ptr<API> &mynt_api, ros::NodeHandle &nh){
  // ========== set exposure parameters ===================== //
  int manual_exposure_flag = 0;
  nh.getParam("manual_exposure/enable", manual_exposure_flag);
  if(manual_exposure_flag){
    int m_gain, m_brightness, m_contrast;
    nh.getParam("manual_exposure/gain", m_gain);
    nh.getParam("manual_exposure/brightness", m_brightness);
    nh.getParam("manual_exposure/contrast", m_contrast);

    mynt_api->SetOptionValue(Option::EXPOSURE_MODE, 1);
    mynt_api->SetOptionValue(Option::GAIN, m_gain);
    mynt_api->SetOptionValue(Option::BRIGHTNESS, m_brightness);
    mynt_api->SetOptionValue(Option::CONTRAST, m_contrast);
  
    LOG(INFO) << "Set CAMERA_EXPOSURE_MODE to MANUAL";
    LOG(INFO) << "Set GAIN to " << m_gain;
    LOG(INFO) << "Set BRIGHTNESS to " << m_brightness;
    LOG(INFO) << "set CONTRAST to " << m_contrast;
  }
  else{
    int a_gain, a_time, a_brightness;
    nh.getParam("auto_exposure/max_gain", a_gain);
    nh.getParam("auto_exposure/max_exposure_time", a_time);
    nh.getParam("auto_exposure/desired_brightness", a_brightness);
  
    mynt_api->SetOptionValue(Option::EXPOSURE_MODE, 0);
    mynt_api->SetOptionValue(Option::MAX_GAIN, a_gain);
    mynt_api->SetOptionValue(Option::MAX_EXPOSURE_TIME, a_time);
    mynt_api->SetOptionValue(Option::DESIRED_BRIGHTNESS, a_brightness);
   
    LOG(INFO) << "Set CAMERA_EXPOSURE_MODE to AUTO";
    LOG(INFO) << "Set MAX_GAIN to " << a_gain;
    LOG(INFO) << "set MAX_EXPOSURE_TIME to " << a_time;
    LOG(INFO) << "Set DESIRED_BRIGHTNESS to " << a_brightness;
  }
  // ================ set other parameters ================================== //
  int frame_rate, imu_frequency;
  nh.getParam("frame_rate", frame_rate);
  nh.getParam("imu_frequency", imu_frequency);

  mynt_api->SetOptionValue(Option::FRAME_RATE, 20);
  mynt_api->SetOptionValue(Option::IMU_FREQUENCY, 200);
  LOG(INFO) << "Set FRAME_RATE to "
            << mynt_api->GetOptionValue(Option::FRAME_RATE);
  LOG(INFO) << "Set IMU_FREQUENCY to "
            << mynt_api->GetOptionValue(Option::IMU_FREQUENCY);
  
 return true;
}