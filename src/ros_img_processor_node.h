#ifndef ros_img_processor_node_H
#define ros_img_processor_node_H

//std C++
#include <iostream>
#include <sstream>

//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

/** \brief Simple Image Processor
 *
 * Simple Image Processor with opencv calls
 *
 */
class RosImgProcessorNode
{
    protected:
        //ros node handle
        ros::NodeHandle nh_;

        //image transport
        image_transport::ImageTransport img_tp_;

        // subscribers to the image and camera info topics
        image_transport::Subscriber image_subs_;
        ros::Subscriber camera_info_subs_;

        //publishers
        image_transport::Publisher image_pub_;
	//ros::Publisher ray_direction_circle = nh_.advertise<std_msgs::String>("ray_direction_String", 1000); //Publisher of rays found
	//ros::Publisher ray_direction_circle_X = nh_.advertise<std_msgs::Float64>("ray_direction_X", 1000); //Publisher of rays 
	//ros::Publisher ray_direction_circle_Y = nh_.advertise<std_msgs::Float64>("ray_direction_Y", 1000); //Publisher of rays 
	ros::Publisher ray_direction_circle = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1); //Publisher of rays found, pose version

        //pointer to received (in) and published (out) images
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        cv_bridge::CvImage cv_img_out_;

		//Camera matrix
		cv::Mat matrixP_;
		cv::Mat matrixK_; //Needed to do the exercise
		cv::Mat matrixKinverted_;
    		cv::Mat rayDirection = (cv::Mat_<double>(3,1) << 0, 0, 0) ; //Calculated

        //image encoding label
        std::string img_encoding_;

        //wished process rate, [hz]
        double rate_;

    protected:
        // callbacks
        void imageCallback(const sensor_msgs::ImageConstPtr& _msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfo & _msg);

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */
        RosImgProcessorNode();

        /** \brief Destructor
        *
        * Destructor
        *
        */
        ~RosImgProcessorNode();

        /** \brief Process input image
        *
        * Process input image
        *
        **/
        void process();

        /** \brief Publish output image
        *
        * Publish output image
        *
        */
        void publish();

        /** \brief Returns rate_
         *
         * Returns rate_
         *
         **/
        double getRate() const;
};
#endif
