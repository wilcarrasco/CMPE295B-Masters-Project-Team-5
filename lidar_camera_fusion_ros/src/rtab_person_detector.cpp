#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/LoopClosure.h>
#include <rtabmap_ros/LoopClosureStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Transform.h>

using namespace cv;
using namespace std;

class PersonTracker
{
public:
    PersonTracker() : it_(nh_)
    {
        // Initialize ROS node handle and image transport
        nh_ = ros::NodeHandle("~");
        it_ = image_transport::ImageTransport(nh_);

        // Subscribe to RGB image topic
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &PersonTracker::imageCb, this);
        depth_sub = nh_.subscribe("depth_registered/image_raw", 1); // needs a callback
        camera_info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &PersonTracker::cameraInfoCallback, this);

        // Initialize RTAB-Map loop closure publisher
        loop_closure_pub_ = nh_.advertise<rtabmap_ros::LoopClosureStamped>("/rtabmap/loop_closure", 1);

        // Load YOLO model for person detection
        net_ = cv::dnn::readNetFromDarknet("../../darknet_ros/darknet_ros/cfg/yolov3-tiny.cfg", "../../darknet_ros/darknet_ros/weights/yolov3-tiny.weights");
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // Advertise markers for detected persons
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("person_markers", 1);
    }

    ~PersonTracker()
    {
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher loop_closure_pub_;

    cv::dnn::Net net_;

    void imageCB(const sensor_msgs::ImageConstPtr& rgb_msg,
                        const sensor_msgs::ImageConstPtr& depth_msg,
                        const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr, cv_depth;;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Convert depth image to float
        cv::Mat depth_f;
        cv_depth->image.convertTo(depth_f, CV_32F, 0.001);

        // Get camera model from camera info
        rtabmap::CameraModel model = rtabmap_ros::cameraModelFromROS(*info_msg);

        // Detect people in the image using YOLO
        cv::Mat blob = cv::dnn::blobFromImage(cv_ptr->image, 1/255.0, cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
        net_.setInput(blob);
        cv::Mat detectionMat = net_.forward();

        vector<Rect> people;
        for (int i = 0; i < detectionMat.rows; i++)
        {
            float confidence = detectionMat.at<float>(i, 5);
            if (confidence > 0.5)
            {
                int x = detectionMat.at<float>(i, 0) * cv_ptr->image.cols;
                int y = detectionMat.at<float>(i, 1) * cv_ptr->image.rows;
                int width = detectionMat.at<float>(i, 2) * cv_ptr->image.cols;
                int height = detectionMat.at<float>(i, 3) * cv_ptr->image.rows;
                Rect person(x - width/2, y - height/2, width, height);
                people.push_back(person);
            }
        }


        // Loop through detected people and add them to RTAB-Map
        for (int i = 0; i < people.size(); i++)
        {
            // Calculate person's position in camera frame
            int x = people[i].x + people[i].width / 2;
            int y = people[i].y + people[i].height / 2;

            // Convert pixel coordinates to 3D point in camera frame
            float fx = 570.34; // focal length of Orbbec Astra camera
            float fy = 570.34;
            float cx = 314.5;
            float cy = 235.5;
            float z = cv_depth->image.at<Vec3f>(y, x)[2];
            float x3d = (x - cx) * z / fx;
            float y3d = (y - cy) * z / fy;

            // Add person's position to RTAB-Map
            rtabmap_ros::msg::KeyPointMsg keypoint;
            keypoint.pt.x = x3d;
            keypoint.pt.y = y3d;
            keypoint.pt.z = z;
            keypoint.response_matches = 1;

            rtabmap_ros::LoopClosure loopClosure;
            loopClosure.words.push_back(i + 1);
            loopClosure.refDescriptors.push_back(0);
            loopClosure.queryDescriptors.push_back(0);
            loopClosure.keyPoints.push_back(keypoint);
            loopClosure_pub_.publish(loopClosure.toROSMsg());
        }
    }
}


//         // Get camera model from camera info
//         rtabmap::CameraModel model = rtabmap_ros::cameraModelFromROS(*info_msg);

//         // Convert RGB image to signature
//         rtabmap::Signature signature(cv_rgb->image, cv::Mat(), cv_depth->image, model.fx(), model.fy(), model.cx(), model.cy(), model.localTransform());

//         // Add persons as user data
//         std::vector<rtabmap::Transform> positions;
//         for (size_t i = 0; i < persons.size(); i++)
//         {
//             // Compute the centroid of the person
//             cv::Point3f centroid = cv::Point3f(0, 0, 0);
//             int count = 0;
//             for (int y = persons[i].y; y < persons[i].y + persons[i].height; y++)
//             {
//                 for (int x = persons[i].x; x < persons[i].x + persons[i].width; x++)
//                 {
//                     float d = depth_f.at<float>(y, x);
//                     if (d > 0 && d < 4.5) // 4.5 meters max detection range
//                     {
//                         cv::Point3f pt = model.projectPixelTo3dRay(cv::Point2f(x,y))*d;
//                         centroid += pt;
//                         count++;
//                     }
//                 }
//             }
//             if (count > 0)
//             {
//             centroid *= (1.0/count);
//             positions.push_back(rtabmap::Transform(centroid.x, centroid.y, centroid.z, 0, 0, 0));
//             }
//         }
//         signature.sensorData().setUserData(std::make_sharedrtabmap::UserData(positions));

//         // Publish RGB image with persons marked
//         cv::Mat img_marked = cv_rgb->image.clone();
//         for (size_t i = 0; i < persons.size(); i++)
//         {
//             cv::rectangle(img_marked, persons[i], cv::Scalar(0, 0, 255), 2);
//         }
//         sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_marked).toImageMsg();
//         pub_debug_image_.publish(msg);

//         // Publish the signature
//         rtabmap_ros::msg::UserDataPtr msgUserData = std::make_shared<rtabmap_ros::msg::UserData>();
//         rtabmap_ros::userDataToROS(*signature.sensorData().userData(), *msgUserData);
//         pub_user_data_.publish(msgUserData);

//         // Publish the updated RGBD image to RTAB-Map
//         rtabmap_ros::msg::RGBDImage rgbd_msg;
//         rgbd_msg.header = cv_depth->header;
//         rgbd_msg.rgb = *cv_rgb;
//         rgbd_msg.depth = *cv_depth;
//         rgbd_msg.depth_bigendian = depth_bigendian;
//         pub_rgbd_image_.publish(rgbd_msg);
//     }
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_tracker");

    PersonTracker person_tracker;

    ros::spin();

    return 0;
}
// Note that you will also need to modify the code to get the depth value for the center point of each detected person using the Orbbec Astra camera. 
// You can do this by subscribing to the `/camera/depth/image_raw` topic and using the `cv_bridge` library to convert the depth image message to an OpenCV image. 
// Then you can use the `cv::Mat::at()` method to get the depth value at the center point.


