#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;

class CameraParam
{
public:
    // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Mat CameraMatrix;
    //  distortion matrix
    cv::Mat Distorsion;
    // size of the image
    cv::Size CamSize;

    // 3x1 matrix (Tx, Ty, Tz), usually 0 for non-stereo cameras or stereo left cameras
    cv::Mat ExtrinsicMatrix;

    /**Empty constructor
     */
    CameraParam()
    {
        CameraMatrix = cv::Mat();
        Distorsion = cv::Mat();
        ExtrinsicMatrix = cv::Mat();
        CamSize = cv::Size(-1, -1);
    }

    /**Sets the parameters
     * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
     * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
     * @param size image size
     */
    CameraParam(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size)
    {
        setParams(cameraMatrix, distorsionCoeff, size);
    }

    void setParams(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size)
    {
        cv::Mat auxCamMatrix;
        ExtrinsicMatrix = cv::Mat::zeros(1, 3, CV_64FC1);
        if (cameraMatrix.rows == 3 && cameraMatrix.cols == 4)
        {
            ExtrinsicMatrix.at<double>(0, 0) = cameraMatrix.at<double>(0, 3);
            ExtrinsicMatrix.at<double>(0, 1) = cameraMatrix.at<double>(1, 3);
            ExtrinsicMatrix.at<double>(0, 2) = cameraMatrix.at<double>(2, 3);

            // Change size to 3x3
            auxCamMatrix = cameraMatrix(cv::Rect(0, 0, 3, 3)).clone();
            cameraMatrix = auxCamMatrix;
        }

        if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3)
            throw cv::Exception(9000, "invalid input cameraMatrix", "CameraParameters::setParams",
                                __FILE__, __LINE__);
        cameraMatrix.convertTo(CameraMatrix, CV_32FC1);
        if (distorsionCoeff.total() < 4 || distorsionCoeff.total() >= 7)
            throw cv::Exception(9000, "invalid input distorsionCoeff",
                                "CameraParameters::setParams", __FILE__, __LINE__);
        cv::Mat auxD;

        distorsionCoeff.convertTo(Distorsion, CV_32FC1);
        CamSize = size;
    }

    CameraParam(const CameraParam &CI)
    {

        CI.CameraMatrix.copyTo(CameraMatrix);
        CI.Distorsion.copyTo(Distorsion);
        CI.ExtrinsicMatrix.copyTo(ExtrinsicMatrix);
        CamSize = CI.CamSize;
    }

    /**Indicates whether this object is valid
     */
    bool isValid() const
    {
        return CameraMatrix.rows != 0 && CameraMatrix.cols != 0 && Distorsion.rows != 0 &&
               Distorsion.cols != 0 && CamSize.width != -1 && CamSize.height != -1;
    }

    CameraParam &operator=(const CameraParam &CI)
    {
        CI.CameraMatrix.copyTo(CameraMatrix);
        CI.Distorsion.copyTo(Distorsion);
        CI.ExtrinsicMatrix.copyTo(ExtrinsicMatrix);
        CamSize = CI.CamSize;
        return *this;
    }

    void clear()
    {
        CameraMatrix = cv::Mat();
        CamSize = cv::Size(-1, -1);
        Distorsion = cv::Mat();
    }
};

CameraParam CameraInfo2CamParams(const sensor_msgs::CameraInfo &cam_info)
{
    cv::Mat cameraMatrix(3, 4, CV_64FC1, 0.0);
    cv::Mat distorsionCoeff(4, 1, CV_64FC1);
    cv::Size size(cam_info.width, cam_info.height);

    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = cam_info.P[0];
    cameraMatrix.at<double>(0, 1) = cam_info.P[1];
    cameraMatrix.at<double>(0, 2) = cam_info.P[2];
    cameraMatrix.at<double>(0, 3) = cam_info.P[3];
    cameraMatrix.at<double>(1, 0) = cam_info.P[4];
    cameraMatrix.at<double>(1, 1) = cam_info.P[5];
    cameraMatrix.at<double>(1, 2) = cam_info.P[6];
    cameraMatrix.at<double>(1, 3) = cam_info.P[7];
    cameraMatrix.at<double>(2, 0) = cam_info.P[8];
    cameraMatrix.at<double>(2, 1) = cam_info.P[9];
    cameraMatrix.at<double>(2, 2) = cam_info.P[10];
    cameraMatrix.at<double>(2, 3) = cam_info.P[11];
    // for(int i=0;i<3;i++){
    //     for(int j=0;j<3;j++){
    //         std::cout<<cameraMatrix.at<double>(i,j)<<" ";
    //     }
    // }

    if (cam_info.D.size() == 5)
    {
        // k1, k2, t1, t2, k3
        for (int i = 0; i < 5; ++i)
            distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
    }
    else
    {
        ROS_WARN("length of camera_info D vector is not 5, assuming zero distortion...");
        for (int i = 0; i < 5; ++i)
            distorsionCoeff.at<double>(i, 0) = 0;
    }

    return CameraParam(cameraMatrix, distorsionCoeff, size);
}

class ChessboardSample
{
public:
    ChessboardSample() : n_("~")
    {
        subimg_ = n_.subscribe("/image", 1, &ChessboardSample::subrgbCallback, this);
        cam_info_sub_ = n_.subscribe("/camera_info", 1, &ChessboardSample::cam_info_callback, this);

        pose_pub = n_.advertise<geometry_msgs::PoseStamped>("pose", 100);
        transform_pub = n_.advertise<geometry_msgs::TransformStamped>("transform", 100);
        position_pub = n_.advertise<geometry_msgs::Vector3Stamped>("position", 100);
        marker_pub = n_.advertise<visualization_msgs::Marker>("marker", 10);
        pixel_pub = n_.advertise<geometry_msgs::PointStamped>("pixel", 10);

        pubimg_ = n_.advertise<sensor_msgs::Image>("chessboard", 1);

        n_.param<float>("marker_size", markerSize, 0.025);
        n_.param<int>("marker_width", markerWidth, 8);
        n_.param<int>("marker_height", markerHeight, 11);

        n_.param<string>("reference_frame", reference_frame, "");
        n_.param<string>("camera_frame", camera_frame, "");
        n_.param<string>("marker_frame", marker_frame, "");

        ROS_ASSERT(camera_frame != "" && marker_frame != "");

        if (reference_frame.empty())
            reference_frame = camera_frame;

        ROS_INFO("chessboard node started with markers of %d x %d and size %f m", markerWidth, markerHeight, markerSize);
        ROS_INFO("chessboard node will publish pose to TF with %s as parent and %s as child.", reference_frame.c_str(),
                 marker_frame.c_str());
    }

    bool getTransform(const std::string &refFrame, const std::string &childFrame, tf::StampedTransform &transform)
    {
        std::string errMsg;

        if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01),
                                          &errMsg))
        {
            ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
            return false;
        }
        else
        {
            try
            {
                _tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), // get latest available
                                            transform);
            }
            catch (const tf::TransformException &e)
            {
                ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
                return false;
            }
        }
        return true;
    }

    void subrgbCallback(const sensor_msgs::ImageConstPtr &color_msg)
    {
        color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        color_pic = color_ptr->image;

        // 识别棋盘格
        Size board_size = Size(markerWidth, markerHeight);
        vector<Point2f> corners;
        Point2f center;
        // 计算corners中所有点的中心
        for (int i = 0; i < corners.size(); i++)
        {
            center.x += corners[i].x;
            center.y += corners[i].y;
        }
        center.x = center.x / corners.size();
        center.y = center.y / corners.size();

        bool patternfound = findChessboardCorners(color_pic, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        static tf::TransformBroadcaster br;
        if (patternfound)
        {
            cv::Mat image_gray;
            cv::cvtColor(color_pic, image_gray, cv::COLOR_BGR2GRAY);
            cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01);
            cornerSubPix(image_gray, corners, Size(11, 11), Size(-1, -1), criteria);
            // for (int i = 0; i < corners.size(); i++)
            // {
            //     cout << corners[i].x << " " << corners[i].y << endl;
            // }
            ros::Time curr_stamp = color_msg->header.stamp;
            drawChessboardCorners(color_pic, board_size, corners, patternfound);
            cv_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_pic).toImageMsg();
            // 计算棋盘格的位姿
            vector<Point3f> obj;
            for (int j = 0; j < markerHeight; j++)
            {
                for (int i = 0; i < markerWidth; i++)
                {
                    obj.push_back(Point3f(i * markerSize, j * markerSize, 0));
                }
            }
            Mat rvec, tvec;
            solvePnP(obj, corners, camParam.CameraMatrix, camParam.Distorsion, rvec, tvec);
            Mat R;
            Rodrigues(rvec, R);

            Eigen::Matrix3d R_eigen;
            Eigen::Vector3d t_eigen;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R_eigen(i, j) = R.at<double>(i, j);
                }
                t_eigen(i) = tvec.at<double>(i);
            }
            // 根据R_eigen和t_eigen计算tf::Transform
            tf::Matrix3x3 R_tf;
            tf::Vector3 t_tf;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R_tf[i][j] = R_eigen(i, j);
                }
                t_tf[i] = t_eigen(i);
                // cout<<t_tf[i]<<endl;
            }
            tf::Transform transform(R_tf, t_tf);
            tf::StampedTransform cameraToReference;
            if (reference_frame != camera_frame)
            {
                getTransform(reference_frame, camera_frame, cameraToReference);
                transform = static_cast<tf::Transform>(cameraToReference) * transform;
            }
            tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame, marker_frame);
            br.sendTransform(stampedTransform);

            geometry_msgs::PoseStamped poseMsg;
            tf::poseTFToMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = curr_stamp;
            pose_pub.publish(poseMsg);

            geometry_msgs::TransformStamped transformMsg;
            tf::transformStampedTFToMsg(stampedTransform, transformMsg);
            transform_pub.publish(transformMsg);

            geometry_msgs::Vector3Stamped positionMsg;
            positionMsg.header = transformMsg.header;
            positionMsg.vector = transformMsg.transform.translation;
            position_pub.publish(positionMsg);

            geometry_msgs::PointStamped pixelMsg;
            pixelMsg.header = transformMsg.header;
            pixelMsg.point.x = center.x;
            pixelMsg.point.y = center.y;
            pixelMsg.point.z = 0;
            pixel_pub.publish(pixelMsg);

            // publish rviz marker representing the ArUco marker patch
            visualization_msgs::Marker visMarker;
            visMarker.header = transformMsg.header;
            visMarker.id = 1;
            visMarker.type = visualization_msgs::Marker::CUBE;
            visMarker.action = visualization_msgs::Marker::ADD;
            visMarker.pose = poseMsg.pose;
            visMarker.scale.x = markerSize * markerWidth;
            visMarker.scale.y = markerSize * markerHeight;
            visMarker.scale.z = 0.001;
            visMarker.color.r = 1.0;
            visMarker.color.g = 0;
            visMarker.color.b = 0;
            visMarker.color.a = 1.0;
            visMarker.lifetime = ros::Duration(3.0);
            marker_pub.publish(visMarker);
        }
        else
        {
            cv_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_pic).toImageMsg();
        }
        pubimg_.publish(cv_msg);
    }

    void cam_info_callback(const sensor_msgs::CameraInfo &msg)
    {
        camParam = CameraInfo2CamParams(msg);
        cam_info_received = true;
        cam_info_sub_.shutdown();
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber subimg_;
    ros::Subscriber cam_info_sub_;

    ros::Publisher pubimg_;
    ros::Publisher pose_pub;
    ros::Publisher transform_pub;
    ros::Publisher position_pub;
    ros::Publisher marker_pub; // rviz visualization marker
    ros::Publisher pixel_pub;

    CameraParam camParam;

    cv_bridge::CvImagePtr color_ptr;
    Mat color_pic;
    sensor_msgs::ImagePtr cv_msg;

    string marker_frame;
    string camera_frame;
    string reference_frame;

    float markerSize;
    int markerWidth;
    int markerHeight;

    bool cam_info_received;

    tf::TransformListener _tfListener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chessboard_simple");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ChessboardSample simple;

    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}