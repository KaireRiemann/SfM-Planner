#include <ros/ros.h>
#include <sfm_planner/CaptureRequest.h>
#include <sfm_planner/CaptureResult.h>
#include <sfm_planner/FaceDetectionRequest.h>
#include <sfm_planner/FaceObservation.h>

#include <string>

namespace {

class InspectionMockNode {
public:
    explicit InspectionMockNode(ros::NodeHandle &nh) {
        nh.param<std::string>("face_request_topic", face_request_topic_,
                              "/inspection/face/request");
        nh.param<std::string>("face_result_topic", face_result_topic_,
                              "/inspection/face/result");
        nh.param<std::string>("capture_request_topic", capture_request_topic_,
                              "/inspection/capture/request");
        nh.param<std::string>("capture_result_topic", capture_result_topic_,
                              "/inspection/capture/result");
        nh.param("face_center_x", face_center_x_, 32.41);
        nh.param("face_center_y", face_center_y_, -1.28);
        nh.param("face_center_z", face_center_z_, 2.36);
        nh.param("face_normal_x", face_normal_x_, -0.998);
        nh.param("face_normal_y", face_normal_y_, 0.041);
        nh.param("face_normal_z", face_normal_z_, 0.027);

        face_pub_ = nh.advertise<sfm_planner::FaceObservation>(face_result_topic_, 1);
        capture_pub_ = nh.advertise<sfm_planner::CaptureResult>(capture_result_topic_, 10);
        face_sub_ = nh.subscribe(face_request_topic_, 1,
                                 &InspectionMockNode::onFaceRequest, this);
        capture_sub_ = nh.subscribe(capture_request_topic_, 10,
                                    &InspectionMockNode::onCaptureRequest, this);
        ROS_INFO("inspection_mock_node ready");
    }

private:
    void onFaceRequest(const sfm_planner::FaceDetectionRequestConstPtr &req) {
        sfm_planner::FaceObservation msg;
        msg.header.stamp = ros::Time::now();
        msg.mission_id = req->mission_id;
        msg.target_version = req->target_version;
        msg.request_id = req->request_id;
        msg.valid = true;
        msg.center.x = face_center_x_;
        msg.center.y = face_center_y_;
        msg.center.z = face_center_z_;
        msg.normal.x = face_normal_x_;
        msg.normal.y = face_normal_y_;
        msg.normal.z = face_normal_z_;
        msg.width = 8.0;
        msg.height = 5.0;
        msg.area = 40.0;
        msg.confidence = 0.92;
        face_pub_.publish(msg);
        ROS_INFO("Published mock FaceObservation");
    }

    void onCaptureRequest(const sfm_planner::CaptureRequestConstPtr &req) {
        sfm_planner::CaptureResult msg;
        msg.header.stamp = ros::Time::now();
        msg.mission_id = req->mission_id;
        msg.target_version = req->target_version;
        msg.request_id = req->request_id;
        msg.viewpoint_id = req->viewpoint_id;
        msg.success = true;
        msg.image_id = "mock_image_" + std::to_string(req->viewpoint_id);
        msg.reason = "ok";
        capture_pub_.publish(msg);
        ROS_INFO("Published mock CaptureResult for viewpoint %u", req->viewpoint_id);
    }

    std::string face_request_topic_;
    std::string face_result_topic_;
    std::string capture_request_topic_;
    std::string capture_result_topic_;
    double face_center_x_{0.0};
    double face_center_y_{0.0};
    double face_center_z_{0.0};
    double face_normal_x_{-1.0};
    double face_normal_y_{0.0};
    double face_normal_z_{0.0};
    ros::Publisher face_pub_;
    ros::Publisher capture_pub_;
    ros::Subscriber face_sub_;
    ros::Subscriber capture_sub_;
};

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "inspection_mock_node");
    ros::NodeHandle nh("~");
    InspectionMockNode node(nh);
    ros::spin();
    return 0;
}
