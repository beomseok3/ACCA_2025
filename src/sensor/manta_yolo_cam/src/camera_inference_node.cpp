/*──────────────────────────────────────────────────────────
 *  Manta YOLO Cam – ROS 2 Node  (Image only)
 *──────────────────────────────────────────────────────────*/
#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/* ─── Vimba X SDK ─────────────────────────────────────── */
#include <VmbCPP/VmbSystem.h>
#include <VmbCPP/IFrameObserver.h>
#include <VmbCPP/Frame.h>
using namespace VmbCPP;

/* ─── YOLO & utilities (검출은 내부 시각화에만 사용) ─── */
#include "det/YOLO8.hpp"
#include "tools/BoundedThreadSafeQueue.hpp"

/*────────────────────────────────────────────────────────*/
class MantaYoloNode : public rclcpp::Node, public IFrameObserver
{
public:
    MantaYoloNode(const std::string& model,
                  const std::string& names,
                  bool gui)
    : Node("manta_yolo_cam"),
      IFrameObserver(nullptr),
      rawQ_(64), show_gui_(gui),
      detector_(model, names, /*GPU=*/true)
    {
        img_pub_ = image_transport::create_publisher(this, "image_raw");

        /* Vimba init */
        VmbSystem& vimba = VmbSystem::GetInstance();
        if (vimba.Startup() != VmbErrorSuccess) throw std::runtime_error("Vimba Startup fail");

        CameraPtrVector cams;  vimba.GetCameras(cams);
        if (cams.empty()) throw std::runtime_error("No camera");
        cam_ = cams[0];
        if (cam_->Open(VmbAccessModeFull) != VmbErrorSuccess) throw std::runtime_error("Open cam");

        if (FeaturePtr feat; cam_->GetFeatureByName("AcquisitionMode", feat) == VmbErrorSuccess)
            feat->SetValue("Continuous");

        constexpr VmbUint32_t NBUF = 8;
        cam_->StartContinuousImageAcquisition(NBUF, IFrameObserverPtr(this));

        cons_thread_ = std::thread(&MantaYoloNode::consume_loop, this);

        if (show_gui_) {
            cv::namedWindow("manta yolo");
            gui_thread_ = std::thread([this]{
                while (rclcpp::ok()) {
                    cv::Mat f; { std::lock_guard<std::mutex> l(gui_mtx_); f = gui_img_.clone(); }
                    if (!f.empty()) cv::imshow("manta yolo", f);
                    if (cv::waitKey(1)==27) rclcpp::shutdown();
                }
            });
        }
    }
    ~MantaYoloNode() override
    {
        stop_ = true;
        if (cons_thread_.joinable()) cons_thread_.join();
        if (gui_thread_.joinable())  gui_thread_.join();
        cam_->StopContinuousImageAcquisition();
        cam_->Close();
        VmbSystem::GetInstance().Shutdown();
    }

    /* Vimba 콜백 */
    void FrameReceived(FramePtr p) override {
        if (stop_) return;
        VmbFrameStatusType s; if (p->GetReceiveStatus(s)!=VmbErrorSuccess||s!=VmbFrameStatusComplete){cam_->QueueFrame(p);return;}
        VmbUchar_t* b=nullptr; if (p->GetImage(b)!=VmbErrorSuccess||!b){cam_->QueueFrame(p);return;}
        VmbUint32_t w=0,h=0; p->GetWidth(w); p->GetHeight(h);
        cv::Mat raw(h,w,CV_8UC1,b), bgr; cv::cvtColor(raw,bgr,cv::COLOR_BayerBG2BGR);
        if(!rawQ_.enqueue(bgr.clone())){cv::Mat dump;rawQ_.dequeue(dump);rawQ_.enqueue(bgr.clone());}
        cam_->QueueFrame(p);
    }

private:
    void consume_loop()
    {
        cv::Mat f;
        while (!stop_ && rawQ_.dequeue(f)) {
            auto dets = detector_.detect(f);          // 검출은 화면용
            detector_.drawBoundingBoxMask(f, dets);

            std_msgs::msg::Header hdr; hdr.stamp=this->now(); hdr.frame_id="camera";
            auto msg = cv_bridge::CvImage(hdr,"bgr8",f).toImageMsg();
            img_pub_.publish(*msg);

            if (show_gui_) { std::lock_guard<std::mutex> l(gui_mtx_); gui_img_ = f.clone(); }
        }
    }

    /* members */
    CameraPtr cam_;
    BoundedThreadSafeQueue<cv::Mat> rawQ_;
    YOLO8Detector detector_;
    image_transport::Publisher img_pub_;

    std::thread cons_thread_;
    bool show_gui_; cv::Mat gui_img_; std::mutex gui_mtx_; std::thread gui_thread_;
    std::atomic<bool> stop_{false};
};

/*────────────────────────────────────────────────────────*/
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        bool gui = true;
        if (argc >= 4 && std::string(argv[3])=="--no-gui") gui=false;
        auto node = std::make_shared<MantaYoloNode>(argv[1], argv[2], gui);
        rclcpp::spin(node);
    } catch(const std::exception& e) { std::cerr<<"Fatal: "<<e.what()<<"\n"; }
    rclcpp::shutdown();
    return 0;
}
