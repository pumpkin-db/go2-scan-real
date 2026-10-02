#include <cmath>
#include <csignal>
#include <thread>
static volatile std::sig_atomic_t stop_requested = 0;
static void handleStopSignal(int) { stop_requested = 1; }
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

class CmdVelBridge {
public:
    CmdVelBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : has_cmd_(false)
    {
        std::string iface;
        pnh.param<std::string>("interface", iface, "eth10");
        pnh.param<double>("hz", hz_, 10.0);
        pnh.param<double>("cmd_timeout_s", cmd_timeout_s_, 0.5);
        pnh.param<bool>("dry_run", dry_run_, false);
        if (!dry_run_) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);
        fprintf(stderr, "[bridge] CycloneDDS init ok iface=%s\n", iface.c_str());

        sport_client_.reset(new unitree::robot::go2::SportClient());
        sport_client_->SetTimeout(3.0f);
        sport_client_->Init();
        fprintf(stderr, "[bridge] SportClient ready\n");

        } // dry_run never initializes DDS or calls hardware
        cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &CmdVelBridge::cmdVelCb, this);
        timer_ = nh.createWallTimer(ros::WallDuration(1.0 / hz_), &CmdVelBridge::timerCb, this);
        last_cmd_ = {};
        fprintf(stderr, "[bridge] up hz=%.1f direct SCAN passthrough; native avoidance untouched\n", hz_);
        fprintf(stderr, "[bridge-diag-v1] enabled: one-second stderr diagnostics; SDK rc=0 does not prove physical motion\n");
    }
    ~CmdVelBridge() {
        timer_.stop();
        for (int i=0;i<3;++i) {
            try { stop("SHUTDOWN_STOP", 0); } catch (...) { fprintf(stderr,"[bridge] shutdown StopMove threw\n"); }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg) {
        last_cmd_ = *msg; last_cmd_stamp_ = std::chrono::steady_clock::now(); has_cmd_ = true;
        ++rx_count_;
    }
    void timerCb(const ros::WallTimerEvent&) {
        if (!has_cmd_) {
            report("WAIT_CMD", 0.0, 0.0, 0.0, -1.0, false, 0, 0.0);
            return;
        }
        double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_stamp_).count();
        if (age > cmd_timeout_s_) {
            stop("STALE_STOP", age);
            return;
        }
        if (!std::isfinite(last_cmd_.linear.x) || !std::isfinite(last_cmd_.linear.y) || !std::isfinite(last_cmd_.angular.z)) { stop("INVALID_STOP", age); return; }
        double vx = last_cmd_.linear.x, vy = last_cmd_.linear.y, vyaw = last_cmd_.angular.z;
        if (vx == 0.0 && vy == 0.0 && vyaw == 0.0) {
            stop("ZERO_STOP", age); return;
        }
        const auto begin = std::chrono::steady_clock::now();
        const int32_t rc = dry_run_ ? 0 : sport_client_->Move(vx, vy, vyaw);
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        ++move_count_;
        if (rc != 0) ++error_count_;
        report("MOVE", vx, vy, vyaw, age, true, rc, elapsed_ms);
    }
private:
    void stop(const char* reason, double age) {
        const auto begin = std::chrono::steady_clock::now();
        if (!dry_run_) sport_client_->SetTimeout(0.3f);
        const int32_t rc = dry_run_ ? 0 : sport_client_->StopMove();
        fprintf(stderr, "[bridge-stop] %s dry_run=%d rc=%d\n", reason, int(dry_run_), int(rc));
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        ++stop_count_;
        if (rc != 0) ++error_count_;
        report(reason, 0.0, 0.0, 0.0, age, true, rc, elapsed_ms);
    }
    void report(const char* state, double vx, double vy, double yaw, double age,
                bool called, int32_t rc, double elapsed_ms) {
        const auto now = std::chrono::steady_clock::now();
        const bool first_error = called && rc != 0 && !reported_error_;
        if (!first_error && reported_ &&
            std::chrono::duration<double>(now - last_report_).count() < 1.0) return;
        reported_ = true;
        if (called && rc != 0) reported_error_ = true;
        last_report_ = now;
        fprintf(stderr, "[bridge-diag-v1] state=%s rx=%llu publishers=%u "
                "input=(%.3f,%.3f,%.3f) sent=(%.3f,%.3f,%.3f) age_s=%.3f "
                "sdk_called=%d rc=%d sdk_ms=%.2f moves=%llu stops=%llu errors=%llu\n",
                state, (unsigned long long)rx_count_, cmd_sub_.getNumPublishers(),
                last_cmd_.linear.x, last_cmd_.linear.y, last_cmd_.angular.z,
                vx, vy, yaw, age, int(called), int(rc), elapsed_ms,
                (unsigned long long)move_count_, (unsigned long long)stop_count_,
                (unsigned long long)error_count_);
    }
    uint64_t rx_count_ = 0, move_count_ = 0, stop_count_ = 0, error_count_ = 0;
    bool reported_ = false, reported_error_ = false;
    std::chrono::steady_clock::time_point last_report_;
    ros::Subscriber cmd_sub_;
    ros::WallTimer timer_;
    std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
    geometry_msgs::Twist last_cmd_;
    std::chrono::steady_clock::time_point last_cmd_stamp_;
    bool dry_run_ = false;
    bool has_cmd_;
    double hz_, cmd_timeout_s_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_bridge", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, handleStopSignal);
    std::signal(SIGTERM, handleStopSignal);
    ros::NodeHandle nh, pnh("~");
    CmdVelBridge bridge(nh, pnh);
    ros::WallRate rate(100);
    while (ros::ok() && !stop_requested) { ros::spinOnce(); rate.sleep(); }
    return 0;
}
