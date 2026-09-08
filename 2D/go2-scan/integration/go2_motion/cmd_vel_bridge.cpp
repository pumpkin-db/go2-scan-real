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
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

class CmdVelBridge {
public:
    CmdVelBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : has_cmd_(false), cmd_age_ok_(false)
    {
        std::string iface;
        pnh.param<std::string>("interface", iface, "eth10");
        pnh.param<double>("hz", hz_, 10.0);
        pnh.param<double>("cmd_timeout_s", cmd_timeout_s_, 0.5);
        pnh.param<double>("threshold_lin", threshold_lin_, 0.05);
        pnh.param<double>("threshold_ang", threshold_ang_, 0.05);
        // 第一轮安全默认：不自动站立、不关闭原生避障
        pnh.param<bool>("auto_stand", auto_stand_, false);
        pnh.param<double>("stand_settle_s", stand_settle_s_, 2.0);
        pnh.param<bool>("disable_avoid", disable_avoid_, false);
        // 实体速度硬限制（默认 0.5 m/s / 0.5 rad/s，后续可 _max_linear_speed:=0.75）
        pnh.param<double>("max_linear_speed", max_linear_speed_, 0.5);
        pnh.param<double>("max_angular_speed", max_angular_speed_, 0.5);

        pnh.param<bool>("dry_run", dry_run_, false);
        if (!dry_run_) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);
        fprintf(stderr, "[bridge] CycloneDDS init ok iface=%s\n", iface.c_str());

        sport_client_.reset(new unitree::robot::go2::SportClient());
        sport_client_->SetTimeout(3.0f);
        sport_client_->Init();
        fprintf(stderr, "[bridge] SportClient ready\n");

        avoid_client_.reset(new unitree::robot::go2::ObstaclesAvoidClient());
        avoid_client_->Init();
        fprintf(stderr, "[bridge] ObstaclesAvoidClient ready\n");

        ros::Duration(1.0).sleep();

        if (auto_stand_) {
            fprintf(stderr, "[bridge] auto_stand: RecoveryStand()\n");
            sport_client_->RecoveryStand();
            ros::Duration(stand_settle_s_).sleep();
            fprintf(stderr, "[bridge] stand settle done\n");
        }

        // 仅当 disable_avoid_=true 才调用 SwitchSet(false) 关闭原生避障；默认 false 不调用，保持避障开启。
        if (disable_avoid_) {
            int32_t r = avoid_client_->SwitchSet(false);
            bool got = true;
            avoid_client_->SwitchGet(got);
            fprintf(stderr, "[bridge] disable_avoid: SwitchSet(false)=%d SwitchGet=%d (0=off good)\n", (int)r, (int)got);
        } else {
            fprintf(stderr, "[bridge] disable_avoid=false -> 不改变原生避障状态；实际开关状态未查询\n");
        }

        } // dry_run never initializes DDS or calls hardware
        cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &CmdVelBridge::cmdVelCb, this);
        timer_ = nh.createWallTimer(ros::WallDuration(1.0 / hz_), &CmdVelBridge::timerCb, this);
        last_cmd_ = {};
        fprintf(stderr, "[bridge] up hz=%.1f auto_stand=%d disable_avoid=%d max_lin=%.3f max_ang=%.3f\n",
                hz_, (int)auto_stand_, (int)disable_avoid_, max_linear_speed_, max_angular_speed_);
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
            stop("STALE_STOP", age); cmd_age_ok_ = false;
            return;
        }
        cmd_age_ok_ = true;
        if (!std::isfinite(last_cmd_.linear.x) || !std::isfinite(last_cmd_.linear.y) || !std::isfinite(last_cmd_.angular.z)) { stop("INVALID_STOP", age); return; }
        double vx = last_cmd_.linear.x, vy = last_cmd_.linear.y, vyaw = last_cmd_.angular.z;
        if (std::abs(vx)<threshold_lin_ && std::abs(vy)<threshold_lin_ && std::abs(vyaw)<threshold_ang_) {
            stop("ZERO_STOP", age); return;
        }
        // 平面速度模长限幅：sqrt(vx^2+vy^2)<=max_linear_speed_，超则按比例同时缩放 vx/vy（保持运动方向）
        double lin = std::sqrt(vx * vx + vy * vy);
        if (lin > max_linear_speed_ && lin > 1e-6) {
            double s = max_linear_speed_ / lin;
            vx *= s; vy *= s;
        }
        // angular.z 单独 clamp
        if (vyaw > max_angular_speed_) vyaw = max_angular_speed_;
        else if (vyaw < -max_angular_speed_) vyaw = -max_angular_speed_;
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
    std::unique_ptr<unitree::robot::go2::ObstaclesAvoidClient> avoid_client_;
    geometry_msgs::Twist last_cmd_;
    std::chrono::steady_clock::time_point last_cmd_stamp_;
    bool dry_run_ = false;
    bool has_cmd_, cmd_age_ok_, auto_stand_, disable_avoid_;
    double hz_, cmd_timeout_s_, threshold_lin_, threshold_ang_, stand_settle_s_;
    double max_linear_speed_, max_angular_speed_;
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
