#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <stdlib.h>

// GO2 sport FSM mode (from unitree_sdk2 example a2_sport_state.cpp FSM_STATE_STR)
enum FsmMode {
  FSM_PASSIVE       = 0,   // 躺/阻尼
  FSM_STAND_DOWN    = 1,   // 趴下
  FSM_STAND_UP      = 2,   // 站立中(过渡)
  FSM_DEFAULT_MODE  = 3,   // 默认站立(已站稳)
  FSM_RECOVERY      = 12   // 恢复站立中
};

using namespace unitree::common;

class EnsureStand {
public:
  EnsureStand(const std::string& iface) {
    // ChannelFactory::Init 必须先于 SportClient 构造
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    sport_.reset(new unitree::robot::go2::SportClient());
    sport_->SetTimeout(3.0f);
    sport_->Init();
    suber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate"));
    suber_->InitChannel(std::bind(&EnsureStand::onState, this, std::placeholders::_1), 1);
  }

  void onState(const void* msg) {
    unitree_go::msg::dds_::SportModeState_ s = *(unitree_go::msg::dds_::SportModeState_*)msg;
    std::lock_guard<std::mutex> lk(mu_);
    state_ = s; got_ = true;
  }

  uint8_t mode() const { std::lock_guard<std::mutex> lk(mu_); return state_.mode(); }
  bool hasState() const { std::lock_guard<std::mutex> lk(mu_); return got_; }
  double velNorm() const {
    std::lock_guard<std::mutex> lk(mu_);
    auto v = state_.velocity();
    return std::sqrt((double)v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  }
  uint32_t errorCode() const { std::lock_guard<std::mutex> lk(mu_); return state_.error_code(); }

  // 已站稳 = DEFAULT_MODE(3) 且 error_code==0 且 速度≈0
  bool isStandingStable(double vel_thr=0.05) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (!got_) return false;
    if (state_.error_code() != 0) return false;
    if (state_.mode() != FSM_DEFAULT_MODE) return false;
    auto v = state_.velocity();
    double vn = std::sqrt((double)v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    return vn < vel_thr;
  }

  // 等待到达"已站稳"并持续 settle 秒；返回是否成功
  bool waitStand(double timeout_s, double settle_s=1.2, double vel_thr=0.05) {
    using namespace std::chrono;
    auto t0 = steady_clock::now();
    double stableSince = -1.0;
    while (duration_cast<duration<double>>(steady_clock::now()-t0).count() < timeout_s) {
      if (isStandingStable(vel_thr)) {
        double el = duration_cast<duration<double>>(steady_clock::now()-t0).count();
        if (stableSince < 0) stableSince = el;
        if (el - stableSince >= settle_s) return true;
      } else {
        stableSince = -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  // 只用原始指针调用 RecoveryStand（不转移所有权）
  unitree::robot::go2::SportClient* sportRaw() { return sport_.get(); }

private:
  mutable std::mutex mu_;
  bool got_ = false;
  unitree_go::msg::dds_::SportModeState_ state_;
  std::unique_ptr<unitree::robot::go2::SportClient> sport_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> suber_;
};

int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth10";
  std::fprintf(stderr, "[ensure_stand] interface=%s\n", iface.c_str());
  EnsureStand es(iface);

  // 等第一个状态
  using namespace std::chrono;
  auto t0 = steady_clock::now();
  while (!es.hasState() && duration_cast<duration<double>>(steady_clock::now()-t0).count() < 6.0)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!es.hasState()) {
    std::fprintf(stderr, "[ERROR] no rt/sportmodestate received (Go2 up on %s?)\n", iface.c_str());
    return 1;
  }

  if (es.isStandingStable()) {
    std::printf("[OK] Go2 already standing (mode=%d)\n", (int)es.mode());
    return 0;
  }

  // 未站稳 -> 只调用一次 RecoveryStand()
  uint8_t m = es.mode();
  std::fprintf(stderr, "[ensure_stand] mode=%d not standing, calling RecoveryStand() once\n", (int)m);
  es.sportRaw()->RecoveryStand();

  // 等待真正站稳（最多 13s，稳定 1.2s）
  if (es.waitStand(13.0, 1.2)) {
    std::printf("[OK] Go2 standing confirmed (mode=%d)\n", (int)es.mode());
    return 0;
  }
  std::fprintf(stderr, "[ERROR] unable to confirm standing state (timeout)\n");
  return 1;
}
