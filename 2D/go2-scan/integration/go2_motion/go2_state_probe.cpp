#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using namespace unitree::common;

class Probe {
public:
  Probe(const std::string& iface) {
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    sport_.reset(new unitree::robot::go2::SportClient());
    sport_->SetTimeout(3.0f);
    sport_->Init();
    suber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate"));
    suber_->InitChannel(std::bind(&Probe::onState, this, std::placeholders::_1), 1);
  }
  void onState(const void* msg) {
    auto s = *(unitree_go::msg::dds_::SportModeState_*)msg;
    ++n_;
    fprintf(stderr, "[cb] #%u mode=%u gait=%u bodyh=%.3f pos=[%.2f,%.2f,%.2f] err=%u vel=[%.3f,%.3f,%.3f]\n",
            (unsigned)n_, (unsigned)s.mode(), (unsigned)s.gait_type(),
            s.body_height(), s.position()[0], s.position()[1], s.position()[2],
            (unsigned)s.error_code(), s.velocity()[0], s.velocity()[1], s.velocity()[2]);
  }
  unitree::robot::go2::SportClient* sport() { return sport_.get(); }
private:
  unsigned n_ = 0;
  std::unique_ptr<unitree::robot::go2::SportClient> sport_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> suber_;
};

int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth10";
  Probe p(iface);
  fprintf(stderr, "[probe] subscribed, printing 3s (standing/idle state) ...\n");
  for (int i = 0; i < 3; i++) { std::this_thread::sleep_for(std::chrono::seconds(1)); fprintf(stderr, "[probe] t=%ds\n", i + 1); }
  fprintf(stderr, "[probe] >>> calling RecoveryStand() once\n");
  p.sport()->RecoveryStand();
  for (int i = 0; i < 12; i++) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    fprintf(stderr, "[probe] t=%ds\n", i + 4);
  }
  fprintf(stderr, "[probe] done\n");
  return 0;
}
