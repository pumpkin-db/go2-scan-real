#include <cstdio>
#include <string>
#include <thread>
#include <chrono>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth10";
  unitree::robot::ChannelFactory::Instance()->Init(0, iface);
  unitree::robot::go2::SportClient sc;
  sc.SetTimeout(5.0f);
  sc.Init();
  fprintf(stderr, "[rec] calling RecoveryStand() once ...\n");
  int32_t ret = sc.RecoveryStand();
  fprintf(stderr, "[rec] RecoveryStand ret=%d\n", ret);
  for (int i = 0; i < 10; i++) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    fprintf(stderr, "[rec] t=%ds\n", i + 1);
  }
  fprintf(stderr, "[rec] done\n");
  return 0;
}
