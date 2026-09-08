#include <cstdio>
#include <string>
#include <thread>
#include <chrono>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

// 正式工具: 无条件调用一次 RecoveryStand；ret==0 后等待 5s 让起身完成；ret!=0 退出非0。
// 不判断 mode/body_height；不关避障；不发 Move。
int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth10";
  unitree::robot::ChannelFactory::Instance()->Init(0, iface);
  unitree::robot::go2::SportClient sc;
  sc.SetTimeout(5.0f);
  sc.Init();
  fprintf(stderr, "[standup] calling RecoveryStand() once ...\n");
  int32_t ret = sc.RecoveryStand();
  fprintf(stderr, "[standup] RecoveryStand ret=%d\n", ret);
  if (ret != 0) { fprintf(stderr, "[standup] FATAL: RecoveryStand ret!=0\n"); return 1; }
  fprintf(stderr, "[standup] ok, waiting ~5s for stand-up to finish ...\n");
  for (int i = 0; i < 5; i++) { std::this_thread::sleep_for(std::chrono::seconds(1)); fprintf(stderr, "[standup] t=%ds\n", i + 1); }
  fprintf(stderr, "[standup] done\n");
  return 0;
}
