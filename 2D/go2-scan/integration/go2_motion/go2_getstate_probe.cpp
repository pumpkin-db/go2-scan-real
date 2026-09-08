#include <cstdio>
#include <string>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

// Client::Call 是 protected；用子类暴露一个公开入口，仅用于只读 GetState(1034) 查询。
class SportClientState : public unitree::robot::go2::SportClient {
public:
  int32_t CallState(const std::string& param, std::string& data) { return Call(1034, param, data); }
};

int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth10";
  unitree::robot::ChannelFactory::Instance()->Init(0, iface);
  SportClientState sc;
  sc.SetTimeout(5.0f);
  sc.Init();
  fprintf(stderr, "[gs] client_api=%s server_api=%s\n",
          sc.GetApiVersion().c_str(), sc.GetServerApiVersion().c_str());

  std::string data1;
  int32_t ret1 = sc.CallState("", data1);
  fprintf(stderr, "[gs] GetState(no-param) ret=%d\n", ret1);
  fprintf(stderr, "[gs] data(no-param)=%s\n", data1.c_str());

  std::string data2;
  std::string param2 = "{\"keys\":[\"state\",\"bodyHeight\",\"footRaiseHeight\",\"speedLevel\",\"gait\"]}";
  int32_t ret2 = sc.CallState(param2, data2);
  fprintf(stderr, "[gs] GetState(keys...) ret=%d\n", ret2);
  fprintf(stderr, "[gs] data(keys)=%s\n", data2.c_str());

  return 0;
}
