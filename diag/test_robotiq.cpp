#include "../src/robotiq.hpp"
#include <iostream>
#include <functional>
void require(bool good,const char* message) { if(!good) throw std::runtime_error(message); }
void until(const std::function<bool()>& condition) {
  auto end=robotiq::Clock::now()+std::chrono::seconds(3);
  while(!condition()) {
    if(robotiq::Clock::now()>end) throw std::runtime_error("test timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
int main(int argc,char** argv) {
  try {
    require(argc==3,"port and scenario required");
    std::string mode=argv[2];
    if(mode=="init") {
      robotiq::Port port(argv[1]); require(robotiq::initialize(port).ready(),"activation did not finish");
    } else {
      robotiq::Controller driver(argv[1],9,32,0);
      driver.connect(); driver.start();
      std::this_thread::sleep_for(std::chrono::milliseconds(160));
      require(driver.healthy(),"initial read failed");
      require(driver.position()==128,"startup moved the gripper");
      driver.request(255);
      until([&]{return driver.position()>160;});
      driver.pause();
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      int held=driver.position();
      std::this_thread::sleep_for(std::chrono::milliseconds(130));
      require(driver.position()==held,"release did not stop motion");
      driver.request(0);
      if(mode=="normal") { until([&]{return driver.position()<held-20;}); driver.stop(); require(driver.healthy(),"stop failed"); }
      else { until([&]{return !driver.healthy();}); require(!driver.failure().empty(),"fault reason missing");driver.stop(); }
    }
    std::cout << "ROBOTIQ_TEST_OK " << mode << '\n';return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
