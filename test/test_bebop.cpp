// Compilation
//
// g++ -o test_bebop test_bebop.cpp bebop.cpp -I $ARSDK_PATH/include -I. -L
// $ARSDK_PATH/lib -larsal -lardiscovery -larcontroller -larnetworkal
// -larcommands -ljson -lmux -lpomp -lulog -larstream -larstream2  -lrtsp -lsdp
// -larnetwork -larmedia -lfutils -larsal -I..

#include <memory>

#include "ros2_bebop_driver/bebop.hpp"

int main(int argc, char* argv[]) {
    std::string bebop_ip = "192.168.50.62";
    unsigned short port = 44444;
    auto bebop = std::make_shared<bebop_driver::Bebop>();
    bebop->connect(bebop_ip, port);
}
