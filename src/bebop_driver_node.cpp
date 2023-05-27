#include <cstdio>

#include "ros2_bebop_driver/bebop.hpp"

int main(int argc, char** argv) {
    printf("hello world ros2_bebop package\n");
    auto bebop = new bebop_driver::Bebop();
    return 0;
}
