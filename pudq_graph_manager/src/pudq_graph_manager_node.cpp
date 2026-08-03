#include "PUDQGraphManager.hpp"

int main(int argc, char ** argv) {
    std::cout << std::fixed << std::setprecision(6);

    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<PUDQGraphManager>());

    std::make_shared<PUDQGraphManager>();

    rclcpp::shutdown();
    return 0;
}
