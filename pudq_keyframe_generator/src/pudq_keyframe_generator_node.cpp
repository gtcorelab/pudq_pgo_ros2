#include "PUDQKeyframeGenerator.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PUDQKeyframeGenerator>());
    rclcpp::shutdown();
    return 0;
}
