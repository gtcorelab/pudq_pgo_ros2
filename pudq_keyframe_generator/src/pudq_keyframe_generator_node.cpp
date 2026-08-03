#include "PUDQKeyframeGenerator.hpp"

int main(int argc, char ** argv) {
    // Set precision
    std::cout << std::fixed << std::setprecision(6);

    // Enable parallelization
    int num_cpu = sysconf(_SC_NPROCESSORS_ONLN);
    omp_set_num_threads(num_cpu);
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PUDQKeyframeGenerator>());
    rclcpp::shutdown();
    return 0;
}
