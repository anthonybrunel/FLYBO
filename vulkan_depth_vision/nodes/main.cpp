#include <iostream>

#include "include/VoxelApp.hpp"

int main() {

    VoxelApp app;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}