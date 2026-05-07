#include <exception>
#include <iostream>

#include "pick/app.hpp"

int main(int argc, char** argv)
{
    try {
        return catcheye::pick::run_app(argc, argv);
    } catch (const std::exception& exception) {
        std::cerr << "startup failed: " << exception.what() << '\n';
        return 1;
    }
}
