
// coinft_bus_interface.cpp

#include "CoinFTBus.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Configure the sensors.
    std::vector<std::pair<int, std::string>> sensorConfigs = {
        { CoinFTBus::LEFT,  "src/UFT3_MLP_4L_scl_1_30.onnx" },
        { CoinFTBus::RIGHT, "src/UFT4_MLP_4L_scl_1_30.onnx" }
    };

    try {
        // Create CoinFTBus object (Handles signals internally)
        CoinFTBus bus("/dev/tty.usbmodem101", 115200, sensorConfigs);

        std::cout << "CoinFTBus initialized and streaming." << std::endl;

        // Main loop: print every 50 samples.
        int sampleCounter = 0;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sampleCounter++;

            if (sampleCounter % 50 == 0) {
                std::vector<double> dataLeft = bus.getLatestData(CoinFTBus::LEFT);
                std::vector<double> dataRight = bus.getLatestData(CoinFTBus::RIGHT);

                std::cout << "Sample " << sampleCounter << ":\n  LEFT sensor FT:  ";
                for (double v : dataLeft) std::cout << v << " ";
                std::cout << "\n  RIGHT sensor FT: ";
                for (double v : dataRight) std::cout << v << " ";
                std::cout << "\n---------------------\n";
            }
        }
    }
    catch (const std::exception &e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}





/*
#include "CoinFTBus.h"   
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <cstdlib>

// Global pointer to the sensor bus object.
CoinFTBus* g_bus = nullptr;

// Signal handler: called on SIGINT or SIGTERM.
void signal_handler(int signum) {
    std::cout << "\nSignal (" << signum << ") received, shutting down sensors gracefully..." << std::endl;
    if (g_bus) {
        g_bus->stopStreaming();  // Stop the streaming thread.
        g_bus->idle();           // Send an IDLE command.
    }
    std::exit(signum);  // Terminate the process.
}

int main() {
    // Install signal handlers.
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    // Optionally, you can install a SIGTSTP handler if you want to catch Ctrl+Z:
    std::signal(SIGTSTP, signal_handler);
    
    // Configure the sensors. Typically, LEFT is 8 and RIGHT is 9.
    std::vector<std::pair<int, std::string>> sensorConfigs = {
        { CoinFTBus::LEFT,  "src/UFT3_MLP_4L_scl_1_30.onnx" },
        { CoinFTBus::RIGHT, "src/UFT4_MLP_4L_scl_1_30.onnx" }
    };

    try {
        // Create a CoinFTBus object.
        // The constructor automatically opens the port, initializes the bus,
        // starts streaming, and waits until all sensors are tared.
        CoinFTBus bus("/dev/tty.usbmodem101", 115200, sensorConfigs);
        g_bus = &bus;  // Set the global pointer for use in the signal handler.

        std::cout << "CoinFTBus initialized and streaming." << std::endl;

        // Main loop: print every 50 samples.
        int sampleCounter = 0;
        while (true) {
            // Give the sensor thread a chance to update.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sampleCounter++;

            if (sampleCounter % 50 == 0) {
                // Retrieve and print the latest FT data from both sensors.
                std::vector<double> dataLeft = bus.getLatestData(CoinFTBus::LEFT);
                std::vector<double> dataRight = bus.getLatestData(CoinFTBus::RIGHT);

                std::cout << "Sample " << sampleCounter << ":" << std::endl;
                std::cout << "  LEFT sensor FT:  ";
                for (double v : dataLeft)
                    std::cout << v << " ";
                std::cout << std::endl;

                std::cout << "  RIGHT sensor FT: ";
                for (double v : dataRight)
                    std::cout << v << " ";
                std::cout << std::endl << "---------------------" << std::endl;
            }
        }
    }
    catch (const std::exception &e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
*/



/*
// main.cpp
#include "CoinFTBus.h"   
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Configure the sensors. Typically LEFT is 8 and RIGHT is 9.
    // If you only have one sensor, you can supply just one configuration.
    std::vector<std::pair<int, std::string>> sensorConfigs = {
        { CoinFTBus::LEFT,  "src/UFT3_MLP_4L_scl_1_30.onnx" },
        { CoinFTBus::RIGHT, "src/UFT4_MLP_4L_scl_1_30.onnx" }
    };

    try {
        // Create a CoinFTBus object.
        // The constructor automatically opens the port, initializes the bus,
        // starts streaming, and waits until all sensors are tared.
        CoinFTBus bus("/dev/tty.usbmodem101", 115200, sensorConfigs);

        std::cout << "CoinFTBus initialized and streaming." << std::endl;

        // Print every 50 samples. (Here we simulate counting samples in our main loop.)
        int sampleCounter = 0;
        while (true) {
            // Give the sensor thread a chance to update (adjust sleep as needed).
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sampleCounter++;

            if (sampleCounter % 50 == 0) {
                // Retrieve and print the latest FT data from both sensors.
                std::vector<double> dataLeft = bus.getLatestData(CoinFTBus::LEFT);
                std::vector<double> dataRight = bus.getLatestData(CoinFTBus::RIGHT);

                std::cout << "Sample " << sampleCounter << ":" << std::endl;
                std::cout << "  LEFT sensor FT:  ";
                for (double v : dataLeft)
                    std::cout << v << " ";
                std::cout << std::endl;

                std::cout << "  RIGHT sensor FT: ";
                for (double v : dataRight)
                    std::cout << v << " ";
                std::cout << std::endl << "---------------------" << std::endl;
            }
        }
    }
    catch (const std::exception &e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

*/
