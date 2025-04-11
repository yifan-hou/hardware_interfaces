#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>

// Key code for 'a' (use 'a' lowercase)
#define KEY_CODE_A 30

int main() {
  // You may need to change this path to match your keyboard's input device
  // Use 'ls -l /dev/input/by-path/ | grep kbd' to find the right keyboard device
  // Then sudo chmod 777 /dev/input/eventX to allow access
  const char* device = "/dev/input/event17";  // Change this as needed

  struct input_event ev;
  int fd;

  fd = open(device, O_RDONLY | O_NONBLOCK);
  if (fd == -1) {
    std::cerr << "Cannot open input device: " << device << std::endl;
    std::cerr << "Try running with sudo or modify the device path."
              << std::endl;
    return 1;
  }

  std::cout << "Monitoring 'a' key (press Ctrl+C to exit)...\n" << std::endl;

  // Set frame rate to 100Hz
  const int targetFPS = 100;
  const std::chrono::microseconds frameTime(10000);  // 10ms = 100Hz

  int keyValue = 0;  // 0 when 'a' is not pressed, 1 when it is
  bool running = true;

  while (running) {
    auto frameStart = std::chrono::high_resolution_clock::now();

    // Process input events
    while (read(fd, &ev, sizeof(ev)) > 0) {
      // Check only key events (type 1) for the 'a' key (code 30)
      if (ev.type == EV_KEY && ev.code == KEY_CODE_A) {
        if (ev.value == 1) {
          // Key down event
          keyValue = 1;
          std::cout << "Key DOWN: 'a' - value = " << keyValue << std::endl;
        } else if (ev.value == 0) {
          // Key up event
          keyValue = 0;
          std::cout << "Key UP: 'a' - value = " << keyValue << std::endl;
        }
      }
      //   // sleep a little
      //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Display current time and key state (optional)
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&now_time);
    time_str.resize(time_str.size() - 1);  // Remove trailing newline

    std::cout << "\rCurrent key value: " << keyValue << " | "
              << time_str.substr(11, 8) << std::flush;

    // Calculate time to sleep to maintain 100Hz
    auto frameEnd = std::chrono::high_resolution_clock::now();
    auto frameDuration = std::chrono::duration_cast<std::chrono::microseconds>(
        frameEnd - frameStart);

    if (frameDuration < frameTime) {
      std::this_thread::sleep_for(frameTime - frameDuration);
    }
  }

  // Close the input device
  close(fd);

  return 0;
}