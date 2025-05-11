#include "config.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <opencv2/highgui.hpp>

using namespace cv;

int main() {
    // Set up environment for the yellow robot with your and enemy marker IDs
    setUpYellow(7, 14);

    // Start strategy in a separate thread
    std::atomic<bool> running;
    running = true;
    std::thread strategyThread([]() {
        strategy.start_test();
    });

    // Draw loop
    while (running) {
        {
            std::lock_guard<std::mutex> lock(strategy.visualiserMutex);
            visualiser.drawImage();
        }
        if (waitKey(10) == 27) { // ESC to break
            running = false;
            break;
        }
    }

    strategy.stopAsyncPositionUpdates(); // ensure clean stop
    strategyThread.join();
    return 0;
}
