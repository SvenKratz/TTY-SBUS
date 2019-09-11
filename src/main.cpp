#include "SBUS.h"
#include <chrono>
#include <thread>

SBUS::SBUS x8r("/dev/ttyUSB0");

int main () {
    x8r.begin();

    uint16_t channels[16];
    uint8_t failSafe;
    uint16_t lostFrame;

    while (true)
    {
        int i = 0;
        while (i < 16)
        {
            bool r = x8r.read(&channels[i], &failSafe, &lostFrame);
            
            // std::cout << i << " r  " << r << " " << channels[i] << " L " << lostFrame << " , ";
            
            ++i;
        }
        // std::cout << std::endl << "====" << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

