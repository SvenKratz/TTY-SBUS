#include "SBUS.h"
#include <chrono>
#include <thread>

SBUS::SBUS x8r("/dev/ttyUSB0");

int main () {
    x8r.begin();

    uint8_t failSafe;
    uint16_t lostFrame;

    while (true)
    {
        int i = 0;
        while (i < 16)
        {
            uint16_t channels[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            bool r = x8r.read(&channels[i], &failSafe, &lostFrame);
            
            // std::cout << i << " r  " << r << " " << channels[i] << " L " << lostFrame << " , ";
            if (r)
            {
                // std::cout << "PKT ==> ";
                // for (int k = 0; k < 16; k++)
                // {
                //     std::cout << std::dec << k << " : " << channels[k] << " | ";
                // }
                // std::cout << std::endl;

            }

            ++i;
        }
        // std::cout << std::endl << "====" << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

