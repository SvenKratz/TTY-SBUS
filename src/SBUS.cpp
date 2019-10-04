/*
SBUS.cpp
Brian R Taylor
brian.taylor@bolderflight.com
Copyright (c) 2016 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <inttypes.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#include "SBUS.h"


using namespace std;

namespace SBUS
{

SBUS::SBUS(string tty)
{
    _tty = tty;

    for(int i=0;i<_numChannels;i++)
        _useWriteCoeff[i] = false;
}

SBUS::~SBUS()
{
    close(_fd);

    if (_readCoeff) {
        for (uint8_t i = 0; i < _numChannels; i++) {
            if (_readCoeff[i]) {
                delete[] _readCoeff[i];
            }
        }
        delete[] _readCoeff;
    }
    if (_writeCoeff) {
        for (uint8_t i = 0; i < _numChannels; i++) {
            if (_writeCoeff[i]) {
                delete[] _writeCoeff[i];
            }
        }
        delete[] _writeCoeff;
    }
}

/* starts the serial communication */
int SBUS::begin()
{
    // initialize parsing state
    _parserState = 0;
    // initialize default scale factors and biases
    for (uint8_t i = 0; i < _numChannels; i++)
    {
        setEndPoints(i, _defaultMin, _defaultMax);
    }

    // begin the serial port for SBUS
    #if defined(__MK20DX128__) || defined(__MK20DX256__)  // Teensy 3.0 || Teensy 3.1/3.2
        _bus->begin(_sbusBaud,SERIAL_8E1_RXINV_TXINV);
        SERIALPORT = _bus;
    #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)  // Teensy 3.5 || Teensy 3.6 || Teensy LC
        _bus->begin(_sbusBaud,SERIAL_8E2_RXINV_TXINV);
    #elif defined(STM32L496xx) || defined(STM32L476xx) || defined(STM32L433xx) || defined(STM32L432xx)  // STM32L4
        _bus->begin(_sbusBaud,SERIAL_SBUS);
    #elif defined(_BOARD_MAPLE_MINI_H_) // Maple Mini
        _bus->begin(_sbusBaud,SERIAL_8E2);
    #elif defined(__RASPBERRYPI_ZERO_W__)
        // open file descriptor
        _fd = open(_tty.c_str(), O_RDWR| O_NOCTTY);

        if(_fd < 0 )
        {
            cerr << "Error " << errno << " opening " << _tty << ": " << strerror(errno) << endl;
            return -1;
        }

        // set custom baudrate
        struct termios2 tio;
        int r = ioctl(_fd, TCGETS2, &tio);
        if(r)
        {
            cerr << "TCGETS2" << endl;
            return -1;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_cflag |= CSTOPB; // 2 stop bits
        tio.c_cflag |= PARENB; // enable parity bit, even by default
        tio.c_ispeed = tio.c_ospeed = _sbusBaud;

        r = ioctl(_fd, TCSETS2, &tio);
        if(r)
        {
            cerr << "TCSETS2" << endl;
            return -1;
        }
    #endif

    return 0;
}

/* read the SBUS data and calibrate it to +/- 1 */
bool SBUS::readCal(float* calChannels, uint8_t* failsafe, uint16_t* lostFrame)
{
    uint16_t channels[_numChannels];
    // read the SBUS data
    if(read(&channels[0],failsafe,lostFrame)) {
        // linear calibration
        for (uint8_t i = 0; i < _numChannels; i++) {
            calChannels[i] = channels[i] * _sbusScale[i] + _sbusBias[i];
            if (_useReadCoeff[i]) {
                calChannels[i] = PolyVal(_readLen[i],_readCoeff[i],calChannels[i]);
            }
        }
        // return true on receiving a full packet
        return true;
  } else {
        // return false if a full packet is not received
        return false;
  }
}

/* read the SBUS data */
bool SBUS::read(uint16_t* channels, uint8_t* failsafe, uint16_t* lostFrame)
{

    // parse the SBUS packet
    if (parse()) {
        // std::cout << "Got legit packet, decoding" << std::endl;

        // For a good parsing solution for Futaba-SBUS, see:
        // https://os.mbed.com/users/Digixx/code/SBUS-Library_16channel/file/83e415034198/FutabaSBUS/FutabaSBUS.cpp/

        uint8_t sbus_data[25] = {
  0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};

        int channel_idx = 0;
        int payload_idx = 0;
        int bit_in_sbus = 0;
        int bit_in_channel = 0;

        if (channels)
        {
            // TODO: use actual legit byte count we get from rx.
            for (int i = 0; i < 176; i++)
            {
                if (_payload[payload_idx] & (1 << bit_in_sbus))
                {
                    channels[channel_idx] |= (1 << bit_in_channel);
                    // cout << "1";
                }
                else
                {
                    // cout << "0";
                }
                
                bit_in_channel++;
                bit_in_sbus++;
                if (bit_in_sbus == 8)
                {
                    bit_in_sbus = 0;
                    payload_idx++;
                    // cout << "-";
                }
  
                if (bit_in_channel == 11)
                {
                    if (i > 0){
                        // std::cout << channel_idx << ":" << channels[channel_idx] << "\t";
                        cout << channel_idx <<" : " << channels[channel_idx] << " ";
                    }
                    bit_in_channel = 0;
                    channel_idx++;
              }
            }
        }

        std::cout << std::endl;
        return true;
    } else {
        // return false if a full packet is not received
        return false;
    }
}

/* parse the SBUS data */
bool SBUS::parse()
{
       
        // reset the parser state if too much time has passed
        // static int elapsedMicros, _sbusTime = 0;
        // if (_sbusTime > SBUS_TIMEOUT_US) {_parserState = 0;}
        // see if serial data is available

        const int kMaxBytes = _payloadSize;

        char buffer[kMaxBytes];


        int bytes = bytesAvalaible();
        _curByte = 0;
        _prevByte = 0;
        char _prevPrevByte = 0;
        char _curByte = 0;
        char _prevByte = 0;

        int bufIdx = 0;

        bool parse = false;

        bzero(_payload, sizeof(_payload));

        while (bytes > 0) {
            // std::cout << "PARSE bytes " << bytes << std::endl;
            char byte[1];
            ::read(_fd, byte, 1 );
            _curByte = byte[0];
            // std::cout << std::hex <<  (int)  _curByte << ',';
            // find the header
            if (_curByte == 0x0f && !parse)
            {
                parse = true;
                continue;
            }
            // footer
            else if (parse && _curByte == 0 && _prevByte == 0 && bufIdx >= 17)
            {
                std::cout << std::dec <<  bufIdx << "> ";
                memcpy(_payload, buffer, sizeof(buffer));
                return true;
            }
            else if (parse && bufIdx < kMaxBytes) {
                buffer[bufIdx++] = _curByte;
                // std::cout << std::hex << (int) _curByte << ",";
            }
            else if (bufIdx >= kMaxBytes)
            {
                // std::cout << "No end" << std::endl;
                return false;
            }
            _prevByte = _curByte;
            _prevPrevByte = _prevByte;
        }
        // return false if a partial packet
        // std::cout << "No more bytes" << std::endl;
        return false;
}

/* write SBUS packets */
void SBUS::write(uint16_t* channels)
{
    static uint8_t packet[25];
    /* assemble the SBUS packet */
    // SBUS header
    packet[0] = _sbusHeader;
    // 16 channels of 11 bit data
    if (channels)
    {
        packet[1] = (uint8_t) ((channels[0] & 0x07FF));
        packet[2] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
        packet[3] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
        packet[4] = (uint8_t) ((channels[2] & 0x07FF)>>2);
        packet[5] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
        packet[6] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
        packet[7] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
        packet[8] = (uint8_t) ((channels[5] & 0x07FF)>>1);
        packet[9] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
        packet[10] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
        packet[11] = (uint8_t) ((channels[7] & 0x07FF)>>3);
        packet[12] = (uint8_t) ((channels[8] & 0x07FF));
        packet[13] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
        packet[14] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);
        packet[15] = (uint8_t) ((channels[10] & 0x07FF)>>2);
        packet[16] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
        packet[17] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
        packet[18] = (uint8_t) ((channels[12] & 0x07FF)>>4 | (channels[13] & 0x07FF)<<7);
        packet[19] = (uint8_t) ((channels[13] & 0x07FF)>>1);
        packet[20] = (uint8_t) ((channels[13] & 0x07FF)>>9 | (channels[14] & 0x07FF)<<2);
        packet[21] = (uint8_t) ((channels[14] & 0x07FF)>>6 | (channels[15] & 0x07FF)<<5);
        packet[22] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    }
    // flags
    packet[23] = 0x00;
    // footer
    packet[24] = _sbusFooter;
    #if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 || Teensy 3.1/3.2
        // use ISR to send byte at a time,
        // 130 us between bytes to emulate 2 stop bits
        noInterrupts();
        memcpy(&PACKET,&packet,sizeof(packet));
        interrupts();
        serialTimer.priority(255);
        serialTimer.begin(sendByte,130);
    #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__) || defined(STM32L496xx) || defined(STM32L476xx) || defined(STM32L433xx) || defined(STM32L432xx) || defined(_BOARD_MAPLE_MINI_H_)  // Teensy 3.5 || Teensy 3.6 || Teensy LC || STM32L4 || Maple Mini
        // write packet
        _bus->write(packet,25);
    #elif defined(__RASPBERRYPI_ZERO_W__)
        ::write(_fd, packet, sizeof(packet));
    #endif
}

/* write SBUS packets from calibrated inputs */
void SBUS::writeCal(float* calChannels)
{
    uint16_t channels[_numChannels] = {0};
    // linear calibration
    if (calChannels)
    {
        for (uint8_t i=0; i<_numChannels; i++)
        {
            if (_useWriteCoeff[i])
            {
                calChannels[i] = PolyVal(_writeLen[i], _writeCoeff[i], calChannels[i]);
            }
            channels[i] = (calChannels[i] - _sbusBias[i])  / _sbusScale[i];
        }
    }

    write(channels);
}

void SBUS::setEndPoints(uint8_t channel,uint16_t min,uint16_t max)
{
    _sbusMin[channel] = min;
    _sbusMax[channel] = max;
    scaleBias(channel);
}

void SBUS::getEndPoints(uint8_t channel,uint16_t *min,uint16_t *max)
{
    if (min&&max) {
        *min = _sbusMin[channel];
        *max = _sbusMax[channel];
    }
}

void SBUS::setReadCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        if (!_readCoeff) {
            _readCoeff = new float*[_numChannels];
        }
        if (!_readCoeff[channel]) {
            _readCoeff[channel] = new float[len];
        } else {
            delete[] _readCoeff[channel];
            _readCoeff[channel] = new float[len];
        }
        for (uint8_t i = 0; i < len; i++) {
            _readCoeff[channel][i] = coeff[i];
        }
        _readLen[channel] = len;
        _useReadCoeff[channel] = true;
    }
}

void SBUS::getReadCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        for (uint8_t i = 0; (i < _readLen[channel]) && (i < len); i++) {
            coeff[i] = _readCoeff[channel][i];
        }
    }
}

void SBUS::setWriteCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        if (!_writeCoeff) {
            _writeCoeff = new float*[_numChannels];
        }
        if (!_writeCoeff[channel]) {
            _writeCoeff[channel] = new float[len];
        } else {
            delete[] _writeCoeff[channel];
            _writeCoeff[channel] = new float[len];
        }
        for (uint8_t i = 0; i < len; i++) {
            _writeCoeff[channel][i] = coeff[i];
        }
        _writeLen[channel] = len;
        _useWriteCoeff[channel] = true;
    }
}

void SBUS::getWriteCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        for (uint8_t i = 0; (i < _writeLen[channel]) && (i < len); i++) {
            coeff[i] = _writeCoeff[channel][i];
        }
    }
}

/* compute scale factor and bias from end points */
void SBUS::scaleBias(uint8_t channel)
{
    _sbusScale[channel] = 2.0f / ((float)_sbusMax[channel] - (float)_sbusMin[channel]);
    _sbusBias[channel] = -1.0f*((float)_sbusMin[channel] + ((float)_sbusMax[channel] - (float)_sbusMin[channel]) / 2.0f) * _sbusScale[channel];
}

float SBUS::PolyVal(size_t PolySize, float *Coefficients, float X) {
    if (Coefficients)
    {
        float Y = Coefficients[0];
        for (uint8_t i = 1; i < PolySize; i++)
        {
            Y = Y*X + Coefficients[i];
        }
        return(Y);
    } else
    {
        return 0;
    }
}

int SBUS::bytesAvalaible()
{
    int bytes_avail;
    ioctl(_fd, FIONREAD, &bytes_avail);
    return bytes_avail;
}

}