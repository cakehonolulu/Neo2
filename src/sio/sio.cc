#include <sio/sio.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

SIORegisters SIO::sio = {};

void SIO::write(uint32_t address, uint32_t value) {
    switch (address) {
        case 0x1000F100:
            sio.LCR = value;
            break;
        case 0x1000F110:
            sio.LSR = value;
            break;
        case 0x1000F120:
            sio.IER = value;
            break;
        case 0x1000F130:
            sio.ISR = value;
            break;
        case 0x1000F140:
            sio.FCR = value;
            break;
        case 0x1000F150:
            sio.BGR = value;
            break;
        case 0x1000F180:
        {
            printf("SIO TXFIFO write: 0x%08X\n", value);
            sio.TXFIFO = value;
            break;
        }
        case 0x1000F1C0:
            sio.RXFIFO = value;
            break;
        default:
            Logger::error("Unknown SIO write address: 0x" + format("{:08X}", address));
            break;
    }
}

uint32_t SIO::read(uint32_t address) {
    switch (address) {
        case 0x1000F100:
            return sio.LCR;
        case 0x1000F110:
            return sio.LSR;
        case 0x1000F120:
            return sio.IER;
        case 0x1000F130:
            return sio.ISR;
        case 0x1000F140:
            return sio.FCR;
        case 0x1000F150:
            return sio.BGR;
        case 0x1000F180:
            return sio.TXFIFO;
        case 0x1000F1C0:
            return sio.RXFIFO;
        default:
            Logger::error("Unknown SIO read address: 0x" + format("{:08X}", address));
            return 0;
    }
}
