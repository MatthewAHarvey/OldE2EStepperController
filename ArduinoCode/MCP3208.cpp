#include "MCP3208.h"

void MCP3208::begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    SPI.begin();
}

uint16_t MCP3208::read(uint8_t pin) {
    uint8_t addr = 0b01100000 | ((pin & 0b111) << 2);
    digitalWrite(_cs, LOW);
    (void) SPI.transfer(addr);
    uint8_t b1 = SPI.transfer(0);
    uint8_t b2 = SPI.transfer(0);
    digitalWrite(_cs, HIGH);

    return (b1 << 4) | (b2 >> 4);
}

int16_t MCP3208::readDif(uint8_t pin) {
    uint8_t diff;
    uint8_t b1, b2;
    uint8_t addr = 0b01000000 | ((pin & 0b11) << 3);
    digitalWrite(_cs, LOW);
    (void) SPI.transfer(addr);
    b1 = SPI.transfer(0);
    b2 = SPI.transfer(0);
    digitalWrite(_cs, HIGH);

    diff = (b1 << 4) | (b2 >> 4);
    if (diff > 0) {
        return diff;
    }
    addr = 0b01000100 | ((pin & 0b11) << 3);
    digitalWrite(_cs, LOW);
    (void) SPI.transfer(addr);
    b1 = SPI.transfer(0);
    b2 = SPI.transfer(0);
    digitalWrite(_cs, HIGH);
    diff = (b1 << 4) | (b2 >> 4);
    return -diff;
}