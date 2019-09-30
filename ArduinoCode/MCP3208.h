#ifndef _MCP3208_H
#define _MCP3208_H

#include <Arduino.h>
#include <SPI.h>


class MCP3208 {
    private:
        // Private functions and variables here.  They can only be accessed
        // by functions within the class.
        uint8_t _cs;

    public:
        // Public functions and variables.  These can be accessed from
        // outside the class.
        MCP3208() : _cs(10) {}
        MCP3208(uint8_t cs) : _cs(cs) {}
    
        void begin();
        uint16_t read(uint8_t pin);
        int16_t readDif(uint8_t pin);
};
#endif