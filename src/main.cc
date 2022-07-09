/*
gal_tester
Copyright (c) 2022, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include <SPI.h>
constexpr auto I9 = A0;
constexpr auto CS = A1;
constexpr auto RESET_IOEXP = A2;
constexpr auto IOEXP_INT = 2;
constexpr auto I1_CLK = 3;

/**
 * @brief The set of registers exposed by the MCP23S17 in the default bank mode
 */
enum class MCP23x17Registers : byte {
    IODIRA = 0,
    IODIRB,
    IPOLA,
    IPOLB,
    GPINTENA,
    GPINTENB,
    DEFVALA,
    DEFVALB,
    INTCONA,
    INTCONB,
    _IOCONA,
    _IOCONB,
    GPPUA,
    GPPUB,
    INTFA,
    INTFB,
    INTCAPA,
    INTCAPB,
    GPIOA,
    GPIOB,
    OLATA,
    OLATB,
    OLAT = OLATA,
    GPIO = GPIOA,
    IOCON = _IOCONA,
    IODIR = IODIRA,
    INTCAP = INTCAPA,
    INTF = INTFA,
    GPPU = GPPUA,
    INTCON = INTCONA,
    DEFVAL = DEFVALA,
    GPINTEN = GPINTENA,
    IPOL = IPOLA,
};

enum class IOExpanderAddress : byte {
    GAL_16V8_Element = 0b0000,
    OtherDevice0 = 0b0010,
    OtherDevice1 = 0b0100,
    OtherDevice2 = 0b0110,
    OtherDevice3 = 0b1000,
    OtherDevice4 = 0b1010,
    OtherDevice5 = 0b1100,
    OtherDevice6 = 0b1110,
};
byte generateReadOpcode(IOExpanderAddress address) noexcept {
    return 0b0100'0001 | static_cast<uint8_t>(address);
}
byte generateWriteOpcode(IOExpanderAddress address) noexcept {
    return 0b0100'0000 | static_cast<uint8_t>(address);
}
/**
 * @brief Read a 16-bit value from a given io expander register
 * @tparam addr The io expander to read from
 * @tparam opcode The register pair to read from
 * @param enable the enable pin
 * @return The 16-bit value pulled from the io expander
 */
template<IOExpanderAddress addr, MCP23x17Registers opcode>
uint16_t read16(int enable) noexcept {
    SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
    digitalWrite(enable, LOW);
    (void)SPI.transfer(generateReadOpcode(addr));
    (void)SPI.transfer(static_cast<byte>(opcode));
    auto lower = static_cast<uint16_t>(SPI.transfer(0));
    auto upper = static_cast<uint16_t>(SPI.transfer(0)) << 8;
    digitalWrite(enable, HIGH);
    SPI.endTransaction();
    return lower | upper;
}
/**
 * @brief Read a 8-bit value from a given io expander register
 * @tparam addr The io expander to read from
 * @tparam opcode The register pair to read from
 * @return The contents of an 8-bit register on the io expander
 */
template<IOExpanderAddress addr, MCP23x17Registers opcode>
uint8_t read8(int enable) noexcept {
    SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
    digitalWrite(enable, LOW);
    (void)SPI.transfer(generateReadOpcode(addr));
    (void)SPI.transfer(static_cast<byte>(opcode));
    auto value = static_cast<uint16_t>(SPI.transfer(0));
    digitalWrite(enable, HIGH);
    SPI.endTransaction();
    return value;
}
/**
 * @brief Write a 16-bit value to a register pair on a target io expander
 * @tparam addr The expander to talk to
 * @tparam opcode The register pair to update
 * @param value The 16-bit value to send to the io expander
 */
template<IOExpanderAddress addr, MCP23x17Registers opcode>
void write16(int enable, uint16_t value) noexcept {
    SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
    digitalWrite(enable, LOW);
    (void)SPI.transfer(generateWriteOpcode(addr));
    (void)SPI.transfer(static_cast<byte>(opcode));
    (void)SPI.transfer(static_cast<uint8_t>(value));
    (void)SPI.transfer(static_cast<uint8_t>(value >> 8));
    digitalWrite(enable, HIGH);
    SPI.endTransaction();
}
/**
 * @brief Write an 8-bit value to a register on a target io expander
 * @tparam addr The expander to talk to
 * @tparam opcode The register to update
 * @param value The 8-bit value to send to the io expander
 */
template<IOExpanderAddress addr, MCP23x17Registers opcode>
void write8(int enable, uint8_t value) noexcept {
    SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
    digitalWrite(enable, LOW);
    (void)SPI.transfer(generateWriteOpcode(addr));
    (void)SPI.transfer(static_cast<byte>(opcode));
    (void)SPI.transfer(static_cast<uint8_t>(value));
    digitalWrite(enable, HIGH);
    SPI.endTransaction();
}
/**
 * @brief Read all 16 GPIOs of an io expander
 * @tparam addr The io expander to read from
 * @return The contents of the GPIO register pair
 */
template<IOExpanderAddress addr>
inline auto readGPIO16(int enable) noexcept {
    return read16<addr, MCP23x17Registers::GPIO>(enable);
}
/**
 * @brief Set all 16 GPIOs of an io expander
 * @tparam addr The io expander to write to
 * @param value The value to set the gpios to
 */
template<IOExpanderAddress addr>
inline void writeGPIO16(int enable, uint16_t value) noexcept {
    write16<addr, MCP23x17Registers::GPIO>(enable, value);
}
/**
 * @brief Describe the directions of all 16 pins on a given io expander.
 * @tparam addr The io expander to update
 * @param value The 16-bit direction mask to write to the io expander (a 1 means input, a 0 means output)
 */
template<IOExpanderAddress addr>
inline void writeDirection(int enable, uint16_t value) noexcept {
    write16<addr, MCP23x17Registers::IODIR>(enable, value);
}
void setInputs(uint16_t values) noexcept {
    union {
        uint16_t raw;
        struct {
            uint16_t i1 : 1;
            uint16_t theByte : 8;
            uint16_t i9 : 1;
            uint16_t rest : 6;
        };
    } thingy;
    thingy.raw = values;
    digitalWrite(I1_CLK, thingy.i1 ? HIGH : LOW);
    digitalWrite(I9, thingy.i1 ? HIGH : LOW);
    write8<IOExpanderAddress::GAL_16V8_Element, MCP23x17Registers::GPIOB>(CS, thingy.theByte);
}

void 
setAllPinsDirections(uint16_t value) noexcept {
    writeDirection<IOExpanderAddress::GAL_16V8_Element>(CS, value);
}
void
setIOPinsDirection(uint8_t value) noexcept {
    setAllPinsDirections(0xFF00 | static_cast<uint16_t>(value)); 
}
void 
setup() {
    Serial.begin(115200);
    SPI.begin();
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    pinMode(RESET_IOEXP, OUTPUT);
    digitalWrite(RESET_IOEXP, LOW);
    digitalWrite(RESET_IOEXP, HIGH);
    pinMode(I9, OUTPUT);
    digitalWrite(I9, LOW);
    pinMode(I1_CLK, OUTPUT);
    pinMode(IOEXP_INT, INPUT_PULLUP);
    setAllPinsDirections(0xFF00);
}

void
loop() {

}

