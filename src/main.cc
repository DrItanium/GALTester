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

#include "BoardTarget.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <avr/pgmspace.h>
#ifdef TARGET_BOARD_ARDUINO_UNO
constexpr auto CS = 10;
constexpr auto RESET_IOEXP = 9;
constexpr auto IOEXP_INT = 8;
constexpr auto I9 = 7;
constexpr auto I1_CLK = 6;
constexpr auto CardDetect = 5;
constexpr auto SDSelect = 4;
#elif defined(TARGET_BOARD_ARDUINO_MEGA2560)
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
constexpr auto CS = A0;
constexpr auto RESET_IOEXP = A2;
constexpr auto IOEXP_INT = 2;
constexpr auto I9 = A1;
constexpr auto I1_CLK = 5;
constexpr auto CardDetect = -1;
constexpr auto SDSelect = 4;
constexpr auto TFTCS = 10;
constexpr auto TFTDC = 9;

Adafruit_ILI9341 tft(TFTCS, TFTDC);
Adafruit_FT6206 tscreen;
#elif defined(TARGET_BOARD_NRF52832_BLUEFRUIT_FEATHER)
constexpr auto CS = 27;
constexpr auto RESET_IOEXP = 30;
constexpr auto IOEXP_INT = 11;
constexpr auto I9 = 7;
constexpr auto I1_CLK = 15;
#else 
#error "unknown board target!"
#endif



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
class GALInterface {
        static constexpr byte generateReadOpcode(IOExpanderAddress addr) noexcept { return 0b0100'0001 | static_cast<uint8_t>(addr); }
        static constexpr byte generateWriteOpcode(IOExpanderAddress addr) noexcept { return 0b0100'0000 | static_cast<uint8_t>(addr); }
    public:
        GALInterface(byte cs, byte oe, byte clk, byte reset, byte interrupt, IOExpanderAddress address);
        void begin() noexcept;
    private:
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

        /**
         * @brief Read a 16-bit value from a given io expander register
         * @tparam addr The io expander to read from
         * @tparam opcode The register pair to read from
         * @param enable the enable pin
         * @return The 16-bit value pulled from the io expander
         */
        uint16_t read16(MCP23x17Registers opcode) const noexcept {
            SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
            digitalWrite(cs_, LOW);
            (void)SPI.transfer(readOpcode_);
            (void)SPI.transfer(static_cast<byte>(opcode));
            auto lower = static_cast<uint16_t>(SPI.transfer(0));
            auto upper = static_cast<uint16_t>(SPI.transfer(0)) << 8;
            digitalWrite(cs_, HIGH);
            SPI.endTransaction();
            return lower | upper;
        }
        /**
         * @brief Read a 8-bit value from a given io expander register
         * @tparam addr The io expander to read from
         * @tparam opcode The register pair to read from
         * @return The contents of an 8-bit register on the io expander
         */
        uint8_t read8(MCP23x17Registers opcode) const noexcept {
            SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
            digitalWrite(cs_, LOW);
            (void)SPI.transfer(readOpcode_);
            (void)SPI.transfer(static_cast<byte>(opcode));
            auto value = static_cast<uint16_t>(SPI.transfer(0));
            digitalWrite(cs_, HIGH);
            SPI.endTransaction();
            return value;
        }
        /**
         * @brief Write a 16-bit value to a register pair on a target io expander
         * @tparam addr The expander to talk to
         * @tparam opcode The register pair to update
         * @param value The 16-bit value to send to the io expander
         */
        void write16(MCP23x17Registers opcode, uint16_t value) noexcept {
            SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
            digitalWrite(cs_, LOW);
            (void)SPI.transfer(writeOpcode_);
            (void)SPI.transfer(static_cast<byte>(opcode));
            (void)SPI.transfer(static_cast<uint8_t>(value));
            (void)SPI.transfer(static_cast<uint8_t>(value >> 8));
            digitalWrite(cs_, HIGH);
            SPI.endTransaction();
        }
        /**
         * @brief Write an 8-bit value to a register on a target io expander
         * @tparam addr The expander to talk to
         * @tparam opcode The register to update
         * @param value The 8-bit value to send to the io expander
         */
        void write8(MCP23x17Registers opcode, uint8_t value) noexcept {
            SPI.beginTransaction(SPISettings(F_CPU, MSBFIRST, SPI_MODE0));
            digitalWrite(cs_, LOW);
            (void)SPI.transfer(writeOpcode_);
            (void)SPI.transfer(static_cast<byte>(opcode));
            (void)SPI.transfer(static_cast<uint8_t>(value));
            digitalWrite(cs_, HIGH);
            SPI.endTransaction();
        }
#if 0
        uint8_t sampleOutputs() noexcept {
            return read8<IOExpanderAddress::GAL_16V8_Element, MCP23x17Registers::GPIOA>(CS);
        }


        void setAllPinsDirections(uint16_t value) noexcept {
            writeDirection<IOExpanderAddress::GAL_16V8_Element>(CS, value);
        }
        void setIOPinsDirection(uint8_t value) noexcept {
            setAllPinsDirections(0xFF00 | static_cast<uint16_t>(value)); 
        }
#endif
        union InputState {
            uint32_t full;
            struct {
                uint32_t clkState : 1;
                uint32_t inputs : 8;
                uint32_t oeState : 1;
                uint32_t ioPins : 8;
            } bits;
            uint32_t getMaskedState() const noexcept { return full & 0b11111111'1'11111111'1; }
        };
    private:
        void updateInputs() noexcept;
    public:
        /**
         * @brief Configure the pins which can be input or output, 0 is output,
         * 1 is input; internally the class inverts the pattern when
         * configuring the MCP23S17.
         */
        void configureIOPins(uint8_t pattern) noexcept;
        void configureIOPin(int32_t pin, int mode) noexcept;
        uint8_t getIOPinConfiguration() const noexcept { return ioPinConfiguration_; }
        uint8_t readOutputs() const noexcept;
        void setInputs(uint32_t pattern) noexcept;
        void setClockFrequency(int frequency) noexcept;
        void treatClockPinAsDigitalInput() noexcept { setClockFrequency(0); }
        bool clockPinIsDigitalInput() const noexcept { return clockFrequency_ == 0; }
        constexpr auto getClockFrequency() const noexcept { return clockFrequency_; }
        void setInput(int pin, bool state) noexcept;
        bool getInputState(int pin) const noexcept;
        void printPinStates() const noexcept;
        bool isOutputPin(int pin) const noexcept;
        bool isInputPin(int pin) const noexcept;
        void displayRegisters() const noexcept;
        bool isIOPin(int pin) const noexcept;
    private:
        uint8_t cs_;
        uint8_t oe_;
        uint8_t clk_;
        uint8_t reset_;
        uint8_t int_;
        IOExpanderAddress addr_;
        uint8_t readOpcode_;
        uint8_t writeOpcode_;
        uint8_t ioPinConfiguration_ = 0; 
        //uint8_t ioPinPullup_ = 0;
        InputState inputPinState_;
        int clockFrequency_ = 0; 
};
GALInterface::GALInterface(byte chipSelect, byte oe, byte clk, byte reset, byte intPin, IOExpanderAddress address) : 
    cs_(chipSelect), 
    oe_(oe),
    clk_(clk),
    reset_(reset),
    int_(intPin),
    addr_(address), 
    readOpcode_(GALInterface::generateReadOpcode(address)), 
    writeOpcode_(GALInterface::generateWriteOpcode(address)) 
{

}
void 
GALInterface::configureIOPin(int32_t pin, int mode) noexcept {
    if (isIOPin(pin)) {
        switch (auto temporary = ioPinConfiguration_; mode) {
            case INPUT:
                /// @todo disable pullups
                temporary |= (1 << (pin - 10));
                configureIOPins(temporary);
                break;
            case OUTPUT:
                temporary &= ~(1 << (pin - 10));
                configureIOPins(temporary);
                break;
            case INPUT_PULLUP:
                /// @todo configure pullups
                temporary |= (1 << (pin - 10));
                configureIOPins(temporary);
                break;
            default:
                break;
        }
    }
}
bool
GALInterface::isIOPin(int pin) const noexcept {
    return pin >= 10 && pin <= 17;
}
bool
GALInterface::isOutputPin(int pin) const noexcept {
    uint8_t inverse = ~ioPinConfiguration_;
    switch (pin) {
        case 10: return ((inverse) & (1 << 0)) == 1;
        case 11: return ((inverse) & (1 << 1)) == 1;
        case 12: return ((inverse) & (1 << 2)) == 1;
        case 13: return ((inverse) & (1 << 3)) == 1;
        case 14: return ((inverse) & (1 << 4)) == 1;
        case 15: return ((inverse) & (1 << 5)) == 1;
        case 16: return ((inverse) & (1 << 6)) == 1;
        case 17: return ((inverse) & (1 << 7)) == 1;
        default: return false;
    }
}

bool
GALInterface::isInputPin(int pin) const noexcept {
    uint8_t inverse = ~ioPinConfiguration_;
    switch (pin) {
        case 10: return ((inverse) & (1 << 0)) == 0;
        case 11: return ((inverse) & (1 << 1)) == 0;
        case 12: return ((inverse) & (1 << 2)) == 0;
        case 13: return ((inverse) & (1 << 3)) == 0;
        case 14: return ((inverse) & (1 << 4)) == 0;
        case 15: return ((inverse) & (1 << 5)) == 0;
        case 16: return ((inverse) & (1 << 6)) == 0;
        case 17: return ((inverse) & (1 << 7)) == 0;
        default: return true;
    }
}

uint8_t
GALInterface::readOutputs() const noexcept {
    return read8(MCP23x17Registers::GPIOA);
}
void
GALInterface::configureIOPins(uint8_t pattern) noexcept {
    ioPinConfiguration_ = pattern;
    // invert the bits since we want inputs _TO_ the GAL being an output and 
    // outputs _FROM_ the gal being inputs to the chip
    write8(MCP23x17Registers::IODIRA, ~ioPinConfiguration_);
}
void
GALInterface::setClockFrequency(int freq) noexcept {
    clockFrequency_ = freq;
    if (freq == 0) {
        digitalWrite(clk_, LOW);
    } else {
        analogWrite(clk_, freq);
    }
}
void
GALInterface::setInput(int pin, bool state) noexcept {
    auto setInputPinValue = [this, theMask = static_cast<uint8_t>(1 << (pin - 1))](bool state) {
        if (state) {
            //set the pin
            inputPinState_.bits.inputs |= theMask;
        } else {
            // clear the pin
            inputPinState_.bits.inputs &= ~theMask;
        }
    };
    auto setOutputPinValue = [this, theMask = static_cast<uint8_t>(1 << (pin - 11))](bool state) {
        if (state) {
            inputPinState_.bits.ioPins |= theMask;
        } else {
            inputPinState_.bits.ioPins &= ~theMask;
        }
    };
    switch (pin) {
        case 0: 
            inputPinState_.bits.clkState = state;
            break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
            setInputPinValue(state);
            break;
        case 9:
            inputPinState_.bits.oeState = state;
            break;
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
            setOutputPinValue(state);
            break;
        default:
            break;
    }
    updateInputs();
}
void
GALInterface::updateInputs() noexcept {
    if (clockPinIsDigitalInput()) {
        digitalWrite(clk_, inputPinState_.bits.clkState);
    }
    digitalWrite(oe_, inputPinState_.bits.oeState);
    write8(MCP23x17Registers::OLATB, inputPinState_.bits.inputs);
    write8(MCP23x17Registers::OLATA, inputPinState_.bits.ioPins);
}

void
GALInterface::setInputs(uint32_t pattern) noexcept {
    inputPinState_.full = pattern;
    updateInputs();
}
bool
GALInterface::getInputState(int pin) const noexcept {
    switch (pin) {
        case 0: 
            return inputPinState_.bits.clkState;
        case 1:
            return inputPinState_.bits.inputs & 0b0000'0001;
        case 2:
            return inputPinState_.bits.inputs & 0b0000'0010;
        case 3:
            return inputPinState_.bits.inputs & 0b0000'0100;
        case 4:
            return inputPinState_.bits.inputs & 0b0000'1000;
        case 5:
            return inputPinState_.bits.inputs & 0b0001'0000;
        case 6:
            return inputPinState_.bits.inputs & 0b0010'0000;
        case 7:
            return inputPinState_.bits.inputs & 0b0100'0000;
        case 8:
            return inputPinState_.bits.inputs & 0b1000'0000;
        case 9:
            return inputPinState_.bits.oeState;
        case 10:
            return inputPinState_.bits.ioPins & 0b0000'0001;
        case 11:
            return inputPinState_.bits.ioPins & 0b0000'0010;
        case 12:
            return inputPinState_.bits.ioPins & 0b0000'0100;
        case 13:
            return inputPinState_.bits.ioPins & 0b0000'1000;
        case 14:
            return inputPinState_.bits.ioPins & 0b0001'0000;
        case 15:
            return inputPinState_.bits.ioPins & 0b0010'0000;
        case 16:
            return inputPinState_.bits.ioPins & 0b0100'0000;
        case 17:
            return inputPinState_.bits.ioPins & 0b1000'0000;
        default:
            return false;
    }
}

void
GALInterface::begin() noexcept {
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);
    pinMode(oe_, OUTPUT);
    digitalWrite(oe_, LOW);
    pinMode(clk_, OUTPUT);
    digitalWrite(clk_, LOW);
    pinMode(reset_, OUTPUT);
    digitalWrite(reset_, LOW);
    digitalWrite(reset_, HIGH);
    pinMode(IOEXP_INT, INPUT_PULLUP);
    write16(MCP23x17Registers::IODIR, 0x00FF); // PORTA is set to inputs, PORTB are outputs
    write16(MCP23x17Registers::OLAT, 0xFFFF);
    setInputs(0); // set all inputs to low
    configureIOPins(0); // all will be _outputs_
    // setup the MCP23S17 as needed
}
void 
GALInterface::printPinStates() const noexcept {
    Serial.println(F("Pin states as seen from the perspective of the gal"));
    Serial.print(F("P1: "));
    if (clockPinIsDigitalInput()) {
        Serial.print(F("I ("));
        if (getInputState(0)) {
            Serial.print(F("HIGH"));
        } else {
            Serial.print(F("LOW"));
        }
    } else {
        Serial.print(F("CLK ("));
        Serial.print(getClockFrequency());
    }
    Serial.println(F(")"));
    for (int i = 1; i < 9; ++i) {
        Serial.print(F("P"));
        Serial.print(i + 1);
        Serial.print(F(": I ("));
        if (getInputState(i)) {
            Serial.println(F("HIGH)"));
        } else {
            Serial.println(F("LOW)"));
        }
    }
    Serial.println(F("P10: GND"));
    Serial.print(F("P11: I/~{OE} ("));
    if (getInputState(9)) {
        Serial.println(F("HIGH)"));
    } else {
        Serial.println(F("LOW)"));
    }
    auto sample = readOutputs();
    for (int i = 10, j = 0; i < 18; ++i, ++j) {
        Serial.print(F("P"));
        Serial.print((i + 2));
        if (isInputPin(i)) {
            if (getInputState(i)) {
                Serial.println(F(": I (LOW)"));
            } else {
                Serial.println(F(": I (HIGH)"));
            }
        } else {
            auto bit = sample & (1 << j);
            Serial.print(F(": O ("));
            Serial.print(bit ? F("HIGH") : F("LOW"));
            Serial.println(F(")"));
        }
    }

    Serial.println(F("P20: VCC"));
}
GALInterface iface(CS, 
        I9, 
        I1_CLK, 
        RESET_IOEXP, 
        IOEXP_INT, 
        IOExpanderAddress::GAL_16V8_Element);
void setupLookupTable() noexcept; 
volatile bool sdEnabled = false;
void 
setup() {
    Serial.begin(115200);
    Serial.println(F("GAL Testing Interface"));
    Serial.println(F("(C) 2022 Joshua Scoggins"));
    Serial.println(F("This is open source software! See LICENSE for details"));
#ifdef TARGET_BOARD_ARDUINO_MEGA2560
    Serial.println();
    Serial.println(F("TFT Bringup"));
    tft.begin();
    auto x = tft.readcommand8(ILI9341_RDMODE);
    Serial.print(F("Display Power Mode: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    Serial.print(F("MADCTL Mode: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDPIXFMT);
    Serial.print(F("Pixel Format: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDIMGFMT);
    Serial.print(F("Image Format: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDSELFDIAG);
    Serial.print(F("Self Diagnostic: 0x")); Serial.println(x, HEX);
    Serial.println();
    Serial.println(F("Clearing Screen!"));
    tft.fillScreen(ILI9341_BLACK);

    Serial.println();
    Serial.println(F("Touchscreen Bringup"));
    tscreen.begin();
#endif
    SPI.begin();
    setupLookupTable();
    iface.begin();
    iface.configureIOPins(0);
#ifndef TARGET_BOARD_ARDUINO_UNO
    pinMode(SDSelect, OUTPUT);
    digitalWrite(SDSelect, HIGH);
    if constexpr (CardDetect != -1) {
        pinMode(CardDetect, INPUT);
    }
    Serial.println(F("Checking for an SD Card..."));
    if (!SD.begin(SDSelect)) {
        if constexpr (CardDetect != -1) {
            if (digitalRead(CardDetect) == HIGH) {
                Serial.println(F("SD CARD FOUND BUT ERROR OCCURRED DURING INIT!"));
            } else {
                Serial.println(F("SD CARD NOT INSERTED"));
            }
        } 
        Serial.println(F("SD CARD NOT AVAILABLE!!!"));
    } else {
        sdEnabled = true;
        Serial.println(F("SD CARD AVAILABLE!"));
    }
#endif
    Serial.println();
}
void read() noexcept;
void eval() noexcept;
void loop() {
    read();
    eval();
}


class Word {
    public:
        explicit Word(const String& name) : name_(name) { }
        virtual ~Word() = default;
        [[nodiscard]] virtual bool matches(const String& name) const noexcept { return name == name_; }
        virtual bool invoke(const String& match) noexcept = 0;
        const String& getName() const noexcept { return name_; }
    private:
        String name_;
};
class LambdaWord : public Word {
    public:
        using Parent = Word;
        using Function = bool (*)(const String&);
        explicit LambdaWord(const String& name, Function theFunction) : Parent(name), func_(theFunction)  { }
        ~LambdaWord() override = default;
        bool invoke(const String& match) noexcept override {
            return func_(match);
        }
    private:
        Function func_;
};
extern Word* lookupTable[256];
bool
listWords(const String&) noexcept {
    Serial.println(F("Registered Words:"));
    int count = 0;
    for (auto* word : lookupTable) {
        if (word) {
            Serial.print(F("- "));
            Serial.println(word->getName());
            ++count;
        }
    }
    Serial.println();
    Serial.print(F("Found "));
    Serial.print(count);
    Serial.println(F(" words in dictionary"));
    Serial.println();
    return true;
}
bool
displayPinout(const String&) noexcept {
    iface.printPinStates();
    return true;
}
void 
GALInterface::displayRegisters() const noexcept {
    Serial.println(F("GAL Interface Registers"));
    Serial.print(F("IO Pins Configuration: 0b"));
    Serial.println(static_cast<byte>(~ioPinConfiguration_), BIN);
    Serial.print(F("Input values: 0b"));
    Serial.println(inputPinState_.getMaskedState(), BIN);
}
bool
displayRegisters(const String&) noexcept {
    iface.displayRegisters();
    return true;
}


Word* findWord(const String& word) noexcept {
    for (auto* theWord: lookupTable) {
        if (theWord && theWord->matches(word)) {
            return theWord;
        }
    }
    return nullptr;
}
enum class ErrorCodes {
    None,
    GeneralFailure,
    UnknownWord,
    BadNumberConvert,
    Unimplemented,
    StackFull,
    StackEmpty,
    NotEnoughStackElements,
    DivideByZero,
};
extern String inputString;
extern bool stringComplete;
extern unsigned int position;
extern ErrorCodes errorMessage;
void
read() {
    while (Serial.available()) {
        char inChar = static_cast<char>(Serial.read());
        inputString += inChar;
        Serial.print(inChar);
        if (inChar == '\n') {
            stringComplete = true;
        } 
    }
}
bool
eval(const String& word) noexcept {
    if (word.length() == 0) {
        return true;
    } else {
        if (auto target = findWord(word); target) {
            return target->invoke(word);
        } else {
            errorMessage = ErrorCodes::GeneralFailure;
            return false;
        }
    }
}
struct TreatAsBoolean { };
bool pushItemOntoStack(int32_t value) noexcept;
bool pushItemOntoStack(bool value, TreatAsBoolean) noexcept { return pushItemOntoStack(value ? 0xFFFF'FFFF : 0); }
template<int32_t constant>
bool pushItemOntoStack(const String&) noexcept { 
    return pushItemOntoStack(constant);
}
bool popItemOffStack(int32_t& item) noexcept;
bool dropTopOfStack() noexcept;
bool dropTopOfStack(const String&) noexcept { return dropTopOfStack(); }
bool stackEmpty() noexcept;
bool stackFull() noexcept;
void clearStack() noexcept;
void clearState() noexcept;
uint32_t stackCapacity() noexcept;
uint32_t numberOfItemsOnStack() noexcept;
void handleError(bool errorState) noexcept;
bool
handleError(bool state, bool stillEvaluating) noexcept {
    // okay so an error happened during evaluation, so we need to clean up
    // and start again
    switch (errorMessage) {
        case ErrorCodes::None: 
            break;
        case ErrorCodes::BadNumberConvert: 
            Serial.println(F("bad numeric conversion")); 
            break;
        case ErrorCodes::DivideByZero: 
            Serial.println(F("divide by zero")); 
            break;
        case ErrorCodes::NotEnoughStackElements:
            Serial.println(F("stack underflow"));
            break;
        case ErrorCodes::StackEmpty:
            Serial.println(F("stack empty"));
            break;
        case ErrorCodes::StackFull:
            Serial.println(F("stack full"));
            break;
        case ErrorCodes::Unimplemented:
            Serial.println(F("unimplemented"));
            break;
        case ErrorCodes::UnknownWord:
            Serial.println(F("unknown word"));
            break;
        default: 
            Serial.println(F("some error happened")); 
            break;
    }
    if (!stillEvaluating || !state) {
        clearState();
    }
    if (!state) {
        clearStack();
    }
    return state;
}
void
clearState() {
    errorMessage = ErrorCodes::None; 
    inputString = "";
    stringComplete = false;
    position = 0;
}

void
eval() noexcept {
    if (stringComplete) {
        stringComplete = false;
        String subWord = "";
        for (position = 0; position < inputString.length(); ) {
            switch (auto c = inputString[position]; c) {
                case ' ':
                case '\t':
                case '\n':
                    // advance position ahead of time in this case
                    ++position;
                    if (!handleError(eval(subWord), true)) {
                        return;
                    } else {
                        subWord = "";
                    }
                    break;
                case '\r':
                    // carriage returns should be ignored
                    ++position;
                    break;
                default:
                    subWord += c;
                    ++position;
                    break;
            }
        }
        if (handleError(eval(subWord), false)) {
            // defer printing out ok until we are done
            Serial.println(F("\tok"));
        }
    } 
}


#ifdef ARDUINO_AVR_UNO
#if __cplusplus >= 201402L
void operator delete(void* ptr, size_t) {
    ::operator delete(ptr);
}

void operator delete[](void* ptr, size_t) {
    ::operator delete(ptr);
}
#endif 
#endif

bool
setClkFrequency(const String&) noexcept {
    errorMessage = ErrorCodes::Unimplemented;
    return false;
}
class InvokeOnPrefixMatchWord : public Word {
    public:
        InvokeOnPrefixMatchWord(const String& name, const String& prefix) : Word(name), prefix_(prefix) { }
        ~InvokeOnPrefixMatchWord() override = default;
        bool matches(const String& message) const noexcept override;
        bool invoke(const String& match) noexcept override;
    protected:
        virtual bool invoke0(const String& substringMatch) noexcept = 0;
    private:
        String prefix_;
};
bool
InvokeOnPrefixMatchWord::matches(const String& word) const noexcept {
    if (prefix_.length() == 0) {
        return true;
    } else {
        return word.startsWith(prefix_);
    }
}
bool
InvokeOnPrefixMatchWord::invoke(const String& match) noexcept {
    // if we got here then it is safe to strip the prefix off
    auto substring = match.substring(prefix_.length());
    substring.trim();
    return invoke0(substring);
}
class ParseNumberAndPushOntoStack : public InvokeOnPrefixMatchWord {
    public:
        ParseNumberAndPushOntoStack(const String& name, const String& prefix) : InvokeOnPrefixMatchWord(name, prefix) { }
        ~ParseNumberAndPushOntoStack() override = default;
    protected:
        bool invoke0(const String& substringMatch) noexcept override;
        virtual bool parse(const String& theValue, int32_t& number) noexcept = 0;
};

bool
ParseNumberAndPushOntoStack::invoke0(const String& match) noexcept {
    int32_t value = 0;
    if (parse(match, value)) {
        return pushItemOntoStack(value);
    } else {
        return false;
    }
}

class NumericBaseCapture : public ParseNumberAndPushOntoStack {
    public:
        NumericBaseCapture(const String& name, const String& prefix, int numericBase) : ParseNumberAndPushOntoStack(name, prefix), base_(numericBase) { }
    protected:
        bool parse(const String& theValue, int32_t& number) noexcept override;
    private:
        int base_;

};
bool
NumericBaseCapture::parse(const String& theValue, int32_t& result) noexcept {
    char* firstBad = nullptr;
    auto theStr = theValue.c_str();
    auto number = strtol(theStr, &firstBad, base_); 
    if (firstBad != nullptr) {
        // okay so we may have got a bad conversion
        // it could also be that we were successful but strtoul views the
        // character beyond the end as "bad"
        // 
        // So convert the pointer addresses and do some calculations to see if
        // we were actually unsuccessful
        auto strStart = reinterpret_cast<uint32_t>(theStr);
        auto badEnd = reinterpret_cast<uint32_t>(firstBad);
        if ((strStart + theValue.length()) == badEnd) {
            // we were actually successful! So assign result the number we got
            result = number;
            return true;
        } else if (strStart == badEnd) {
            errorMessage = ErrorCodes::UnknownWord;
            return false;
        } else {
            errorMessage = ErrorCodes::BadNumberConvert;
            return false;
        }
    } else {
        // okay so we were successful
        result = number;
        return true;
    }
}

uint32_t lookupTableCount = 0;
bool
defineWord(Word* theWord) noexcept {
    if (theWord && (lookupTableCount < 256)) {
        lookupTable[lookupTableCount] = theWord;
        ++lookupTableCount;
        return true;
    } 
    return false;
}
bool
defineWord(const String& name, LambdaWord::Function func) noexcept {
    return defineWord(new LambdaWord(name, func));
}
template<typename T, typename ... Args>
bool
defineSpecialWord(const String& name, Args&& ... args) noexcept {
    return defineWord(new T(name, args...));
}
bool popAndPrintStackTop(const String&) noexcept;
bool printStackContents(const String&) noexcept;
bool addTwoNumbers(const String&) noexcept;
bool subtractTwoNumbers(const String&) noexcept;
bool multiplyTwoNumbers(const String&) noexcept;
bool divideTwoNumbers(const String&) noexcept;
bool moduloTwoNumbers(const String&) noexcept;
bool twoNumbersEqual(const String&) noexcept;
bool twoNumbersNotEqual(const String&) noexcept;
bool topGreaterThanLower(const String&) noexcept;
bool topLessThanLower(const String&) noexcept;
bool topGreaterThanOrEqualLower(const String&) noexcept;
bool topLessThanOrEqualLower(const String&) noexcept;
bool duplicateTop(const String&) noexcept;
bool setIOPinMode(const String&) noexcept;
bool touchPoint(const String&) noexcept;
void
setupLookupTable() noexcept {
    defineWord(F("words"), listWords);
    defineWord(F("pins"), displayPinout);
    defineWord(F("status"), displayRegisters);
    defineWord(F("set-clock-frequency"), setClkFrequency);
    defineWord(F("drop"), dropTopOfStack);
    defineWord(F("."), popAndPrintStackTop);
    defineWord(F(".s"), printStackContents);
    defineWord(F("+"), addTwoNumbers);
    defineWord(F("-"), subtractTwoNumbers);
    defineWord(F("*"), multiplyTwoNumbers);
    defineWord(F("/"), divideTwoNumbers);
    defineWord(F("%"), moduloTwoNumbers);
    defineWord(F("dup"), duplicateTop);
    defineWord(F("=="), twoNumbersEqual);
    defineWord(F("!="), twoNumbersNotEqual);
    defineWord(F(">"), topLessThanLower);
    defineWord(F("<"), topGreaterThanLower);
    defineWord(F("<="), topLessThanOrEqualLower);
    defineWord(F(">="), topGreaterThanOrEqualLower);
    defineWord(F("io-pin-mode"), setIOPinMode);
    defineWord(F("touch-point"), touchPoint);
#define X(str, target) defineWord(F(str) , pushItemOntoStack<target>)
    X("input", INPUT);
    X("output", OUTPUT);
    //X("input-pullup", INPUT_PULLUP);
    X("low", LOW);
    X("high", HIGH);
    X("true", 0xFFFF'FFFF);
    X("false", 0);
    X("P1", 0);
    X("P2", 1);
    X("P3", 2);
    X("P4", 3);
    X("P5", 4);
    X("P6", 5);
    X("P7", 6);
    X("P8", 7);
    X("P9", 8);
    X("P11", 9);
    X("P12", 10);
    X("P13", 11);
    X("P14", 12);
    X("P15", 13);
    X("P16", 14);
    X("P17", 15);
    X("P18", 16);
    X("P19", 17);
#undef X

    defineSpecialWord<NumericBaseCapture>(F("binary-convert (prefix is 0b)"), F("0b"), 2);

    // must come last
    if (!defineSpecialWord<NumericBaseCapture>(F("fallback-capture (prefix is nothing)"), "", 0)) {
        Serial.println(F("TOO MANY WORDS DEFINED! HALTING!!"));
        while (true);
    }
}
Word* lookupTable[256] = { 0 };

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
unsigned int position = 0;
ErrorCodes errorMessage = ErrorCodes::None;
constexpr auto numStackElements = 16;
uint8_t stackPosition = numStackElements;
uint32_t theStack[numStackElements];

void 
clearStack() noexcept {
    stackPosition = numStackElements;
    for (int i = 0;i < numStackElements; ++i) {
        theStack[i] = 0;
    }
}


bool 
pushItemOntoStack(int32_t value) noexcept {
    if (stackFull()) {
        errorMessage = ErrorCodes::StackFull;
        return false;
    } else {
        theStack[--stackPosition] = value;
        return true;
    }
}
bool 
popItemOffStack(int32_t& item) noexcept {
    if (stackEmpty()) {
        errorMessage = ErrorCodes::StackEmpty;
        return false;
    } else {
        item = theStack[stackPosition];
        ++stackPosition;
        return true;
    }
}
bool 
stackEmpty() noexcept {
    return stackPosition == numStackElements;
}

bool 
stackFull() noexcept {
    return stackPosition == 0;
}

uint32_t
stackCapacity() noexcept {
    return numStackElements;
}

uint32_t
numberOfItemsOnStack() noexcept {
    return stackCapacity() - stackPosition;
}
bool
dropTopOfStack() noexcept {
    int32_t temporary = 0;
    return popItemOffStack(temporary);
}
bool
popAndPrintStackTop(const String&) noexcept {
    int32_t value = 0;
    if (popItemOffStack(value)) {
        Serial.print(value);
        return true;
    } else {
        return false;
    }
}
bool
printStackContents(const String&) noexcept {
    if (!stackEmpty()) {
        for (int i = numStackElements - 1; i >= stackPosition; --i) {
            Serial.print(theStack[i]);
            Serial.print(F(" "));
        }
    }
    Serial.println();
    return true;
}

bool 
addTwoNumbers(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower + top);
}
bool 
subtractTwoNumbers(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower - top);
}
bool 
multiplyTwoNumbers(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower * top);
}

bool 
divideTwoNumbers(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    if (top == 0) {
        errorMessage = ErrorCodes::DivideByZero;
        return false;
    }
    return pushItemOntoStack(lower / top);
}

bool 
moduloTwoNumbers(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    if (top == 0) {
        errorMessage = ErrorCodes::DivideByZero;
        return false;
    }
    return pushItemOntoStack(lower % top);
}

bool 
twoNumbersEqual(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower == top, TreatAsBoolean{});
}
bool 
twoNumbersNotEqual(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower != top, TreatAsBoolean{});
}

bool 
topGreaterThanLower(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower < top, TreatAsBoolean{});
}

bool 
topGreaterThanOrEqualLower(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower <= top, TreatAsBoolean{});
}

bool 
topLessThanLower(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower > top, TreatAsBoolean{});
}
bool 
topLessThanOrEqualLower(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t top, lower;
    popItemOffStack(top);
    popItemOffStack(lower);
    return pushItemOntoStack(lower >= top, TreatAsBoolean{});
}
bool 
duplicateTop(const String&) noexcept {
    int32_t result = 0;
    if (popItemOffStack(result)) {
        return pushItemOntoStack(result) && pushItemOntoStack(result);
    } else {
        return false;
    }
}

bool
setIOPinMode(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t mode, targetPin;
    popItemOffStack(mode);
    // assume that targetPin is actually subtracted by one
    popItemOffStack(targetPin);
    iface.configureIOPin(targetPin, mode);
    return true;
}


bool
touchPoint(const String&) noexcept {
    if (tscreen.touched()) {
        auto p = tscreen.getPoint();
        return pushItemOntoStack(p.y) && pushItemOntoStack(p.x);
    } else {
        return pushItemOntoStack(-1) && pushItemOntoStack(-1);
    }
}
