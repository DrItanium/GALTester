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
constexpr auto CS = 10;
constexpr auto RESET_IOEXP = 9;
constexpr auto IOEXP_INT = 8;
constexpr auto I9 = 7;
constexpr auto I1_CLK = 6;
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
bool
GALInterface::isOutputPin(int pin) const noexcept {
    switch (pin) {
        case 10: return ((~ioPinConfiguration_) & (1 << 0)) == 0;
        case 11: return ((~ioPinConfiguration_) & (1 << 1)) == 0;
        case 12: return ((~ioPinConfiguration_) & (1 << 2)) == 0;
        case 13: return ((~ioPinConfiguration_) & (1 << 3)) == 0;
        case 14: return ((~ioPinConfiguration_) & (1 << 4)) == 0;
        case 15: return ((~ioPinConfiguration_) & (1 << 5)) == 0;
        case 16: return ((~ioPinConfiguration_) & (1 << 6)) == 0;
        case 17: return ((~ioPinConfiguration_) & (1 << 7)) == 0;
        default: return false;
    }
}

bool
GALInterface::isInputPin(int pin) const noexcept {
    switch (pin) {
        case 10: return ((~ioPinConfiguration_) & (1 << 0)) == 1;
        case 11: return ((~ioPinConfiguration_) & (1 << 1)) == 1;
        case 12: return ((~ioPinConfiguration_) & (1 << 2)) == 1;
        case 13: return ((~ioPinConfiguration_) & (1 << 3)) == 1;
        case 14: return ((~ioPinConfiguration_) & (1 << 4)) == 1;
        case 15: return ((~ioPinConfiguration_) & (1 << 5)) == 1;
        case 16: return ((~ioPinConfiguration_) & (1 << 6)) == 1;
        case 17: return ((~ioPinConfiguration_) & (1 << 7)) == 1;
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
    for (int i = 10; i < 18; ++i) {
        Serial.print(F("P"));
        Serial.print((i + 2));
        if (isInputPin(i)) {
            if (getInputState(i)) {
                Serial.println(F(": I (LOW)"));
            } else {
                Serial.println(F(": I (HIGH)"));
            }
        } else {
            Serial.println(F(": O"));
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
void 
setup() {
    Serial.begin(115200);
    SPI.begin();
    iface.begin();
    iface.configureIOPins(0b00'111111);
    Serial.println(F("GAL Testing Interface"));
    Serial.println(F("(C) 2022 Joshua Scoggins"));
    Serial.println(F("This is open source software! See LICENSE for details"));
    Serial.println();
}


String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
unsigned int position = 0;
class Word {
    public:
        explicit Word(const String& name) : name_(name) { }
        virtual ~Word() = default;
        [[nodiscard]] bool matches(const String& name) const noexcept { return name == name_; }
        virtual void invoke() noexcept = 0;
        const String& getName() const noexcept { return name_; }
    private:
        String name_;
};
class LambdaWord : public Word {
    public:
        using Parent = Word;
        using Function = void (*)();
        explicit LambdaWord(const String& name, Function theFunction) : Parent(name), func_(theFunction)  { }
        ~LambdaWord() override = default;
        void invoke() noexcept override {
            func_();
        }
    private:
        Function func_;
};
extern Word* lookupTable[256];
void listWords() noexcept {
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
}
void
displayPinout() noexcept {
    iface.printPinStates();
}
Word* lookupTable[256] = { 
    new LambdaWord("words", listWords),
    new LambdaWord("pinout", displayPinout),
};
Word* findWord(const String& word) noexcept {
    for (auto* theWord: lookupTable) {
        if (theWord && theWord->matches(word)) {
            return theWord;
        }
    }
    return nullptr;
}
void
read() {
    while (Serial.available()) {
        char inChar = static_cast<char>(Serial.read());
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        } 
    }
}
void
eval(const String& word) noexcept {
    if (word.length() == 0) {
        return;
    } else {
        if (auto target = findWord(word); target) {
            target->invoke();
        } else {
            Serial.print(F("UNKNOWN WORD: \""));
            Serial.print(word);
            Serial.println(F("\""));
        }
    }
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
                    eval(subWord);
                    subWord = "";
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
        if (subWord.length() != 0) {
            eval(subWord);
            subWord = "";
        } 
        Serial.println(F("\tok"));
        // carry out the last word if we ended with a newline
        inputString = "";
    } 
}

void
loop() {
    read();
    eval();
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
