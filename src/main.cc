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
#include <RTClib.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#include <Wire.h>
RTC_PCF8523 rtc;

constexpr auto CS = getCSPin();
constexpr auto RESET_IOEXP = getIOEXPResetPin();
constexpr auto I9 = getI9Pin();
constexpr auto IOEXP_INT = getIOEXPIntPin();
constexpr auto I1_CLK = getI1CLKPin();
constexpr auto SDSelect = getSDCardPin();
constexpr uint32_t MaxSPISpeed = 4 * 1000 * 1000; // 4 MHz to make sure


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
#ifndef _BV
#define _BV(x) (1 << x)
#endif
class GALPinDescription {
    public:
        enum ValidStates : byte {
            None = 0,
            Input = _BV(1),
            Output = _BV(2),
            Clock = _BV(3),
            OutputEnable = _BV(4),
            OutputEnable_Input = OutputEnable | Input,
            Input_Output = Input | Output,
            Clock_Input = Input | Clock,
        };
    public:
        constexpr GALPinDescription(int index, byte mask = _BV(0), ValidStates states = ValidStates::None) noexcept : index_(index), mask_(mask), states_(states) { }
        constexpr auto index() const noexcept { return index_; }
        constexpr auto zeroIndex() const noexcept { return index() - 1; }
        constexpr auto mask() const noexcept { return mask_; }
        constexpr auto valid() const noexcept { return states_ != ValidStates::None; }
        constexpr auto clockPin() const noexcept { return (states_ & ValidStates::Clock) != 0; }
        constexpr auto outputEnablePin() const noexcept { return (states_ & ValidStates::OutputEnable) != 0; }
        constexpr auto inputPin() const noexcept { return (states_ & ValidStates::Input) != 0; }
        constexpr auto outputPin() const noexcept { return (states_ & ValidStates::Output) != 0; }
        constexpr auto inputOnlyPin() const noexcept { return inputPin() && !outputPin() && !outputEnablePin(); }
        constexpr auto ioPin() const noexcept { return inputPin() && outputPin(); }
        constexpr operator bool() const noexcept { return valid(); }
    private:
        int index_;
        byte mask_;
        ValidStates states_;
};
static constexpr GALPinDescription GAL16V8[20] {
    GALPinDescription { 1, _BV(0), GALPinDescription::ValidStates::Clock_Input },
    GALPinDescription { 2, _BV(0), GALPinDescription::ValidStates::Input },
    GALPinDescription { 3, _BV(1), GALPinDescription::ValidStates::Input },
    GALPinDescription { 4, _BV(2), GALPinDescription::ValidStates::Input },
    GALPinDescription { 5, _BV(3), GALPinDescription::ValidStates::Input },
    GALPinDescription { 6, _BV(4), GALPinDescription::ValidStates::Input },
    GALPinDescription { 7, _BV(5), GALPinDescription::ValidStates::Input },
    GALPinDescription { 8, _BV(6), GALPinDescription::ValidStates::Input },
    GALPinDescription { 9, _BV(7), GALPinDescription::ValidStates::Input },
    GALPinDescription { 10 }, // GND
    GALPinDescription { 11, _BV(0), GALPinDescription::ValidStates::OutputEnable_Input },

    GALPinDescription { 12, _BV(0), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 13, _BV(1), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 14, _BV(2), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 15, _BV(3), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 16, _BV(4), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 17, _BV(5), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 18, _BV(6), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 19, _BV(7), GALPinDescription::ValidStates::Input_Output },
    GALPinDescription { 20 }, // VCC
};
static_assert(GAL16V8[10].outputEnablePin());
static_assert(GAL16V8[11].ioPin());
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
            SPI.beginTransaction(SPISettings(MaxSPISpeed, MSBFIRST, SPI_MODE0));
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
            SPI.beginTransaction(SPISettings(MaxSPISpeed, MSBFIRST, SPI_MODE0));
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
            SPI.beginTransaction(SPISettings(MaxSPISpeed, MSBFIRST, SPI_MODE0));
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
            SPI.beginTransaction(SPISettings(MaxSPISpeed, MSBFIRST, SPI_MODE0));
            digitalWrite(cs_, LOW);
            (void)SPI.transfer(writeOpcode_);
            (void)SPI.transfer(static_cast<byte>(opcode));
            (void)SPI.transfer(static_cast<uint8_t>(value));
            digitalWrite(cs_, HIGH);
            SPI.endTransaction();
        }
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
        /// @todo implement GAL22V10
    public:
        /**
         * @brief Configure the pins which can be input or output, 0 is output,
         * 1 is input; internally the class inverts the pattern when
         * configuring the MCP23S17.
         */
        void configureIOPins(uint8_t pattern) noexcept;
        void configureIOPin(const GALPinDescription& pin, int mode) noexcept;
        uint8_t getIOPinConfiguration() const noexcept { return read8(MCP23x17Registers::IODIRA); }
        uint8_t readOutputs() const noexcept;
        uint32_t readInputs() const noexcept;
        void setInputs(uint32_t pattern) noexcept;
        void setInput(int pin, bool state) noexcept;
        bool getInputState(const GALPinDescription& desc) const noexcept;
    private:
        template<typename T>
        void
        printSingleHighLow(T& thing, bool cond) const noexcept {
            if (cond) {
                thing.print(F("HIGH"));
            } else {
                thing.print(F("LOW"));
            }
        }
        template<typename T>
        void
        printSingleHighLowInput(T& thing, const GALPinDescription& i) const noexcept {
            printSingleHighLow(thing, getInputState(i));
        }
        template<typename T>
        void
        printPinId(T& thing, int index) const noexcept {
            thing.print(F("P"));
            thing.print(index);
            thing.print(F(": "));
        }
    public:
        template<typename T>
        void 
        printPinStates(T& thing) const noexcept {
            auto sample = readOutputs();
            thing.println(F("Pin states as seen from the perspective of the gal"));
            for (const auto& pin : GAL16V8) {
                if (pin) {
                    printPinId(thing, pin.index());
                    if (pin.clockPin()) {
                        thing.print(F("I ("));
                        printSingleHighLowInput(thing, pin);
                    } else if (pin.outputEnablePin()) {
                        thing.print(F("I/~{OE} ("));
                        printSingleHighLowInput(thing, pin);
                    } else if (pin.inputOnlyPin()) {
                        thing.print(F("I ("));
                        printSingleHighLowInput(thing, pin);
                    } else if (pin.ioPin()) {
                        if (isInputPin(pin)) {
                            thing.print(F("I ("));
                            printSingleHighLowInput(thing, pin);
                        } else {
                            thing.print(F("O ("));
                            printSingleHighLow(thing, sample & pin.mask());
                        }
                    } else {
                        thing.print(F("U ("));
                    }
                    thing.println(F(")"));
                }
            }
        }
        bool isInputPin(const GALPinDescription& pin) const noexcept;
        void displayRegisters() const noexcept;
    private:
        uint8_t cs_;
        uint8_t oe_;
        uint8_t clk_;
        uint8_t reset_;
        uint8_t int_;
        IOExpanderAddress addr_;
        uint8_t readOpcode_;
        uint8_t writeOpcode_;
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
GALInterface::configureIOPin(const GALPinDescription& pin, int mode) noexcept {
    if (pin.ioPin()) {
        switch (auto temporary = getIOPinConfiguration(); mode) {
            case INPUT:
                temporary |= pin.mask();
                configureIOPins(temporary);
                break;
            case OUTPUT:
                temporary &= ~(pin.mask());
                configureIOPins(temporary);
                break;
            default:
                break;
        }
    }
}

bool
GALInterface::isInputPin(const GALPinDescription& pin) const noexcept {
    return pin.valid() && (pin.inputOnlyPin() || ((static_cast<uint8_t>(getIOPinConfiguration()) & static_cast<uint8_t>(pin.mask())) == 0));
}

uint8_t
GALInterface::readOutputs() const noexcept {
    return read8(MCP23x17Registers::GPIOA);
}
uint32_t 
GALInterface::readInputs() const noexcept {
    InputState state;
    state.bits.oeState = digitalRead(oe_) != 0 ? 1 : 0;
    state.bits.clkState = digitalRead(clk_) != 0 ? 1 : 0;
    state.bits.ioPins = read8(MCP23x17Registers::OLATA);
    state.bits.inputs = read8(MCP23x17Registers::OLATB);
    return state.full;
}
void
GALInterface::configureIOPins(uint8_t pattern) noexcept {
    // invert the bits since we want inputs _TO_ the GAL being an output and 
    // outputs _FROM_ the gal being inputs to the chip
    write8(MCP23x17Registers::IODIRA, static_cast<uint8_t>(pattern));
}
void
GALInterface::setInput(int k, bool state) noexcept {
    if (auto& pin = GAL16V8[k % 20]; pin.clockPin()) {
        digitalWrite(clk_, state ? HIGH : LOW);
    } else if (pin.inputOnlyPin()) {
        auto readState = read8(MCP23x17Registers::OLATB);
        if (state) {
            readState |= pin.mask();
        } else {
            readState &= static_cast<uint8_t>(~pin.mask());
        }
        write8(MCP23x17Registers::OLATB, readState);
    } else if (pin.outputEnablePin()) {
        digitalWrite(oe_, state ? HIGH : LOW);
    } else if (pin.ioPin()) {
        auto readState = read8(MCP23x17Registers::OLATA);
        if (state) {
            readState |= pin.mask();
        } else {
            readState &= static_cast<byte>(~pin.mask());
        }
        write8(MCP23x17Registers::OLATA, readState);
    } 
}

void
GALInterface::setInputs(uint32_t pattern) noexcept {
    InputState tmp;
    tmp.full = pattern;
    digitalWrite(clk_, tmp.bits.clkState ? HIGH : LOW);
    digitalWrite(oe_, tmp.bits.oeState ? HIGH : LOW);
    write8(MCP23x17Registers::OLATB, tmp.bits.inputs);
    write8(MCP23x17Registers::OLATA, tmp.bits.ioPins);
}
bool
GALInterface::getInputState(const GALPinDescription& pin) const noexcept {
    if (pin.clockPin()) {
        return digitalRead(clk_) == HIGH;
    } else if (pin.inputOnlyPin()) {
        return read8(MCP23x17Registers::OLATB) & pin.mask();
    } else if (pin.outputPin()) {
        return read8(MCP23x17Registers::OLATA) & pin.mask();
    } else if (pin.outputEnablePin()) {
        return digitalRead(oe_) == HIGH;
    } else {
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
    write16(MCP23x17Registers::IODIR, 0x00FF); // PORTB is set to inputs, PORTA are outputs
    write16(MCP23x17Registers::OLAT, 0xFFFF);
    setInputs(0); // set all inputs to low
    // setup the MCP23S17 as needed
}
GALInterface iface(CS, 
        I9, 
        I1_CLK, 
        RESET_IOEXP, 
        IOEXP_INT, 
        IOExpanderAddress::GAL_16V8_Element);
volatile bool sdEnabled = false;
volatile bool rtcEnabled = false;
void read() noexcept;
void eval() noexcept;
void initRTCDateTime() noexcept {
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
}
void
printRTCDateTime() noexcept {
        auto now = rtc.now();
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(' ');
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.println();
}
void 
setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }
    Serial.println(F("GAL Testing Interface"));
    Serial.println(F("(C) 2022 Joshua Scoggins"));
    Serial.println(F("This is open source software! See LICENSE for details"));
    Serial.println();
    SPI.begin();
    iface.begin();
    pinMode(SDSelect, OUTPUT);
    digitalWrite(SDSelect, HIGH);
    Serial.println(F("Checking for an SD Card..."));
    if (!SD.begin(SDSelect)) {
        Serial.println(F("SD CARD NOT AVAILABLE!!!"));
    } else {
        sdEnabled = true;
        Serial.println(F("SD CARD AVAILABLE!"));
        // load the configuration from the SDCard to set everything up?
    }

    Serial.println(F("Checking for RTC..."));
    if (! rtc.begin()) {
        Serial.println(F("NO RTC FOUND!"));
    } else {
        rtcEnabled = true;
        Serial.println(F("RTC FOUND!"));
        if (! rtc.initialized() || rtc.lostPower()) {
            Serial.println(F("RTC not initialized, fixing..."));
            initRTCDateTime();
        }
        rtc.start();
        printRTCDateTime();
    }
    Serial.println();

}

void loop() {
    read();
    eval();
}




class PureWord {
    public:
        virtual ~PureWord() = default;
        [[nodiscard]] virtual bool matches(const String& name) const noexcept = 0;
        virtual bool invoke(const String& match) const noexcept = 0;
        virtual String getName() const noexcept = 0;
};

extern const PureWord* lookupTable[];
bool listWords(const String&) noexcept;
bool displayPinout(const String&) noexcept;
bool displayRegisters(const String&) noexcept;
const PureWord* findWord(const String&) noexcept;
void 
GALInterface::displayRegisters() const noexcept {
    Serial.println(F("GAL Interface Registers"));
    Serial.print(F("IO Pins Configuration: 0b"));
    Serial.println(static_cast<byte>(~getIOPinConfiguration()), BIN);
    Serial.print(F("Input values: 0b"));
    Serial.println(readInputs(), BIN);
    Serial.print(F("Outputs: 0x"));
    Serial.println(readOutputs(), HEX);
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
    NoSDCard,
    CantOpenFile,
    NoRTC,
};
extern String inputString;
extern bool stringComplete;
extern unsigned int position;
extern ErrorCodes errorMessage;
void
read() noexcept {
    while (Serial.available()) {
        char inChar = static_cast<char>(Serial.read());
        if (inChar == 0x08) {
            if (inputString.length() > 0) {
                inputString.remove(inputString.length() - 1, 1);
            }
            Serial.print(inChar);
            Serial.print(' ');
            Serial.print(inChar);
        } else {
            inputString += inChar;
            Serial.print(inChar);
        }
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
template<typename T> struct TreatAs { };
using TreatAsBoolean = TreatAs<bool>;
bool pushItemOntoStack(int32_t value) noexcept;
bool pushItemOntoStack(bool value, TreatAsBoolean) noexcept { return pushItemOntoStack(value ? 0xFFFF'FFFF : 0); }
bool pushItemOntoStack(volatile bool value, TreatAs<volatile bool>) noexcept { return pushItemOntoStack(static_cast<bool>(value), TreatAsBoolean{}); }
bool pushItemOntoStack(int value, TreatAs<int>) noexcept { return pushItemOntoStack(value); }
template<typename T>
bool pushItemOntoStack(T value, TreatAs<T>) noexcept {
    return pushItemOntoStack(static_cast<int32_t>(value));
}
bool expectedNumberOfItemsOnStack(byte numberOfItems) noexcept;
bool popItemOffStack(int32_t& item) noexcept;
bool dropTopOfStack() noexcept;
bool dropTopOfStack(const String&) noexcept { return dropTopOfStack(); }
bool stackEmpty() noexcept;
bool stackFull() noexcept;
void clearStack() noexcept;
void clearState() noexcept;
bool swapTopTwoStackElements() noexcept;
uint32_t stackCapacity() noexcept;
uint32_t numberOfItemsOnStack() noexcept;
void handleError(bool errorState) noexcept;
bool duplicateTopOfStack() noexcept;
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
        case ErrorCodes::NoSDCard:
            Serial.println(F("no sd card inserted"));
            break;
        case ErrorCodes::CantOpenFile:
            Serial.println(F("Cannot open file for writing!"));
            break;
        case ErrorCodes::NoRTC:
            Serial.println(F("no rtc active!"));
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
clearState() noexcept {
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


#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_LEONARDO)
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
class InvokeOnPrefixMatchWord : public PureWord {
    public:
        InvokeOnPrefixMatchWord(const __FlashStringHelper* name, const __FlashStringHelper* prefix) : name_(name), prefix_(prefix) { }
        ~InvokeOnPrefixMatchWord() override = default;
        bool matches(const String& message) const noexcept override;
        bool invoke(const String& match) const noexcept override;
        String getName() const noexcept override { return name_; }
        String getPrefix() const noexcept { return prefix_; }
    protected:
        virtual bool invoke0(const String& substringMatch) const noexcept = 0;
    private:
        const __FlashStringHelper* name_;
        const __FlashStringHelper* prefix_;
};
bool
InvokeOnPrefixMatchWord::matches(const String& word) const noexcept {
    if (String prefix(getPrefix()); prefix.length() == 0) {
        return true;
    } else {
        return word.startsWith(prefix);
    }
}
bool
InvokeOnPrefixMatchWord::invoke(const String& match) const noexcept {
    // if we got here then it is safe to strip the prefix off
    auto substring = match.substring(getPrefix().length());
    substring.trim();
    return invoke0(substring);
}
class ParseNumberAndPushOntoStack : public InvokeOnPrefixMatchWord {
    public:
        ParseNumberAndPushOntoStack(const __FlashStringHelper* name, const __FlashStringHelper* prefix) : InvokeOnPrefixMatchWord(name, prefix) { }
        ~ParseNumberAndPushOntoStack() override = default;
    protected:
        bool invoke0(const String& substringMatch) const noexcept override;
        virtual bool parse(const String& theValue, int32_t& number) const noexcept = 0;
};

bool
ParseNumberAndPushOntoStack::invoke0(const String& match) const noexcept {
    int32_t value = 0;
    if (parse(match, value)) {
        return pushItemOntoStack(value);
    } else {
        return false;
    }
}

class NumericBaseCapture : public ParseNumberAndPushOntoStack {
    public:
        NumericBaseCapture(const __FlashStringHelper* name, const __FlashStringHelper* prefix, int numericBase) : ParseNumberAndPushOntoStack(name, prefix), base_(numericBase) { }
    protected:
        bool parse(const String& theValue, int32_t& number) const noexcept override;
    private:
        int base_;

};
bool
NumericBaseCapture::parse(const String& theValue, int32_t& result) const noexcept {
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

bool popAndPrintStackTop(const String&) noexcept;
bool printStackContents(const String&) noexcept;
bool addTwoNumbers(const String&) noexcept;
bool subtractTwoNumbers(const String&) noexcept;
bool multiplyTwoNumbers(const String&) noexcept;
bool divideTwoNumbers(const String&) noexcept;
bool moduloTwoNumbers(const String&) noexcept;
bool orTwoNumbers(const String&) noexcept;
bool andTwoNumbers(const String&) noexcept;
bool xorTwoNumbers(const String&) noexcept;
bool depth(const String&) noexcept;
bool duplicateTop(const String&) noexcept;
bool setIOPinMode(const String&) noexcept;
bool setInputPinValue(const String&) noexcept;
bool swapStackElements(const String&) noexcept;
bool runThroughAllPermutations(const String&) noexcept;
bool twoNumbersEqual(const String&) noexcept;
bool twoNumbersNotEqual(const String&) noexcept;
bool topGreaterThanLower(const String&) noexcept;
bool topLessThanLower(const String&) noexcept;
bool topGreaterThanOrEqualLower(const String&) noexcept;
bool topLessThanOrEqualLower(const String&) noexcept;
//bool extractBit(const String&) noexcept;
class PureLambdaWord : public PureWord {
    public:
        using FunctionBody = bool(*)(const String&);
        PureLambdaWord(const __FlashStringHelper* name, FunctionBody value) : name_(name), value_(value) { }
        [[nodiscard]] bool matches(const String& name) const noexcept override {
            // unpack when we need it
            return name == name_;
        }
        String getName() const noexcept override { return name_; }
        bool invoke(const String& str) const noexcept override { return value_(str); }
    private:
        const __FlashStringHelper* name_;
        FunctionBody value_;
};

#define X(name, str, fn) \
    PROGMEM const char name ## _String [] = str ; \
    const PureLambdaWord name(reinterpret_cast<const __FlashStringHelper*>(name ## _String), fn) 
#define Y(name, str, value) \
    bool name ## _Function (const String&) noexcept { return pushItemOntoStack(value, TreatAs<decltype(value)> {}); } \
    X(name, str, name ## _Function )
#define Z(name, str, pfix, base) \
    PROGMEM const char name ## _String0 [] = str; \
    PROGMEM const char name ## _String1 [] = pfix; \
    const NumericBaseCapture name ( \
            reinterpret_cast<const __FlashStringHelper*>(name ## _String0), \
            reinterpret_cast<const __FlashStringHelper*>(name ## _String1), \
            base)
X(words, "words", listWords);
X(pinsOp, "pins", displayPinout);
X(statusOp, "status", displayRegisters);
X(doPermutationsOp, "do-permutations", runThroughAllPermutations);
X(inputPin, "input-pin", [](const String& theStr) noexcept { return pushItemOntoStack(INPUT, TreatAs<decltype(INPUT)>{}) && setIOPinMode(theStr); });
X(outputPin, "output-pin", [](const String& theStr) noexcept { return pushItemOntoStack(OUTPUT, TreatAs<decltype(OUTPUT)> {}) && setIOPinMode(theStr); });
X(inputLow, "set-low", [](const String& theStr) noexcept { return pushItemOntoStack(LOW, TreatAs<decltype(LOW)>{}) && setInputPinValue(theStr); });
X(inputHigh, "set-high", [](const String& theStr) noexcept { return pushItemOntoStack(HIGH, TreatAs<decltype(HIGH)> { }) && setInputPinValue(theStr); });
X(dateTimeNow, "now", [](const String&) noexcept { printRTCDateTime(); return true; });
X(ioDirection, "io-dir", [](const String&) noexcept {
        if (expectedNumberOfItemsOnStack(1)) {
            int32_t value;
            popItemOffStack(value);
            iface.configureIOPins(static_cast<uint8_t>(value));
            return true;
        } else {
            return false;
        }
    });
X(assignAllInputs, "all-inputs", [](const String&) noexcept {
        if (expectedNumberOfItemsOnStack(1)) {
            int32_t value;
            popItemOffStack(value);
            iface.setInputs(value);
            return true;
        } else {
            return false;
        }
        });
Y(Pin_I1,  "P1",  GAL16V8[0].zeroIndex()); 
Y(Pin_I2,  "P2",  GAL16V8[1].zeroIndex());
Y(Pin_I3,  "P3",  GAL16V8[2].zeroIndex());
Y(Pin_I4,  "P4",  GAL16V8[3].zeroIndex());
Y(Pin_I5,  "P5",  GAL16V8[4].zeroIndex());
Y(Pin_I6,  "P6",  GAL16V8[5].zeroIndex());
Y(Pin_I7,  "P7",  GAL16V8[6].zeroIndex());
Y(Pin_I8,  "P8",  GAL16V8[7].zeroIndex());
Y(Pin_I9,  "P9",  GAL16V8[8].zeroIndex());
Y(Pin_I10, "P11", GAL16V8[10].zeroIndex()); 
Y(Pin_IO8, "P12", GAL16V8[11].zeroIndex());
Y(Pin_IO7, "P13", GAL16V8[12].zeroIndex());
Y(Pin_IO6, "P14", GAL16V8[13].zeroIndex());
Y(Pin_IO5, "P15", GAL16V8[14].zeroIndex());
Y(Pin_IO4, "P16", GAL16V8[15].zeroIndex());
Y(Pin_IO3, "P17", GAL16V8[16].zeroIndex());
Y(Pin_IO2, "P18", GAL16V8[17].zeroIndex());
Y(Pin_IO1, "P19", GAL16V8[18].zeroIndex());
Z(hexParse, "hex number parse", "0x", 16);
Z(binaryParse, "binary number parse", "0b", 2);
Z(fallback, "fallback numeric conversion (no prefix)", "", 0);
#undef Z
#undef Y
#undef X
const PureWord* lookupTable[] = { 
    &words,
    &inputPin,
    &outputPin,
    &inputLow,
    &inputHigh,
    &pinsOp,
    &statusOp,
    &doPermutationsOp,
    &dateTimeNow,
    // constants
    &Pin_I1, 
    &Pin_I2,
    &Pin_I3,
    &Pin_I4,
    &Pin_I5,
    &Pin_I6,
    &Pin_I7,
    &Pin_I8,
    &Pin_I9,
    &Pin_I10, 
    &Pin_IO8,
    &Pin_IO7,
    &Pin_IO6,
    &Pin_IO5,
    &Pin_IO4,
    &Pin_IO3,
    &Pin_IO2,
    &Pin_IO1,
    &ioDirection,
    &assignAllInputs,
    &hexParse,
    &binaryParse,
    // must come last
    &fallback,
};
constexpr auto lookupTableCount = sizeof(lookupTable)/sizeof(const PureWord*);

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
unsigned int position = 0;
ErrorCodes errorMessage = ErrorCodes::None;
constexpr auto numStackElements = 8;
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
setIOPinMode(const String&) noexcept {
    if (numberOfItemsOnStack() < 2) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    int32_t mode, targetPin;
    popItemOffStack(mode);
    // assume that targetPin is actually subtracted by one
    popItemOffStack(targetPin);
    iface.configureIOPin(GAL16V8[targetPin%20], mode);
    return true;
}
bool
setInputPinValue(const String&) noexcept {
    if (expectedNumberOfItemsOnStack(2)) {
        int32_t value, targetPin;
        popItemOffStack(value);
        popItemOffStack(targetPin);
        iface.setInput(targetPin, value != 0);
        return true;
    }
    return false;
}
bool 
expectedNumberOfItemsOnStack(byte numberOfItems) noexcept {
    if (numberOfItemsOnStack() < numberOfItems) {
        errorMessage = ErrorCodes::NotEnoughStackElements;
        return false;
    }
    return true;
}

bool
depth(const String&) noexcept {
    return pushItemOntoStack(numberOfItemsOnStack());
}
bool 
swapStackElements(const String&) noexcept {
    return swapTopTwoStackElements();
}

bool 
swapTopTwoStackElements() noexcept {
    if (expectedNumberOfItemsOnStack(2)) {
        auto top = theStack[stackPosition];
        auto next = theStack[stackPosition + 1];
        theStack[stackPosition] = next;
        theStack[stackPosition + 1] = top;
        return true;
    }
    return false;
}

bool
duplicateTopOfStack() noexcept {
    return expectedNumberOfItemsOnStack(1) && pushItemOntoStack(theStack[stackPosition]);
}

bool
runThroughAllPermutations(const String&) noexcept {
    if (sdEnabled) {
        // okay so we assume that at this point, we have already defined the
        // apropriate pinout
        File file = SD.open("output.txt", FILE_WRITE);
        if (file) {
            // the simplest thing to do now would be to just run through all
            // 18-bit permutations and capture the result
            for (int32_t pattern = 0; pattern < 0b11'1111'1111'1111'1111; ++pattern) {
                iface.setInputs(pattern);
                auto result = iface.readOutputs();
                file.print(F("0b"));
                file.print(pattern, BIN);
                file.print(F(" => 0b"));
                file.println(static_cast<uint16_t>(result), BIN);
            }
        } else {
            errorMessage = ErrorCodes::CantOpenFile;
        }
        file.close();
    } else {
        errorMessage = ErrorCodes::NoSDCard;
    }
    return sdEnabled;
}

bool
listWords(const String&) noexcept {
    for (auto* word : lookupTable) {
        if (word) {
            Serial.println(word->getName());
        }
    }
    return true;
}
bool
displayPinout(const String&) noexcept {
    iface.printPinStates(Serial);
    return true;
}
bool
displayRegisters(const String& theString) noexcept {
    iface.displayRegisters();
    Serial.println(F("STACK CONTENTS: "));
    return printStackContents(theString);
}


const PureWord* findWord(const String& word) noexcept {
    for (auto* theWord: lookupTable) {
        if (theWord && theWord->matches(word)) {
            return theWord;
        }
    }
    return nullptr;
}
