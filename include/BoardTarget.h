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

#ifndef GAL_TESTER_BOARD_TARGET_H__
#define GAL_TESTER_BOARD_TARGET_H__
#include <Arduino.h>
constexpr auto getSDCardPin() noexcept {
#ifdef ARDUINO_METRO_ESP32S2
    return 9;
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
    return SDCARD_SS_PIN;
#else
    return 4;
#endif
}

constexpr auto getI1CLKPin() noexcept {
#ifdef ARDUINO_METRO_ESP32S2
    return 10;
#else
    return 4;
#endif
}
constexpr auto getIOEXPIntPin() noexcept {
#ifdef ARDUINO_METRO_ESP32S2
    return 7;
#else
    return 2;
#endif
}
constexpr auto getCSPin() noexcept { return A0; }
constexpr auto getI9Pin() noexcept { return A1; }
constexpr auto getIOEXPResetPin() noexcept { return A2; }
#endif
