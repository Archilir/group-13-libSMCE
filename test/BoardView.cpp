/*
 *  test/BoardView.cpp
 *  Copyright 2020-2021 ItJustWorksTM
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <catch2/catch_test_macros.hpp>
#include "SMCE/Board.hpp"
#include "SMCE/BoardConf.hpp"
#include "SMCE/BoardView.hpp"
#include "SMCE/Toolchain.hpp"
#include "defs.hpp"

using namespace std::literals;

TEST_CASE("BoardView GPIO", "[BoardView]") {
    smce::Toolchain tc{SMCE_PATH};
    REQUIRE(!tc.check_suitable_environment());
    smce::Sketch sk{SKETCHES_PATH "pins", {.fqbn = "arduino:avr:nano"}};
    const auto ec = tc.compile(sk);
    if (ec)
        std::cerr << tc.build_log().second;
    REQUIRE_FALSE(ec);
    smce::Board br{};
    // clang-format off
    smce::BoardConfig bc{
        /* .pins = */{0, 2},
        /* .gpio_drivers = */{
            smce::BoardConfig::GpioDrivers{
                0,
                smce::BoardConfig::GpioDrivers::DigitalDriver{true, false},
                smce::BoardConfig::GpioDrivers::AnalogDriver{true, false}
            },
            smce::BoardConfig::GpioDrivers{
                2,
                smce::BoardConfig::GpioDrivers::DigitalDriver{false, true},
                smce::BoardConfig::GpioDrivers::AnalogDriver{false, true}
            },
        }
    };
    // clang-format on
    REQUIRE(br.configure(std::move(bc)));
    REQUIRE(br.attach_sketch(sk));
    REQUIRE(br.start());
    auto bv = br.view();
    REQUIRE(bv.valid());
    auto pin0 = bv.pins[0];
    REQUIRE(pin0.exists());
    auto pin0d = pin0.digital();
    REQUIRE(pin0d.exists());
    REQUIRE(pin0d.can_read());
    REQUIRE_FALSE(pin0d.can_write());
    auto pin0a = pin0.analog();
    REQUIRE(pin0a.exists());
    REQUIRE(pin0a.can_read());
    REQUIRE_FALSE(pin0a.can_write());
    auto pin1 = bv.pins[1];
    REQUIRE_FALSE(pin1.exists());
    auto pin2 = bv.pins[2];
    REQUIRE(pin2.exists());
    auto pin2d = pin2.digital();
    REQUIRE(pin2d.exists());
    REQUIRE_FALSE(pin2d.can_read());
    REQUIRE(pin2d.can_write());
    auto pin2a = pin2.analog();
    REQUIRE(pin2a.exists());
    REQUIRE_FALSE(pin2a.can_read());
    REQUIRE(pin2a.can_write());
    std::this_thread::sleep_for(1ms);

    pin0d.write(false);
    test_pin_delayable(pin2d, true, 16384, 1ms);
    pin0d.write(true);
    test_pin_delayable(pin2d, false, 16384, 1ms);
    REQUIRE(br.stop());
}

TEST_CASE("BoardView UART", "[BoardView]") {
    smce::Toolchain tc{SMCE_PATH};
    REQUIRE(!tc.check_suitable_environment());
    smce::Sketch sk{SKETCHES_PATH "uart", {.fqbn = "arduino:avr:nano"}};
    const auto ec = tc.compile(sk);
    if (ec)
        std::cerr << tc.build_log().second;
    REQUIRE_FALSE(ec);
    smce::Board br{};
    REQUIRE(br.configure({.uart_channels = {{}}}));
    REQUIRE(br.attach_sketch(sk));
    REQUIRE(br.start());
    auto bv = br.view();
    REQUIRE(bv.valid());
    auto uart0 = bv.uart_channels[0];
    REQUIRE(uart0.exists());
    REQUIRE(uart0.rx().exists());
    REQUIRE(uart0.tx().exists());
    auto uart1 = bv.uart_channels[1];
    REQUIRE_FALSE(uart1.exists());
    REQUIRE_FALSE(uart1.rx().exists());
    REQUIRE_FALSE(uart1.tx().exists());
    std::this_thread::sleep_for(1ms);

    std::array out = {'H', 'E', 'L', 'L', 'O', ' ', 'U', 'A', 'R', 'T', '\0'};
    std::array<char, out.size()> in{};
    REQUIRE(uart0.rx().write(out) == out.size());
    int ticks = 16'000;
    do {
        if (ticks-- == 0)
            FAIL("Timed out");
        std::this_thread::sleep_for(1ms);
    } while (uart0.tx().size() != in.size());
    REQUIRE(uart0.tx().front() == 'H');
    REQUIRE(uart0.tx().read(in) == in.size());
    REQUIRE(uart0.tx().front() == '\0');
    REQUIRE(uart0.tx().size() == 0);
    REQUIRE(in == out);

#if !MSVC_DEBUG
    std::reverse(out.begin(), out.end());
    REQUIRE(uart0.rx().write(out) == out.size());
    ticks = 16'000;
    do {
        if (ticks-- == 0)
            FAIL("Timed out");
        std::this_thread::sleep_for(1ms);
    } while (uart0.tx().size() != in.size());
    REQUIRE(uart0.tx().read(in) == in.size());
    REQUIRE(uart0.tx().size() == 0);
    REQUIRE(in == out);
#endif

    REQUIRE(br.stop());
}

constexpr auto div_ceil(std::size_t lhs, std::size_t rhs) { return lhs / rhs + !!(lhs % rhs); }

constexpr std::byte operator""_b(char c) noexcept { return static_cast<std::byte>(c); }

constexpr std::size_t bpp_444 = 4 + 4 + 4;
constexpr std::size_t bpp_888 = 8 + 8 + 8;

TEST_CASE("BoardView RGB444 cvt", "[BoardView]") {
    smce::Toolchain tc{SMCE_PATH};
    REQUIRE(!tc.check_suitable_environment());
    smce::Sketch sk{SKETCHES_PATH "noop", {.fqbn = "arduino:avr:nano"}};
    const auto ec = tc.compile(sk);
    if (ec)
        std::cerr << tc.build_log().second;
    REQUIRE_FALSE(ec);
    smce::Board br{};
    REQUIRE(br.configure({.frame_buffers = {{}}}));
    REQUIRE(br.attach_sketch(sk));
    REQUIRE(br.prepare());
    auto bv = br.view();
    REQUIRE(bv.valid());
    REQUIRE(br.start());
    REQUIRE(br.suspend());
    auto fb = bv.frame_buffers[0];
    REQUIRE(fb.exists());

    {
        constexpr std::size_t height = 1;
        constexpr std::size_t width = 1;
                            
        /*
        16 bits pre conversion
        24 bits after conversion
        Converting from:
        GGGG BBBB 0000 RRRR
        To:
        RRRRRRRR GGGGGGGG BBBBBBBB
        */
        constexpr std::array in = {'\xBC'_b, '\x0A'_b};
                                    /*
                                    input represented in binary is:
                                    hexa BC = binary 1011 1100
                                    hexa 0A = binary 0000 1010
                                    ->
                                    G=1011 
                                    B=1100   
                                    R=1010 
                                    */
        constexpr std::array expected_out = {'\xA0'_b, '\xB0'_b, '\xC0'_b};
                                    /*
                                    expected output would therefore be same binary 
                                    number for corresponding colour, but adding 0000
                                    to the tail. Resulting in:
                                    R binary 1010 0000 = hexa A0
                                    G binary 1011 0000 = hexa B0
                                    B binary 1100 0000 = hexa C0
                                    */

        static_assert(in.size() == expected_out.size() / 3 * 2);
                                    /*
                                    Output size becomes 3/2 larger than input size
                                    Since conversion adds 3/2 more pixels
                                    */

        fb.set_height(height);
        fb.set_width(width);
        REQUIRE(fb.write_rgb444(in));

        std::array<std::byte, std::size(expected_out)> out;
        REQUIRE(fb.read_rgb888(out));
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 2;
        constexpr std::size_t width = 2;
                                /*
                                
                                Test case follows same logic in conversion as above but with larger test size,
                                Converting from:
                                GGGG BBBB 0000 RRRR
                                To:
                                RRRRRRRR GGGGGGGG BBBBBBBB
                                */
                               
        constexpr std::array in = {'\x23'_b, '\xF1'_b, '\x56'_b, '\xF4'_b, '\x89'_b, '\xF7'_b, '\xBC'_b, '\xFA'_b};
        
                                    /*
                                    input represented in binary is:
                                    hexa 23 = binary 0010 0011
                                    hexa F1 = binary 1111 0001 
                                    ->
                                    G=0010 
                                    B=0011
                                    0=1111
                                    R=0001 
                                    
                                    This process continues for the entire array.
                                    */
                                            
        constexpr std::array expected_out = {'\x10'_b, '\x20'_b, '\x30'_b, '\x40'_b, '\x50'_b, '\x60'_b,
                                             '\x70'_b, '\x80'_b, '\x90'_b, '\xA0'_b, '\xB0'_b, '\xC0'_b};
                                    
                                    /*
                                    R binary 0001 0000 = hexa 10
                                    G binary 0010 0000 = hexa 20
                                    B binary 0011 0000 = hexa 30
                                    
                                    This conversion continues for the entire array.
                                    */

        static_assert(in.size() == expected_out.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        fb.write_rgb444(in);

        std::array<std::byte, std::size(expected_out)> out;
        fb.read_rgb888(out);
        REQUIRE(out == expected_out);  
        /*
        We check whether the conversion from RGB444 to RGB888 equals the expected output. If implementation of RGB444 is correct
        the conversion should yield the expected output according to the explanation above.
        */
    }

    {
        constexpr std::size_t height = 1;
        constexpr std::size_t width = 1;
        
         /*
        Converting from:
        RRRRRRRR GGGGGGGG BBBBBBBB
        To:
        GGGG BBBB 0000 RRRR
        */

        constexpr std::array in = {'\xAD'_b, '\xBE'_b, '\xCF'_b};
        
                                    /*
                                    input represented in binary is:
                                    R hexa AD = binary 1010 1101
                                    G hexa BE = binary 1011 1110
                                    B hexa CF = binary 1100 1111       
                                    */
        
        constexpr std::array expected_out = {'\xBC'_b, '\x0A'_b};
        
                                    /*
                                    Expected output would therefore be: 
                                    G binary 1011 = hexa B
                                    B binary 1100 = hexa C
                                    0 binary 0000 = hexa 0
                                    R binary 1010 = hexa A
                                    
                                    which equals hexa BC, 0A
                                    */
        
        static_assert(expected_out.size() == in.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        REQUIRE(fb.write_rgb888(in));

        std::array<std::byte, std::size(expected_out)> out;
        REQUIRE(fb.read_rgb444(out));
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 2;
        constexpr std::size_t width = 2;

        constexpr std::array in = {'\x1A'_b, '\x2B'_b, '\x3C'_b, '\x4D'_b, '\x5E'_b, '\x6F'_b,
                                   '\x7A'_b, '\x8B'_b, '\x9C'_b, '\xAD'_b, '\xBE'_b, '\xCF'_b};
        
                                    /*
                                    input represented in binary is:
                                    R hexa 1A = binary 0001 1010
                                    G hexa 2B = binary 0010 1011
                                    B hexa 3C = binary 0011 1100       
                                    */
                                    
        constexpr std::array expected_out = {'\x23'_b, '\x01'_b, '\x56'_b, '\x04'_b,
                                             '\x89'_b, '\x07'_b, '\xBC'_b, '\x0A'_b};
        
        
                                    /*
                                    Expected output would therefore be: 
                                    G binary 0010 = hexa 2
                                    B binary 0011 = hexa 3
                                    0 binary 0000 = hexa 0
                                    R binary 0001 = hexa 1
                                    
                                    which equals hexa 23, 01
                                    */
        
        
        static_assert(expected_out.size() == in.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        fb.write_rgb888(in);

        std::array<std::byte, std::size(expected_out)> out;
        fb.read_rgb444(out);
        REQUIRE(out == expected_out);
        /*
        We check whether the conversion from RGB888 to RGB444 equals the expected output. If implementation of RGB444 is correct
        the conversion should yield the expected output according to the explanation above.
        */
    }

    REQUIRE(br.resume());
    REQUIRE(br.stop());
}

TEST_CASE("BoardView RGB565 cvt", "[BoardView]") {
    smce::Toolchain tc{SMCE_PATH};
    REQUIRE(!tc.check_suitable_environment());
    smce::Sketch sk{SKETCHES_PATH "noop", {.fqbn = "arduino:avr:nano"}};
    const auto ec = tc.compile(sk);
    if (ec)
        std::cerr << tc.build_log().second;
    REQUIRE_FALSE(ec);
    smce::Board br{};
    REQUIRE(br.configure({.frame_buffers = {{}}}));
    REQUIRE(br.attach_sketch(sk));
    REQUIRE(br.start());
    auto bv = br.view();
    REQUIRE(bv.valid());
    REQUIRE(br.suspend());
    auto fb = bv.frame_buffers[0];
    REQUIRE(fb.exists());

    {

        //used framework from RGB444 test, increasing the excepted output pixel accordingly
        constexpr std::size_t height = 1;
        constexpr std::size_t width = 1;

        /*
        16 bits pre conversion
        24 bits after conversion
        Converting from:
        RRRRR GGGGGG BBBBB 
        To:
        RRRRRRRR GGGGGGGG BBBBBBBB
        */
        
        constexpr std::array in = {'\xBC'_b, '\x0A'_b};
        
        
                                    /*
                                    input represented in binary is:
                                    hexa BC = binary 1011 1100
                                    hexa 0A = binary 0000 1010
                                    ->
                                    R=1011 1 
                                    G=1000 00  
                                    B=0101 0
                                    */
      
        constexpr std::array expected_out = {'\xB8'_b, '\x80'_b, '\x50'_b};
                                    
                                    /*
                                    Expected output would therefore be: 
                                    All colour bits are increased to 8 bits to match the 24 bits we 
                                    convert. Converts from 16 bits to 24 bits.
                                    Furthermore, using little endian, therefore adding zeros to the right side.
                                    
                                    R binary 1011 1000 = hexa B8
                                    G binary 1000 0000 = hexa 80
                                    B binary 0101 0000 = hexa 50
                                    */
                             
        static_assert(in.size() == expected_out.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        REQUIRE(fb.write_rgb565(in));

        std::array<std::byte, std::size(expected_out)> out;
        REQUIRE(fb.read_rgb888(out));
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 2;
        constexpr std::size_t width = 2;
                                    
                       
        constexpr std::array in = {'\x23'_b, '\xF1'_b, '\x56'_b, '\xF4'_b, '\x89'_b, '\xF7'_b, '\xBC'_b, '\xFA'_b};
                       
 
                                    /*
                                    input represented in binary is:
                                    hexa 23 = binary 0010 0011
                                    hexa F1 = binary 1111 0001
                                    ->
                                    R=0010 0 
                                    G=0111 11  
                                    B=1000 1
                                    */
                                                            
        constexpr std::array expected_out = {'\x20'_b, '\x7C'_b, '\x88'_b, '\x50'_b, '\xDC'_b, '\xA0'_b,
                                             '\x88'_b, '\x3C'_b, '\xB8'_b, '\xB8'_b, '\x9C'_b, '\xD0'_b};
        
                                    /*
                                    Expected output would therefore be: 
                                    All colour bits are increased to 8 bits to match the 24 bits we 
                                    convert. Converts from 16 bits to 24 bits.
                                    Furthermore, using little endian, therefore adding zeros to the right side.
                                    
                                    R binary 0010 0000 = hexa 20
                                    G binary 0111 1100 = hexa 7C
                                    B binary 1000 1000 = hexa 88
                                    */
        static_assert(in.size() == expected_out.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        fb.write_rgb565(in);

        std::array<std::byte, std::size(expected_out)> out;
        fb.read_rgb888(out);
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 1;
        constexpr std::size_t width = 1;

        constexpr std::array in = {'\xAD'_b, '\xBE'_b, '\xCF'_b};
        
                                    /*
                                    input represented in binary is:
                                    hexa AD = binary 1010 1101
                                    hexa BE = binary 1011 1110
                                    hexa CF = binary 1100 1111
                                    ->
                                    R=1010 1101 
                                    G=1011 1110
                                    B=1100 1111
                                    */
        
        constexpr std::array expected_out = {'\xAD'_b, '\xD9'_b};
       
        
                                    /*
                                    R binary 1010 1 
                                    G binary 1011 10 
                                    B binary 1100 1 
                                   
                                    AD,D9
                                    */
        
        static_assert(expected_out.size() == in.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        REQUIRE(fb.write_rgb888(in));

        std::array<std::byte, std::size(expected_out)> out;
        REQUIRE(fb.read_rgb565(out));
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 2;
        constexpr std::size_t width = 2;

        constexpr std::array in = {'\x1A'_b, '\x2B'_b, '\x3C'_b, '\x4D'_b, '\x5E'_b, '\x6F'_b,
                                   '\x7A'_b, '\x8B'_b, '\x9C'_b, '\xAD'_b, '\xBE'_b, '\xCF'_b};
        
        
                                    /*
                                    input represented in binary is:
                                    hexa 1A = binary 0001 1010
                                    hexa 2B = binary 0010 1011
                                    hexa 3C = binary 0011 1100
                                    ->
                                    R=0001 1010 
                                    G=0010 1011
                                    B=0011 1100
                                    */
        
        
                                        
        
        constexpr std::array expected_out = {'\x19'_b, '\x67'_b, '\x4A'_b, '\xCD'_b,
                                             '\x7C'_b, '\x73'_b, '\xAD'_b, '\xD9'_b};
        
                                    /*
                                    R binary 0001 1 
                                    G binary 0010 11 
                                    B binary 0011 1 
                                   
                                    19,67
                                    */
        
        static_assert(expected_out.size() == in.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        fb.write_rgb888(in);

        std::array<std::byte, std::size(expected_out)> out;
        fb.read_rgb565(out);
        REQUIRE(out == expected_out);
    }

    REQUIRE(br.stop());
}
