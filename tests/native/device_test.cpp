#include <array>
#include <cassert>
#include <cstdint>
#include "ESPressio_RTC_RV3028.hpp"
using namespace ESPressio;
struct Bus {
    std::array<std::uint8_t,256> r{};
    bool Read(std::uint8_t, std::uint8_t reg, std::uint8_t* d, std::size_t n) noexcept {
        for(std::size_t i=0;i<n;++i) { d[i]=r[reg+i]; }
        return true;
    }
    bool Write(std::uint8_t, std::uint8_t reg, const std::uint8_t* d, std::size_t n) noexcept {
        for(std::size_t i=0;i<n;++i) { r[reg+i]=d[i]; }
        return true;
    }
};
using Device = RTC::RV3028::Device<Bus>;
static_assert(RTC::IsRealTimeClockV<Device>);
static_assert(Platform::Clock::IsClockSourceV<RTC::RV3028::Clock32k>);
static_assert(Platform::Clock::FrequencyHz<RTC::RV3028::Clock64Hz> == 64U);
static_assert(Platform::Clock::FrequencyHz<RTC::RV3028::Clock8192Hz> == 8192U);
int main(){
    Bus b; Device rtc(b);
    RTC::DateTime in{2026,9,12,16,45,30,6};
    assert(rtc.Write(in)==RTC::Result::Ok);
    RTC::Reading out{}; assert(rtc.Read(out)==RTC::Result::Ok); assert(out.Value.Year==2026);
    b.r[0x0E] |= 0x01; assert(rtc.Read(out)==RTC::Result::Ok); assert(out.Validity==RTC::TimeValidity::PowerOnReset);
    assert(rtc.ClearPowerOnResetFlag()==RTC::Result::Ok);
    assert(rtc.WriteUnixTime(0x12345678U)==RTC::Result::Ok);
    std::uint32_t unixTime{}; assert(rtc.ReadUnixTime(unixTime)==RTC::Result::Ok); assert(unixTime==0x12345678U);
    b.r[0x35]=0x08; assert(rtc.ConfigureClockOutput(RTC::RV3028::ClockOutputFrequency::Hz32768)==RTC::Result::Ok);
    assert((b.r[0x35]&0xC7U)==0xC0U);
}
