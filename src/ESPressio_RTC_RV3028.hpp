#pragma once

#include <cstdint>

#include <ESPressio_RTC.hpp>

namespace ESPressio::RTC::RV3028 {

inline constexpr std::uint8_t I2CAddress = 0x52U;
struct Origin final : ESPressio::Platform::Backend {};

enum class ClockOutputFrequency : std::uint8_t {
    Hz32768 = 0,
    Hz8192 = 1,
    Hz1024 = 2,
    Hz64 = 3,
    Hz32 = 4,
    Hz1 = 5,
    TimerInterrupt = 6,
    Disabled = 7
};

template <typename TBus>
class Device final
    : public RTC::DeviceProviderDeclaration<
          Origin,
          ESPressio::Platform::CapabilitySet<
              RTC::Capability::UnixTime,
              RTC::Capability::ClockOutput>> {
public:
    explicit constexpr Device(TBus& bus) noexcept : registers_(bus) {}

    Result Read(Reading& reading) noexcept {
        std::uint8_t data[7]{};
        auto result = registers_.Read(0x00U, data, sizeof(data));
        if (result != Result::Ok) return result;

        std::uint8_t control2{};
        result = registers_.ReadByte(0x10U, control2);
        if (result != Result::Ok) return result;

        DateTime value{};
        value.Second = FromBcd(static_cast<std::uint8_t>(data[0] & 0x7FU));
        value.Minute = FromBcd(static_cast<std::uint8_t>(data[1] & 0x7FU));
        value.Hour = DecodeHour(data[2], (control2 & 0x02U) != 0U);
        value.Weekday = static_cast<std::uint8_t>(data[3] & 0x07U);
        value.Day = FromBcd(static_cast<std::uint8_t>(data[4] & 0x3FU));
        value.Month = FromBcd(static_cast<std::uint8_t>(data[5] & 0x1FU));
        value.Year = static_cast<std::uint16_t>(2000U + FromBcd(data[6]));
        if (!RTC::IsValid(value)) return Result::InvalidDateTime;

        std::uint8_t status{};
        result = registers_.ReadByte(0x0EU, status);
        if (result != Result::Ok) return result;
        reading.Value = value;
        reading.Validity = (status & 0x01U) != 0U
                               ? TimeValidity::PowerOnReset
                               : TimeValidity::Valid;
        return Result::Ok;
    }

    Result Write(const DateTime& value) noexcept {
        if (!RTC::IsValid(value) || value.Year < 2000U || value.Year > 2099U)
            return Result::InvalidDateTime;

        // Force the device into 24-hour mode before writing the canonical value.
        auto result = registers_.UpdateBits(0x10U, 0x02U, 0x00U);
        if (result != Result::Ok) return result;

        std::uint8_t data[7]{
            ToBcd(value.Second),
            ToBcd(value.Minute),
            ToBcd(value.Hour),
            static_cast<std::uint8_t>(value.Weekday & 0x07U),
            ToBcd(value.Day),
            ToBcd(value.Month),
            ToBcd(static_cast<std::uint8_t>(value.Year % 100U))
        };
        result = registers_.Write(0x00U, data, sizeof(data));
        if (result != Result::Ok) return result;
        return ClearPowerOnResetFlag();
    }

    Result ClearPowerOnResetFlag() noexcept {
        return registers_.UpdateBits(0x0EU, 0x01U, 0x00U);
    }

    Result ReadUnixTime(std::uint32_t& value) noexcept {
        // The RV-3028 Unix registers are readable but not blocked during a read.
        // Micro Crystal recommends reading all four bytes twice and accepting a
        // consistent pair. Use at most three samples so the operation remains
        // deterministically bounded even when a 1 Hz increment lands mid-read.
        std::uint32_t previous{};
        auto result = ReadUnixTimeSample(previous);
        if (result != Result::Ok) return result;

        for (std::uint8_t attempt = 0U; attempt < 2U; ++attempt) {
            std::uint32_t current{};
            result = ReadUnixTimeSample(current);
            if (result != Result::Ok) return result;
            if (current == previous) {
                value = current;
                return Result::Ok;
            }
            previous = current;
        }
        return Result::DeviceBusy;
    }

    Result WriteUnixTime(std::uint32_t value) noexcept {
        // Reset the 1 Hz prescaler first, as recommended by Micro Crystal, so
        // the four-register write cannot discard a pending Unix-time tick.
        auto result = registers_.UpdateBits(0x10U, 0x01U, 0x01U);
        if (result != Result::Ok) return result;

        std::uint8_t data[4]{
            static_cast<std::uint8_t>(value & 0xFFU),
            static_cast<std::uint8_t>((value >> 8U) & 0xFFU),
            static_cast<std::uint8_t>((value >> 16U) & 0xFFU),
            static_cast<std::uint8_t>((value >> 24U) & 0xFFU)
        };
        return registers_.Write(0x1BU, data, sizeof(data));
    }

    /**
     * Configures the active RAM mirror of the CLKOUT configuration.
     *
     * This intentionally does not commit EEPROM, avoiding write wear and
     * password/EEPROM command side effects. A later refresh/POR may restore the
     * EEPROM-backed configuration.
     */
    Result ConfigureClockOutput(ClockOutputFrequency frequency,
                                bool enabled = true,
                                bool synchronized = true) noexcept {
        std::uint8_t current{};
        auto result = registers_.ReadByte(0x35U, current);
        if (result != Result::Ok) return result;
        current = static_cast<std::uint8_t>(current & 0x08U); // preserve PORIE only
        if (enabled && frequency != ClockOutputFrequency::Disabled)
            current = static_cast<std::uint8_t>(current | 0x80U);
        if (synchronized)
            current = static_cast<std::uint8_t>(current | 0x40U);
        current = static_cast<std::uint8_t>(current | static_cast<std::uint8_t>(frequency));
        return registers_.WriteByte(0x35U, current);
    }

    Result IsEepromBusy(bool& busy) noexcept {
        std::uint8_t status{};
        const auto result = registers_.ReadByte(0x0EU, status);
        busy = (status & 0x80U) != 0U;
        return result;
    }

private:
    Result ReadUnixTimeSample(std::uint32_t& value) noexcept {
        std::uint8_t data[4]{};
        const auto result = registers_.Read(0x1BU, data, sizeof(data));
        if (result != Result::Ok) return result;
        value = static_cast<std::uint32_t>(data[0]) |
                (static_cast<std::uint32_t>(data[1]) << 8U) |
                (static_cast<std::uint32_t>(data[2]) << 16U) |
                (static_cast<std::uint32_t>(data[3]) << 24U);
        return Result::Ok;
    }

    static constexpr std::uint8_t DecodeHour(std::uint8_t value, bool mode12Hour) noexcept {
        if (!mode12Hour)
            return FromBcd(static_cast<std::uint8_t>(value & 0x3FU));
        const auto hour12 = FromBcd(static_cast<std::uint8_t>(value & 0x1FU));
        const bool pm = (value & 0x20U) != 0U;
        return static_cast<std::uint8_t>((hour12 % 12U) + (pm ? 12U : 0U));
    }

    RTC::RegisterDevice<TBus, I2CAddress> registers_;
};

/** ISR-counted Platform Clock aliases for fixed RV-3028 CLKOUT rates.
 * TimerInterrupt mode is intentionally excluded because it is not a fixed-rate
 * oscillator source suitable for a static Platform Clock declaration.
 */
using Clock1Hz = RTC::ExternalTickClock<Origin, 1U>;
using Clock32Hz = RTC::ExternalTickClock<Origin, 32U>;
using Clock64Hz = RTC::ExternalTickClock<Origin, 64U>;
using Clock1024Hz = RTC::ExternalTickClock<Origin, 1024U>;
using Clock8192Hz = RTC::ExternalTickClock<Origin, 8192U>;
using Clock32k = RTC::ExternalTickClock<
    Origin,
    32768U,
    32U,
    ESPressio::Platform::CapabilitySet<
        ESPressio::Platform::Capability::HighResolutionClock>>;

} // namespace ESPressio::RTC::RV3028
