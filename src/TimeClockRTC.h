#include "DeviceRTC.h"

namespace OpenKNX
{
    namespace Time
    {
#ifndef ARDUINO_ARCH_SAMD
        class TimeClockRTC : public TimeClock
        {
            friend class DeviceRTC;

          public:
            inline void setup() override
            {
                if (!openknxRTCModule.isInitialized())
                {
                    openknxRTCModule.setI2CSettings(OKNXHW_DEVICE_RTC_I2C_INST, OKNXHW_DEVICE_RTC_I2C_SCL, OKNXHW_DEVICE_RTC_I2C_SDA,
                                                    OKNXHW_DEVICE_RTC_I2C_ADDRESS,
                                                    OKNXHW_DEVICE_RTC_EEPROM_I2C_ADDRESS, OKNXHW_REG2_HWRTC_I2C_EEPROM_SIZE);
                    openknxRTCModule.init();
                    if (openknxRTCModule.isInitialized()) _TimeClockRTC_isRunning = true;
                }
                else
                    _TimeClockRTC_isRunning = true;
            }
            inline void loop() override {}
            inline void setTime(time_t epoch, unsigned long millisReceivedTimestamp) override
            {
                if (_TimeClockRTC_isRunning)
                {
                    openknxRTCModule.adjust(::DateTime(epoch));
                    struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
                    settimeofday(&tv, nullptr);
                }
            }
            inline time_t getTime() override
            {
                if (_TimeClockRTC_isRunning) return openknxRTCModule.getTime();
                else
                    return 0;
            }
            inline bool isRunning() override
            {
                return _TimeClockRTC_isRunning;
            }

          private:
            bool _TimeClockRTC_isRunning = false;
        }; // Class TimeClockRTC
#endif // ARDUINO_ARCH_SAMD
    } // namespace Time
} // namespace OpenKNX