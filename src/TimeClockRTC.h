#include "DeviceRTC.h"
namespace OpenKNX
{
    namespace Time
    {
        class TimeClockRTC : public TimeClock
        {
            friend class DeviceRTC;
          public:
            inline void setup() override
            {
                if (!openknxRTCModule.isInitialized())
                {
                    openknxRTCModule.setI2CSettings(OKNXHW_DEVICE_RTC_I2C_0_1, OKNXHW_DEVICE_RTC_I2C_SCL, OKNXHW_DEVICE_RTC_I2C_SDA,
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
                if (_TimeClockRTC_isRunning) openknxRTCModule.adjust(::DateTime(epoch));
            }
            inline time_t getTime() override
            {
                if (_TimeClockRTC_isRunning)
                {
                    time_t now = openknxRTCModule.getTime();
                    return now;
                }
                return 0;
            }
            inline bool isRunning() override
            {
                return _TimeClockRTC_isRunning;
            }

          private:
            bool _TimeClockRTC_isRunning = false;
        }; // Class TimeClockRTC
    } // namespace Time
} // namespace OpenKNX