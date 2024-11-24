#pragma once

#include <OpenKNX.h>
#include <RTClib.h>
#include <Wire.h>

#define DeviceRTC_Display_Name "DeviceRTC" // Display name
#define DeviceRTC_Display_Version "0.0.1"  // Display version

// Uncomment the following lines to enable the respective features
#define ENABLE_TEMPERATURE // Enable temperature reading
#define ENABLE_EEPROM      // Enable external EEPROM
#define ENABLE_ALARMS      // Enable alarms

class i2cRTC
{
  public:
    i2cRTC();  // Constructor
    ~i2cRTC(); // Destructor

    struct RTCSettings
    {
        uint8_t i2cAddress = 0x68;       // Default I2C address for DS3231
        uint8_t i2cAddressEEPROM = 0x57; // Default I2C address for DS3231 EEPROM
        uint16_t i2cEEPROMSize = 0x1000; // Default I2C EEPROM size for DS3231 (4096 bytes (4K), 0x000 to 0xFFF)
        bool bIsi2c1 = false;            // true:i2c1 false:i2c0
        pin_size_t sda = -1;             // SDA pin
        pin_size_t scl = -1;             // SCL pin
    } rtcSettings;

    std::unique_ptr<TwoWire> customI2C; // Custom I2C object

    void setup();                       // Setup method for initialization
    bool initRTC();                     // Initialize the RTC
    bool initRTC(RTCSettings settings); // Initialize the RTC with custom settings
    bool scanI2C(int8_t* devices);      // Scan the I2C bus for devices
};

class DeviceRTC : public OpenKNX::Module
{
  public:
    DeviceRTC();                                                  // Constructor
    void begin();                                                 // Initialize the RTC
    DateTime now();                                               // Get the current time from the RTC
    void adjust(const DateTime& dt);                              // Adjust the RTC to a new time
    inline bool isInitialized() { return _deviceRTCinitialized; } // Check if the RTC is initialized

    void init() override;                                                   // Initialize the module
    void setup(bool configured) override;                                   // Setup the module
    void processInputKo(GroupObject& obj) override;                         // Process input KO
    void showHelp() override;                                               // Show help for console commands
    bool processCommand(const std::string command, bool diagnose) override; // Process console commands

    inline const std::string name() { return DeviceRTC_Display_Name; }       // Library name
    inline const std::string version() { return DeviceRTC_Display_Version; } // Library version

    void setI2CSettings(bool bIsi2c1, uint8_t scl, uint8_t sda, uint8_t address,
                        uint8_t addresseeprom, uint16_t eepromSize); // Set I2C settings
    void testRTC();                                                  // Test the RTC functionality
    void logCurrentTime();                                           // Log the current time from the RTC
    void setTime(const std::string& time);                           // Set the time
    void setDate(const std::string& date);                           // Set the date
    time_t getTime();                                                // Get the Unix time

#ifdef ENABLE_TEMPERATURE
    float getTemperature(); // Get the temperature from the RTC
#endif

#ifdef ENABLE_EEPROM
    void writeEEPROM(uint16_t address, uint8_t data); // Write data to the external EEPROM
    uint8_t readEEPROM(uint16_t address);             // Read data from the external EEPROM
    void testEEPROM();                                // Test the EEPROM functionality
#endif

#ifdef ENABLE_ALARMS
    enum AlarmType
    {
        Alarm1 = 1,
        Alarm2 = 2
    };
    struct Alarm
    {
        AlarmType type;
        uint8_t hour;
        uint8_t minute;
        uint8_t second; // Only for Alarm1
        enum DayOfWeek
        {
            Sunday,
            Monday,
            Tuesday,
            Wednesday,
            Thursday,
            Friday,
            Saturday
        } dayOfWeek;
        struct AlarmMode
        {
            bool matchSeconds = false;   // On every second
            bool matchMinutes = false;   // On every minute
            bool matchHours = false;     // On every hour
            bool matchDay = false;       // On every day
            bool matchDayOfWeek = false; // On every day of week
        } mode;
        enum ControlStatus
        {
            AlarmEnabled = 0x00,
            AlarmDisabled = 0x80
        } controlStatus;
    };

    bool setAlarm(Alarm alarm);                                                 // Set an alarm
    Alarm getAlarm(AlarmType type);                                              // Get alarm settings
    void logAlarm(AlarmType type);                                               // Log the current alarm settings
    inline bool checkAlarm(AlarmType type) { return rtc.alarmFired(type); } // Check if an alarm has triggered
    inline void disableAlarm(AlarmType type) { rtc.disableAlarm(type); }    // Disable an alarm
    bool checkAndClearAlarmStatus(AlarmType type);                               // Check if the alarm was triggered and clear the flag
#endif

  private:
    bool _deviceRTCinitialized = false; // Whether the RTC is initialized
    RTC_DS3231 rtc;                     // RTC object
    // i2cRTC rtcI2C; // i2cRTC object
};

extern DeviceRTC openknxRTCModule; // Declare the DeviceRTC object
