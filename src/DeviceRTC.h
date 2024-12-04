#pragma once
/**
 * @file        DeviceRTC.h
 * @brief       This module offers a real-time clock (RTC) for the OpenKNX ecosystem
 * @version     0.0.1
 * @date        2024-11-27
 * @copyright   Copyright (c) 2024, Erkan Çolak (erkan@çolak.de)
 *              Licensed under GNU GPL v3.0
 */
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
    i2cRTC();
    ~i2cRTC();

    struct RTCSettings
    {
        uint8_t i2cAddress = 0x68;       // Default I2C address for DS3231
        uint8_t i2cAddressEEPROM = 0x57; // Default I2C address for DS3231 EEPROM
        uint16_t i2cEEPROMSize = 0x1000; // Default I2C EEPROM size for DS3231 (4096 bytes (4K), 0x000 to 0xFFF)
        i2c_inst_t* i2cInst = nullptr;   // I2C instance (i2c0 or i2c1)
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
    DeviceRTC();

    void begin();                                                    // Initialize the RTC
    inline bool isInitialized() { return _deviceRTCinitialized; }    // Check if the RTC is initialized
    void setI2CSettings(i2c_inst_t* i2cInst, uint8_t scl, uint8_t sda, uint8_t address,
                        uint8_t addresseeprom, uint16_t eepromSize); // Set I2C settings
    // OpenKNX Module methods
    void init() override;                                                    // Initialize the module
    void setup(bool configured) override;                                    // Setup the module
    void processInputKo(GroupObject& obj) override;                          // Process input KO
    void showHelp() override;                                                // Show help for console commands
    bool processCommand(const std::string command, bool diagnose) override;  // Process console commands
    inline const std::string name() { return DeviceRTC_Display_Name; }       // Library name
    inline const std::string version() { return DeviceRTC_Display_Version; } // Library version
    
    DateTime now();                        // Get the current time from the RTC
    void setTime(const std::string& time); // Set the time
    void setDate(const std::string& date); // Set the date
    time_t getTime();                      // Get the Unix time
    void adjust(const DateTime& dt);       // Adjust the RTC to a new time
    void testRTC();                        // Test the RTC functionality
    void logCurrentTime();                 // Log the current time from the RTC
#ifdef ENABLE_TEMPERATURE
    float getTemperature(); // Get the temperature from the RTC
#endif  // ENABLE_TEMPERATURE

#ifdef ENABLE_EEPROM
    void writeEEPROM(uint16_t address, uint8_t data); // Write data to the external EEPROM
    uint8_t readEEPROM(uint16_t address);             // Read data from the external EEPROM
    void testEEPROM();                                // Test the EEPROM functionality
#endif  // ENABLE_EEPROM

#ifdef ENABLE_ALARMS
    enum AlarmType
    {
        Alarm1 = 1,
        Alarm2 = 2
    };
    struct Alarm
    {
        AlarmType type;     // Alarm type (Alarm1 or Alarm2)
        uint8_t hour;       // 0-23
        uint8_t minute;     // 0-59
        uint8_t second;     // Only for Alarm1
        uint8_t dayOfMonth; // 1-31 or 0 for every day
        enum DayOfWeek
        {
            Monday,
            Tuesday,
            Wednesday,
            Thursday,
            Friday,
            Saturday,
            Sunday,
            Undefined
        } dayOfWeek; // Only for Alarm2

        Ds3231Alarm1Mode modeAlarm1; // Alarm1 mode
        Ds3231Alarm2Mode modeAlarm2; // Alarm2 mode
        bool alarmEnabled;           // Whether the alarm is enabled
    };

    bool setAlarm(Alarm alarm);                                               // Set an alarm
    Alarm getAlarm(AlarmType type);                                           // Get alarm settings
    void logAlarm(AlarmType type);                                            // Log the current alarm settings
    inline bool checkAlarm(AlarmType type) { return rtc.alarmFired(type); }   // Check if an alarm has triggered
    inline void disableAlarm(AlarmType type) { rtc.disableAlarm(type); }      // Disable an alarm
    inline void clearAlarm(AlarmType type) { rtc.clearAlarm(type); }          // Clear an alarm
    bool checkAndClearAlarm(AlarmType type, bool force = false);              // Check if the alarm was triggered and clear the flag
    inline byte decToBcd(byte val) { return ((val / 10 * 16) + (val % 10)); } // Convert decimal to BCD
    inline int bcdToDec(byte val) { return ((val / 16 * 10) + (val % 16)); }  // Convert BCD to decimal
#endif  // ENABLE_ALARMS

  private:
    bool _deviceRTCinitialized = false; // Whether the RTC is initialized
    RTC_DS3231 rtc;                     // RTC object
    // i2cRTC rtcI2C; // i2cRTC object
};

extern DeviceRTC openknxRTCModule; // Declare the DeviceRTC object
