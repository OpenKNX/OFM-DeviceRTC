#pragma once

#include <RTClib.h>
#include <OpenKNX.h>
#include <Wire.h>

#define DeviceRTC_Display_Name "DeviceRTC" // Display name
#define DeviceRTC_Display_Version "0.0.1" // Display version

// Uncomment the following lines to enable the respective features
#define ENABLE_TEMPERATURE // Enable temperature reading
#define ENABLE_EEPROM      // Enable external EEPROM
#define ENABLE_ALARMS      // Enable alarms

#ifdef ENABLE_EEPROM
#define EEPROM_SIZE 0x800 // Define for AT24C32 EEPROM size (4K, 0x000 to 0x7FF)
#endif

class i2cRTC {
public:
    i2cRTC();  // Constructor
    ~i2cRTC(); // Destructor

    struct RTCSettings {
        uint8_t i2cAddress = 0x68; // Default I2C address for DS3231
        bool bIsi2c1 = false; // true:i2c1 false:i2c0
        pin_size_t sda = -1; // SDA pin
        pin_size_t scl = -1; // SCL pin
    } rtcSettings;

    std::unique_ptr<TwoWire> customI2C; // Custom I2C object

    void setup(); // Setup method for initialization
    bool initRTC(); // Initialize the RTC
    bool initRTC(RTCSettings settings); // Initialize the RTC with custom settings
};

class DeviceRTC : public OpenKNX::Module {
public:
    DeviceRTC(); // Constructor
    void begin(); // Initialize the RTC
    DateTime now(); // Get the current time from the RTC
    void adjust(const DateTime& dt); // Adjust the RTC to a new time
    
    void init() override; // Initialize the module
    void setup(bool configured) override; // Setup the module
    void processInputKo(GroupObject& obj) override; // Process input KO
    void showHelp() override; // Show help for console commands
    bool processCommand(const std::string command, bool diagnose) override; // Process console commands
    
    inline const std::string name() { return DeviceRTC_Display_Name; }       // Library name
    inline const std::string version() { return DeviceRTC_Display_Version; } // Library version

    void setI2CSettings(uint8_t scl, uint8_t sda, uint8_t address, bool bIsi2c1); // Set I2C settings
    void testRTC(); // Test the RTC functionality
    void logCurrentTime(); // Log the current time from the RTC
    void setTime(const std::string& time); // Set the time
    void setDate(const std::string& date); // Set the date

#ifdef ENABLE_TEMPERATURE
    float getTemperature(); // Get the temperature from the RTC
#endif

#ifdef ENABLE_EEPROM
    void writeEEPROM(uint16_t address, uint8_t data); // Write data to the external EEPROM
    uint8_t readEEPROM(uint16_t address); // Read data from the external EEPROM
#endif

#ifdef ENABLE_ALARMS
    void setAlarm1(const DateTime& dt); // Set Alarm 1
    void setAlarm2(const DateTime& dt); // Set Alarm 2
    void clearAlarms(); // Clear all alarms
    bool checkAlarm1(); // Check if Alarm 1 has triggered
    bool checkAlarm2(); // Check if Alarm 2 has triggered
#endif

private:
    RTC_DS3231 rtc; // RTC object
    //i2cRTC rtcI2C; // i2cRTC object
};

extern DeviceRTC openknxRTCModule; // Declare the DeviceRTC object
