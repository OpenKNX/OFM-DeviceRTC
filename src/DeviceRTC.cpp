#include "DeviceRTC.h"

DeviceRTC openknxRTCModule;
i2cRTC *rtcI2C = new i2cRTC();

/**
 * @brief Construct a new i2cRTC::i2cRTC object and initialize the RTC settings.
 *
 */
i2cRTC::i2cRTC() : rtcSettings()
{
    rtcSettings.i2cAddress = 0x68; // Default I2C address for DS3231
    rtcSettings.bIsi2c1 = false;   // true:i2c1 false:i2c0
    rtcSettings.sda = -1;          // SDA pin
    rtcSettings.scl = -1;          // SCL pin
}

/**
 * @brief Destroy the i2cRTC::i2cRTC object from the memory.
 *
 */
i2cRTC::~i2cRTC()
{
    customI2C.reset(); // Remove the I2C object from the memory
}

/**
 * @brief Initialize the i2cRTC object with the default settings.
 * @return true if the RTC was initialized successfully
 */
bool i2cRTC::initRTC()
{
    if (rtcSettings.sda < 0 || rtcSettings.scl < 0)
    {
        return false; // SDA and SCL pins must be set
    }
    customI2C = std::make_unique<TwoWire>(rtcSettings.bIsi2c1 ? i2c1 : i2c0, rtcSettings.sda, rtcSettings.scl);
    return true;
}

/**
 * @brief Initialize the i2cRTC object with the custom settings.
 * @param settings the RTC settings
 * @return true if the RTC was initialized successfully
 */
bool i2cRTC::initRTC(RTCSettings settings)
{
    rtcSettings = settings;
    return initRTC();
}

/**
 * @brief Setup method for initialization
 *
 */
void i2cRTC::setup()
{
    // ToDo: Setup OpenKNX Hardware Specific I2C settings, like SDA, SCL, I2C address, etc.
}

/**
 * @brief Construct a new DeviceRTC::DeviceRTC object
 *
 */
DeviceRTC::DeviceRTC() : rtc() {}

/**
 * @brief Initialize the RTC
 *
 */
void DeviceRTC::begin()
{
    if (!rtcI2C->initRTC())
    {
        logErrorP("RTC I2C initialization failed!");
    }
    else
    {
        logInfoP("RTC I2C initialized.");
    }
    if (!rtc.begin(rtcI2C->customI2C.get()))
    {
        // Error handling if the RTC cannot be initialized
        logErrorP("RTC initialization failed!");
    }
    else
    {
        logInfoP("RTC initialized.");
    }
    if (rtc.lostPower())
    {
        // Set the RTC to a default date if it lost power
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        logInfoP("RTC lost power, setting default time.");
    }
    else
    {
        logInfoP("RTC power OK.");
    }
}

/**
 * @brief Get the current time from the RTC
 *
 * @return DateTime
 */
DateTime DeviceRTC::now()
{
    return rtc.now();
}

/**
 * @brief Adjust the RTC to a new time
 *
 * @param dt New DateTime to set
 */
void DeviceRTC::adjust(const DateTime &dt)
{
    rtc.adjust(dt);
}

/**
 * @brief Initialize the module
 *
 */
void DeviceRTC::init()
{
    // Initialization code for OpenKNX
    logInfoP("DeviceRTC initialized.");
    return begin();
}

/**
 * @brief Setup the module
 *
 * @param configured Whether the module is configured
 */
void DeviceRTC::setup(bool configured)
{
    // Setup code for OpenKNX
    logInfoP("DeviceRTC setup with configured: %d", configured);
}

/**
 * @brief Process input KO
 *
 * @param obj GroupObject to process
 */
void DeviceRTC::processInputKo(GroupObject &obj)
{
    // Process input KO for OpenKNX
    // logInfoP("Processing input KO: %s", obj.getName().c_str());
}

/**
 * @brief Show help for console commands
 *
 */
void DeviceRTC::showHelp()
{
    // Show help information for OpenKNX
    openknx.console.printHelpLine("rtc ?", "Device RTC Control. Use 'rtc ?' for more.");
}

/**
 * @brief Process console commands
 *
 * @param command Command to process
 * @param diagnose Whether to diagnose the command
 */
bool DeviceRTC::processCommand(const std::string command, bool diagnose)
{
    bool bRet = false;
    if ((!diagnose) && command.compare(0, 4, "rtc ") == 0)
    {
        logInfoP("Processing RTC command: %s", command.c_str());
        if (command.compare(4, 5, "test") == 0)
        {
            testRTC();
        }
        else if (command.compare(4, 4, "log") == 0)
        {
            logCurrentTime();
        }
        else if (command.compare(4, 8, "settime") == 0)
        {
            setTime(command.substr(12));
        }
        else if (command.compare(4, 8, "setdate") == 0)
        {
            setDate(command.substr(12));
#ifdef ENABLE_TEMPERATURE
        }
        else if (command.compare(4, 4, "temp") == 0)
        {
            float temp = getTemperature();
            logInfoP("RTC temperature: %.2f C", temp);
#endif
#ifdef ENABLE_EEPROM
        }
        else if (command.compare(4, 12, "writeeeprom") == 0)
        {
            int address, data;
            if (sscanf(command.c_str() + 16, "%d %d", &address, &data) == 2)
            {
                writeEEPROM(address, data);
                logInfoP("Written %d to EEPROM address %d", data, address);
            }
            else
            {
                logErrorP("Invalid command format. Use: rtc writeeeprom <address> <data>");
            }
        }
        else if (command.compare(4, 11, "readeeprom") == 0)
        {
            int address;
            if (sscanf(command.c_str() + 15, "%d", &address) == 1)
            {
                uint8_t data = readEEPROM(address);
                logInfoP("Read %d from EEPROM address %d", data, address);
            }
            else
            {
                logErrorP("Invalid command format. Use: rtc readeeprom <address>");
            }
#endif
#ifdef ENABLE_ALARMS
        }
        else if (command.compare(4, 9, "setalarm1") == 0)
        {
            int hour, minute, second, day, month, year;
            if (sscanf(command.c_str() + 13, "%d:%d:%d %d:%d:%d", &hour, &minute, &second, &day, &month, &year) == 6)
            {
                DateTime dt(year, month, day, hour, minute, second);
                setAlarm1(dt);
                logInfoP("Alarm 1 set to: %02d:%02d:%02d %02d-%02d-%02d", dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(), dt.year());
            }
            else
            {
                logErrorP("Invalid date/time format. Use HH:MM:SS DD:MM:YYYY");
            }
        }
        else if (command.compare(4, 9, "setalarm2") == 0)
        {
            int hour, minute, second, day, month, year;
            if (sscanf(command.c_str() + 13, "%d:%d:%d %d:%d:%d", &hour, &minute, &second, &day, &month, &year) == 6)
            {
                DateTime dt(year, month, day, hour, minute, second);
                setAlarm2(dt);
                logInfoP("Alarm 2 set to: %02d:%02d:%02d %02d-%02d-%02d", dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(), dt.year());
            }
            else
            {
                logErrorP("Invalid date/time format. Use HH:MM:SS DD:MM:YYYY");
            }
        }
        else if (command.compare(4, 11, "clearalarms") == 0)
        {
            clearAlarms();
            logInfoP("All alarms cleared.");
        }
#endif
        else
        {
            openknx.logger.begin();
            openknx.logger.log("");
            openknx.logger.color(CONSOLE_HEADLINE_COLOR);
            openknx.logger.log("======================= Help: Device RTC Control ===========================");
            openknx.logger.color(0);
            openknx.logger.log("Command(s)               Description");
            openknx.console.printHelpLine("rtc test", "Test the RTC functionality");
            openknx.console.printHelpLine("rtc log", "Log the current time from RTC");
            openknx.console.printHelpLine("rtc settime HH:MM:SS", "Set the time (24-hour format)");
            openknx.console.printHelpLine("rtc setdate DD:MM:YYYY", "Set the date");
#ifdef ENABLE_TEMPERATURE
            openknx.console.printHelpLine("rtc temp", "Get the temperature from RTC");
#endif
#ifdef ENABLE_EEPROM
            openknx.console.printHelpLine("rtc writeeeprom <address> <data>", "Write data to external EEPROM");
            openknx.console.printHelpLine("rtc readeeprom <address>", "Read data from external EEPROM");
#endif
#ifdef ENABLE_ALARMS
            openknx.console.printHelpLine("rtc setalarm1 HH:MM:SS DD:MM:YYYY", "Set Alarm 1");
            openknx.console.printHelpLine("rtc setalarm2 HH:MM:SS DD:MM:YYYY", "Set Alarm 2");
            openknx.console.printHelpLine("rtc clearalarms", "Clear all alarms");
#endif
            openknx.logger.color(CONSOLE_HEADLINE_COLOR);
            openknx.logger.log("--------------------------------------------------------------------------------");
            openknx.logger.color(0);
            openknx.logger.end();
            bRet = true;
        }
        bRet = true;
    }
    return bRet;
}

/**
 * @brief Set I2C settings
 *
 * @param scl SCL pin
 * @param sda SDA pin
 * @param address I2C address
 */
void DeviceRTC::setI2CSettings(uint8_t scl, uint8_t sda, uint8_t address, bool bIsi2c1)
{
    i2cRTC::RTCSettings settings;
    settings.scl = scl;
    settings.sda = sda;
    settings.i2cAddress = address;
    settings.bIsi2c1 = bIsi2c1;
    rtcI2C->initRTC(settings);
}

/**
 * @brief Test the RTC functionality
 *
 */
void DeviceRTC::testRTC()
{
    logInfoP("Testing RTC...");
    DateTime now = rtc.now();
    logInfoP("Current time: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
}

/**
 * @brief Log the current time from the RTC
 *
 */
void DeviceRTC::logCurrentTime()
{
    DateTime now = rtc.now();
    logInfoP("RTC current time: %02d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
}

/**
 * @brief Set the time
 *
 * @param time Time string in HH:MM:SS format
 */
void DeviceRTC::setTime(const std::string &time)
{
    int hour, minute, second;
    if (sscanf(time.c_str(), "%d:%d:%d", &hour, &minute, &second) == 3)
    {
        DateTime now = rtc.now();
        DateTime newTime(now.year(), now.month(), now.day(), hour, minute, second);
        rtc.adjust(newTime);
        logInfoP("Time set to: %02d:%02d:%02d", hour, minute, second);
    }
    else
    {
        logErrorP("Invalid time format. Use HH:MM:SS");
    }
}

/**
 * @brief Set the date
 *
 * @param date Date string in DD:MM:YYYY format
 */
void DeviceRTC::setDate(const std::string &date)
{
    int day, month, year;
    if (sscanf(date.c_str(), "%d:%d:%d", &day, &month, &year) == 3)
    {
        DateTime now = rtc.now();
        DateTime newDate(year, month, day, now.hour(), now.minute(), now.second());
        rtc.adjust(newDate);
        logInfoP("Date set to: %02d-%02d-%02d", day, month, year);
    }
    else
    {
        logErrorP("Invalid date format. Use DD:MM:YYYY");
    }
}

#ifdef ENABLE_TEMPERATURE
/**
 * @brief Get the temperature from the RTC
 *
 * @return float Temperature in Celsius
 */
float DeviceRTC::getTemperature()
{
    return rtc.getTemperature();
}
#endif

#ifdef ENABLE_EEPROM
/**
 * @brief Write data to the external EEPROM
 *
 * @param address EEPROM address to write to
 * @param data Data to write
 */
void DeviceRTC::writeEEPROM(uint16_t address, uint8_t data)
{
    if (address >= EEPROM_SIZE)
    { // Check against defined EEPROM size
        logErrorP("Invalid EEPROM address. Must be between 0x000 and 0x7FF.");
        return;
    }
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddress);
    rtcI2C->customI2C->write((address >> 8) & 0xFF); // MSB
    rtcI2C->customI2C->write(address & 0xFF);        // LSB
    rtcI2C->customI2C->write(data);
    rtcI2C->customI2C->endTransmission();
    delay(5); // EEPROM write delay
}

/**
 * @brief Read data from the external EEPROM
 *
 * @param address EEPROM address to read from
 * @return uint8_t Data read from EEPROM
 */
uint8_t DeviceRTC::readEEPROM(uint16_t address)
{
    if (address >= EEPROM_SIZE)
    { // Check against defined EEPROM size
        logErrorP("Invalid EEPROM address. Must be between 0x000 and 0x7FF.");
        return 0;
    }
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddress);
    rtcI2C->customI2C->write((address >> 8) & 0xFF); // MSB
    rtcI2C->customI2C->write(address & 0xFF);        // LSB
    rtcI2C->customI2C->endTransmission();
    rtcI2C->customI2C->requestFrom(rtcI2C->rtcSettings.i2cAddress, (uint8_t)1);
    return rtcI2C->customI2C->read();
}
#endif

#ifdef ENABLE_ALARMS
/**
 * @brief Set Alarm 1
 *
 * @param dt DateTime to set for Alarm 1
 */
void DeviceRTC::setAlarm1(const DateTime &dt)
{
    rtc.setAlarm1(dt, DS3231_A1_Date);
}

/**
 * @brief Set Alarm 2
 *
 * @param dt DateTime to set for Alarm 2
 */
void DeviceRTC::setAlarm2(const DateTime &dt)
{
    rtc.setAlarm2(dt, DS3231_A2_Date);
}

/**
 * @brief Clear all alarms
 *
 */
void DeviceRTC::clearAlarms()
{
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
}

/**
 * @brief Check if Alarm 1 has triggered
 *
 * @return true if Alarm 1 has triggered, false otherwise
 */
bool DeviceRTC::checkAlarm1()
{
    return rtc.alarmFired(1);
}

/**
 * @brief Check if Alarm 2 has triggered
 *
 * @return true if Alarm 2 has triggered, false otherwise
 */
bool DeviceRTC::checkAlarm2()
{
    return rtc.alarmFired(2);
}
#endif