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
    openknx.console.printHelpLine("rtc", "Device RTC Control. Use 'rtc ?' for more.");
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
        if (command.compare(4, 4, "test") == 0) // rtc test
        {
            testRTC();
            bRet = true;
        }
        else if (command.compare(4, 3, "log") == 0) // rtc log
        {
            
            logCurrentTime();
            bRet = true;
        }
        else if (command.compare(4, 6, "stime ") == 0) // rtc stime HH:MM:SS
        {
            logInfoP("Setting time: %s", command.substr(10).c_str());
            setTime(command.substr(10));
            bRet = true;
        }
        else if (command.compare(4, 6, "sdate ") == 0) // rtc sdate DD:MM:YYYY
        {
            // rtc sdate DD:MM:YYYY
            logInfoP("Setting date: %s", command.substr(10).c_str());
            setDate(command.substr(10));
            bRet = true;
        }
#ifdef ENABLE_TEMPERATURE
        else if (command.compare(4, 4, "temp") == 0) // rtc temp
        {
            float temp = getTemperature();
            logInfoP("RTC temperature: %.2f C", temp);
            bRet = true;
        }
#endif // ENABLE_TEMPERATURE
#ifdef ENABLE_EEPROM
        else if (command.compare(4, 8, "testeprm") == 0) // rtc testeprm
        {
            testEEPROM();
            bRet = true;
        }
        else if (command.compare(4, 6, "weprm ") == 0) // rtc weprm <address> <data>
        {
            int address, data;
            if (sscanf(command.c_str() + 10, "%d %d", &address, &data) == 2)
            {
                writeEEPROM(address, data);
                logInfoP("Written %d to EEPROM address %d", data, address);
                bRet = true;
            }
            else
            {
                logErrorP("Invalid command format. Use: rtc writeeeprom <address> <data>");
            }
        }
        else if (command.compare(4, 6, "reprm ") == 0) // rtc reprm <address>
        {
            int address;
            if (sscanf(command.c_str() + 10, "%d", &address) == 1)
            {
                uint8_t data = readEEPROM(address);
                logInfoP("Read %d from EEPROM address %d", data, address);
                bRet = true;
            }
            else
            {
                logErrorP("Invalid command format. Use: rtc readeeprom <address>");
            }
        }
#endif // ENABLE_EEPROM
#ifdef ENABLE_ALARMS
        else if (command.compare(4, 4, "sa1 ") == 0) // rtc sa1 HH:MM:SS DD:MM:YYYY
        {
            uint8_t hour, minute, second, day, month; uint16_t year;
            if (sscanf(command.c_str() + 8, "%d:%d:%d %d:%d:%d", &hour, &minute, &second, &day, &month, &year) == 6)
            {
                logInfoP("Setting Alarm 1: %02d:%02d:%02d %02d:%02d:%02d", hour, minute, second, day, month, year);
                DateTime dt(year, month, day, hour, minute, second);
                setAlarm1(dt);
                logInfoP("Alarm 1 set to: %02d:%02d:%02d %02d:%02d:%02d", dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(), dt.year());
                bRet = true;
            }
            else
            {
                logErrorP("Invalid date/time format. Use HH:MM:SS DD:MM:YYYY");
            }
        }
        else if (command.compare(4, 3, "ga1") == 0) // rtc ga1
        {
            logAlarm1();
            bRet = true;
        }
        else if (command.compare(4, 4, "sa2 ") == 0) // rtc sa2 HH:MM:SS DD:MM:YYYY
        {
            // rtc sa2 HH:MM:SS DD:MM:YYYY
            uint8_t hour, minute, second, day, month; uint16_t year;
            if (sscanf(command.c_str() + 8, "%d:%d:%d %d:%d:%d", &hour, &minute, &second, &day, &month, &year) == 6)
            {
                logInfoP("Setting Alarm 2: %02d:%02d:%02d %02d.%02d.%02d", hour, minute, second, day, month, year);
                DateTime dt(year, month, day, hour, minute, second);
                setAlarm2(dt);
                logInfoP("Alarm 2 set to: %02d:%02d:%02d %02d.%02d.%02d", dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(), dt.year());
                bRet = true;
            }
            else
            {
                logErrorP("Invalid date/time format. Use HH:MM:SS DD:MM:YYYY");
            }
        }
        else if (command.compare(4, 3, "ga2") == 0) // rtc ga2
        {
            logAlarm2();
            bRet = true;
        }
        else if (command.compare(4, 3, "cal") == 0)
        {
            clearAlarms();
            logInfoP("All alarms cleared.");
            bRet = true;
        }
#endif // ENABLE_ALARMS
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
            openknx.console.printHelpLine("rtc stime HH:MM:SS", "Set the time (24-hour format)");
            openknx.console.printHelpLine("rtc sdate DD:MM:YYYY", "Set the date");
#ifdef ENABLE_TEMPERATURE
            openknx.console.printHelpLine("rtc temp", "Get the temperature from RTC");
#endif
#ifdef ENABLE_EEPROM
            openknx.console.printHelpLine("rtc weprm <address> <data>", "Write data to external EEPROM");
            openknx.console.printHelpLine("rtc reprm <address>", "Read data from external EEPROM");
            openknx.console.printHelpLine("rtc testeprm", "Test internal EEPROM");
#endif
#ifdef ENABLE_ALARMS
            openknx.console.printHelpLine("rtc sa1 HH:MM:SS DD:MM:YYYY", "Set Alarm 1");
            openknx.console.printHelpLine("rtc ga1", "Get Alarm 1");
            openknx.console.printHelpLine("rtc sa2 HH:MM:SS DD:MM:YYYY", "Set Alarm 2");
            openknx.console.printHelpLine("rtc ga2", "Get Alarm 2");
            openknx.console.printHelpLine("rtc cal", "Clear all alarms");
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
    // Check if the RTC lost power
    DateTime now = rtc.now();
    if (rtc.lostPower())
    {
        logErrorP("RTC lost power!");
    }
    else
    {
        logInfoP("RTC power OK.");
    }
    // Check if the RTC is running
    if (!rtc.begin(rtcI2C->customI2C.get()))
    {
        logErrorP("RTC not running!");
    }
    else
    {
        logInfoP("RTC running.");
    }
    logInfoP("Current time: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
    logInfoP("Current date: %02d-%02d-%02d", now.day(), now.month(), now.year());
    logInfoP("Unixtime: %lu", rtc.now().unixtime());
    logInfoP("RTC square wave output: %s", rtc.readSqwPinMode() == DS3231_OFF ? "Off" : "On");
#ifdef ENABLE_TEMPERATURE
    logInfoP("Temperature: %.2f C", rtc.getTemperature());
#endif
    logInfoP("RTC test complete.");
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
 * @brief Test the EEPROM functionality
 *
 */
void DeviceRTC::testEEPROM()
{
    logInfoP("EEPROM Read/Write test...");
    logInfoP("EEPROM Size: %d bytes", EEPROM_SIZE);
    uint32_t startTime = millis();
    bool errorFound = false;
    uint32_t bytesWritten = 0;

    // Test specific ranges
    uint16_t testRanges[][2] = {
        {0x0000, 0x00FF},
        {0x0400, 0x04FF},
        {0x0800, 0x08FF},
        {0x0C00, 0x0CFF}};

    for (auto &range : testRanges)
    {
        for (uint16_t i = range[0]; i <= range[1]; i++)
        {
            writeEEPROM(i, i % 256);
            delay(10); // Ensure write delay
            bytesWritten++;
            if (i % 256 == 0)
            {
                logInfoP("EEPROM writing at address 0x%04X of 0x%02X passed.", i, i % 256);
            }
        }
    }

    uint32_t writeTime = millis() - startTime;
    logInfoP("EEPROM write test completed in %lu ms", writeTime);
    logInfoP("Total bytes written: %lu bytes (%.2f KB)", bytesWritten, bytesWritten / 1024.0);

    logInfoP("Reading back EEPROM data...");
    startTime = millis();

    for (auto &range : testRanges)
    {
        for (uint16_t i = range[0]; i <= range[1]; i++)
        {
            uint8_t data = readEEPROM(i);
            delay(10); // Ensure read delay
            if (data != (i % 256))
            {
                logErrorP("EEPROM read error at address 0x%04X. Expected 0x%02X, got 0x%02X", i, i % 256, data);
                errorFound = true;
            }
            else if (i % 256 == 0)
            {
                logDebugP("EEPROM read test at address 0x%04X passed.", i);
            }
        }
    }

    uint32_t readTime = millis() - startTime;
    logInfoP("EEPROM read test completed in %lu ms", readTime);
    logInfoP("Total bytes written: %lu bytes (%.2f KB)", bytesWritten, bytesWritten / 1024.0);
    logInfoP("Total bytes read back: %lu bytes (%.2f KB)", bytesWritten, bytesWritten / 1024.0);
    logInfoP("EEPROM write speed: %.2f bytes/s", (float)bytesWritten / (writeTime / 1000.0));
    logInfoP("EEPROM read speed: %.2f bytes/s", (float)bytesWritten / (readTime / 1000.0));
    logInfoP("Total EEPROM test time: %lu ms", writeTime + readTime);

    if (errorFound)
    {
        logErrorP("EEPROM test found errors in some memory areas.");
    }
    else
    {
        logInfoP("EEPROM test completed successfully with no errors.");
    }
}

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
        logErrorP("Invalid EEPROM address. Must be between 0x000 and 0xFFF.");
        return;
    }
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddress);
    rtcI2C->customI2C->write((address >> 8) & 0xFF); // MSB
    rtcI2C->customI2C->write(address & 0xFF);        // LSB
    rtcI2C->customI2C->write(data);
    rtcI2C->customI2C->endTransmission();
    delay(10); // EEPROM write delay
    logDebugP("Written 0x%02X to EEPROM address 0x%04X", data, address);
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
        logErrorP("Invalid EEPROM address. Must be between 0x000 and 0xFFF.");
        return 0;
    }
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddress);
    rtcI2C->customI2C->write((address >> 8) & 0xFF); // MSB
    rtcI2C->customI2C->write(address & 0xFF);        // LSB
    rtcI2C->customI2C->endTransmission();
    rtcI2C->customI2C->requestFrom(rtcI2C->rtcSettings.i2cAddress, (uint8_t)1);
    if (rtcI2C->customI2C->available())
    {
        uint8_t data = rtcI2C->customI2C->read();
        logDebugP("Read 0x%02X from EEPROM address 0x%04X", data, address);
        return data;
    }
    else
    {
        logErrorP("EEPROM read error at address 0x%04X", address);
        return 0;
    }
}
#endif

#ifdef ENABLE_ALARMS
/**
 * @brief Get the current Alarm 1 settings
 *
 * @return DateTime Current Alarm 1 settings
 */
DateTime DeviceRTC::getAlarm1()
{
    uint8_t buffer[7];
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddress);
    rtcI2C->customI2C->write(0x07); // Alarm 1 register start address
    rtcI2C->customI2C->endTransmission();
    rtcI2C->customI2C->requestFrom(rtcI2C->rtcSettings.i2cAddress, 4);
    for (int i = 0; i < 4; i++)
    {
        buffer[i] = rtcI2C->customI2C->read();
    }
    //return DateTime(2000, 1, 1, BCD2DEC(buffer[2]), BCD2DEC(buffer[1]), BCD2DEC(buffer[0]));
    return DateTime(BCD2DEC(buffer[6]) + 2000, BCD2DEC(buffer[5]), BCD2DEC(buffer[4]), BCD2DEC(buffer[2]), BCD2DEC(buffer[1]), BCD2DEC(buffer[0])); 
}

/**
 * @brief Log the current Alarm 1 settings
 *
 */
void DeviceRTC::logAlarm1()
{
    DateTime alarm1 = getAlarm1();
    logInfoP("Alarm 1 is set to: %02d-%02d-%02d %02d:%02d:%02d", alarm1.year(), alarm1.month(), alarm1.day(), alarm1.hour(), alarm1.minute(), alarm1.second());
}

/**
 * @brief Get the current Alarm 2 settings
 *
 * @return DateTime Current Alarm 2 settings
 */
DateTime DeviceRTC::getAlarm2()
{
    uint8_t buffer[7];
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddress);
    rtcI2C->customI2C->write(0x0B); // Alarm 2 register start address
    rtcI2C->customI2C->endTransmission();
    rtcI2C->customI2C->requestFrom(rtcI2C->rtcSettings.i2cAddress, 3);
    for (int i = 0; i < 3; i++)
    {
        buffer[i] = rtcI2C->customI2C->read();
    }
    // es soll auch das gesetzte datum und uhrzeit zurückgegeben werden
    return DateTime(BCD2DEC(buffer[6]) + 2000, BCD2DEC(buffer[5]), BCD2DEC(buffer[4]), BCD2DEC(buffer[2]), BCD2DEC(buffer[1]), BCD2DEC(buffer[0]));
    
}

/**
 * @brief Log the current Alarm 2 settings
 *
 */
void DeviceRTC::logAlarm2()
{
    DateTime alarm2 = getAlarm2();
    logInfoP("Alarm 2 is set to: %02d-%02d-%02d %02d:%02d:%02d", alarm2.year(), alarm2.month(), alarm2.day(), alarm2.hour(), alarm2.minute(), alarm2.second());
}
#endif