#include "DeviceRTC.h"
#include "TimeClockRTC.h"

DeviceRTC openknxRTCModule;
i2cRTC *rtcI2C = new i2cRTC();

/**
 * @brief Construct a new i2cRTC::i2cRTC object and initialize the RTC settings.
 */
i2cRTC::i2cRTC() : rtcSettings()
{
    rtcSettings.i2cAddress = 0x68;       // Default I2C address for DS3231
    rtcSettings.i2cAddressEEPROM = 0x57; // Default I2C address for DS3231 EEPROM
    rtcSettings.i2cEEPROMSize = 0x1000;  // Default I2C EEPROM size for DS3231 (4096 bytes (4K), 0x000 to 0xFFF)
    rtcSettings.i2cInst = nullptr;       // I2C instance (i2c0 or i2c1)
    rtcSettings.sda = -1;                // SDA pin
    rtcSettings.scl = -1;                // SCL pin
}

/**
 * @brief Destroy the i2cRTC::i2cRTC object from the memory.
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
    if (rtcSettings.sda < 0 || rtcSettings.scl < 0 || rtcSettings.i2cInst == nullptr)
    {
        return false; // SDA and SCL and i2c instance must be set
    }
    customI2C = std::make_unique<TwoWire>(rtcSettings.i2cInst, rtcSettings.sda, rtcSettings.scl);
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
 * @brief Scans the I2C bus for devices.
 * @param devices required to store the device addresses.
 *                Mark the end of the list with -1.
 * @return true if any devices are found
 */
bool i2cRTC::scanI2C(int8_t *devices)
{
    int8_t nDevices = 0;
    for (uint8_t address = 1; address < 127; address++) // Check all possible I2C addresses
    {
        customI2C->beginTransmission(address);
        uint8_t error = customI2C->endTransmission();
        if (error == 0)
        {
            // Check if the device list is not full
            if (nDevices < (sizeof(devices) / sizeof(devices[0])))
            {
                devices[nDevices++] = address;
            }
            else
                break; // Stop, the device list is full
        }
        // else if (error == 4) // Error code 4 means no device found at this address
        // else if (error == 3) // Error code 3 means a communication error
        // else if (error == 2) // Error code 2 means the device did not acknowledge
        // else if (error == 1) // Error code 1 means the bus is busy
        // else  // Unknown error
    }
    if (nDevices == 0)
    {
        return false; // Return false if no devices found
    }
    devices[nDevices] = -1; // Mark the end of the list with -1
    return true;
}

/**
 * @brief Setup method for initialization
 */
void i2cRTC::setup()
{
    // ToDo: Setup OpenKNX Hardware Specific I2C settings, like SDA, SCL, I2C address, etc.
}

/**
 * @brief Construct a new DeviceRTC::DeviceRTC object
 */
DeviceRTC::DeviceRTC() : rtc() {}

/**
 * @brief Initialize the RTC
 */
void DeviceRTC::begin()
{
    if (_deviceRTCinitialized) return;
    if (!rtcI2C->initRTC())
    {
        logErrorP("RTC I2C initialization failed!");
        return;
    }
    else
    {
        logInfoP("RTC I2C initialized.");
    }
    if (!rtc.begin(rtcI2C->customI2C.get()))
    {
        logErrorP("Device RTC initialization failed!");
        return;
    }
    else
    {
        _deviceRTCinitialized = true;
        logInfoP("Device RTC initialized.");
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

    // TimeManager - Setup the RTC as the new time clock source
    OpenKNX::Time::TimeClock *timeClockRTC = new OpenKNX::Time::TimeClockRTC();
    openknx.time.setTimeClock(timeClockRTC, true);
    logInfoP("TimeManager - TimeClock RTC set.");
}

/**
 * @brief Get the current time from the RTC
 * @return DateTime
 */
DateTime DeviceRTC::now()
{
    return rtc.now();
}

/**
 * @brief Adjust the RTC to a new time
 * @param dt New DateTime to set
 */
void DeviceRTC::adjust(const DateTime &dt)
{
    rtc.adjust(dt);
}

/**
 * @brief Initialize the module
 */
void DeviceRTC::init()
{
    if (!_deviceRTCinitialized)
    {
        logInfoP("DeviceRTC initialize...");
        return begin();
    }
    else
    {
        logInfoP("DeviceRTC already initialized.");
    }
}

/**
 * @brief Setup the module
 * @param configured Whether the module is configured
 */
void DeviceRTC::setup(bool configured)
{
    // Setup code for OpenKNX
    logInfoP("DeviceRTC setup with configured: %d", configured);
}

/**
 * @brief Process input KO
 * @param obj GroupObject to process
 */
void DeviceRTC::processInputKo(GroupObject &obj)
{
    // Process input KO for OpenKNX
    // logInfoP("Processing input KO: %s", obj.getName().c_str());
}

/**
 * @brief Show help for console commands
 */
void DeviceRTC::showHelp()
{
    // Show help information for OpenKNX
    openknx.console.printHelpLine("rtc", "Device RTC Control. Use 'rtc ?' for more.");
}

/**
 * @brief Process console commands
 * @param command Command to process
 * @param diagnose Whether to diagnose the command
 */
bool DeviceRTC::processCommand(const std::string command, bool diagnose)
{
    bool bRet = false;
    if ((!diagnose) && command.compare(0, 4, "rtc ") == 0)
    {
        bRet = true;                            // Ok, we are in rtc command!
        if (command.compare(4, 4, "test") == 0) // rtc test
        {
            testRTC();
        }
        else if (command.compare(4, 4, "time") == 0 || (command.length() == 4)) // rtc time
        {
            logCurrentTime();
        }
        else if (command.compare(4, 6, "stime ") == 0) // rtc stime HH:MM:SS
        {
            logInfoP("Setting time: %s", command.substr(10).c_str());
            setTime(command.substr(10));
        }
        else if (command.compare(4, 6, "sdate ") == 0) // rtc sdate DD:MM:YYYY
        {
            // rtc sdate DD:MM:YYYY
            logInfoP("Setting date: %s", command.substr(10).c_str());
            setDate(command.substr(10));
        }
        else if (command.compare(4, 4, "scan") == 0) // rtc scan
        {
            int8_t devices[128];
            if (rtcI2C->scanI2C(devices))
            {
                logInfoP("I2C devices found:");
                for (int i = 0; devices[i] != -1; i++)
                {
                    logInfoP("Device at address 0x%02X", devices[i]);
                }
            }
            else
            {
                logErrorP("No I2C devices found.");
            }
        }
#ifdef ENABLE_TEMPERATURE
        else if (command.compare(4, 4, "temp") == 0) // rtc temp
        {
            float temp = getTemperature();
            logInfoP("RTC temperature: %.2f C", temp);
        }
#endif // ENABLE_TEMPERATURE
#ifdef ENABLE_EEPROM
        else if (command.compare(4, 7, "eeprom") == 0) // rtc testeprm
        {
            testEEPROM();
        }
        else if (command.compare(4, 8, "eepromw ") == 0) // rtc weprm <address> <data>
        {
            int address, data;
            if (sscanf(command.c_str() + 12, "%d %d", &address, &data) == 2)
            {
                writeEEPROM(address, data);
                logInfoP("Written %d to EEPROM address %d", data, address);
            }
            else
            {
                logErrorP("Invalid command format. Use: rtc eepromw <address> <data>");
                bRet = false;
            }
        }
        else if (command.compare(4, 8, "eepromr ") == 0) // rtc read eeprom <address>
        {
            int address;
            if (sscanf(command.c_str() + 12, "%d", &address) == 1)
            {
                uint8_t data = readEEPROM(address);
                logInfoP("Read %d from EEPROM address %d", data, address);
            }
            else
            {
                logErrorP("Invalid command format. Use: rtc eepromr <address>");
                bRet = false;
            }
        }
#endif // ENABLE_EEPROM
#ifdef ENABLE_ALARMS
        else if (command.compare(4, 3, "sa ") == 0) // rtc sa 1 2 12 12:12:12 12
        {
            logDebugP("Command: %s", command.c_str() + 7);
            uint8_t alarmType, dayOfWeek, dayOfMonth, hour, minute, second, alarmMode;
            const uint8_t retScanf = sscanf(command.c_str() + 7, "%hhu %hhu %hhu %hhu:%hhu:%hhu %hhu",
                                            &alarmType, &dayOfWeek, &dayOfMonth, &hour, &minute, &second, &alarmMode);
            if (retScanf == 7)
            {
                logInfoP("Setting to set Alarm %d: %02d:%02d:%02d DoW: %d DoM: %d Mode: %d",
                         alarmType, hour, minute, second, dayOfWeek, dayOfMonth, alarmMode);

                DeviceRTC::Alarm alarm = {
                    .type = static_cast<DeviceRTC::AlarmType>(alarmType),
                    .hour = hour,
                    .minute = minute,
                    .second = second,
                    .dayOfMonth = dayOfMonth,
                    .dayOfWeek = static_cast<DeviceRTC::Alarm::DayOfWeek>(dayOfWeek),
                    .modeAlarm1 = (alarmType == 1) ? static_cast<Ds3231Alarm1Mode>(alarmMode) : Ds3231Alarm1Mode::DS3231_A1_Second, // Default to second
                    .modeAlarm2 = (alarmType == 2) ? static_cast<Ds3231Alarm2Mode>(alarmMode) : Ds3231Alarm2Mode::DS3231_A2_Minute  // Default to minute
                };

                if (setAlarm(alarm))
                    logInfoP("Alarm %d Set to: %02d:%02d:%02d DoW: %d DoM: %d Mode: %d",
                             alarm.type, alarm.hour, alarm.minute, alarm.second,
                             alarm.dayOfWeek, alarm.dayOfMonth, alarm.modeAlarm1);
                else
                    logErrorP("Failed to set alarm.");
            }
            else // Show help if the command is not correct
            {
                logErrorP("Invalid Alarm command format.");
                logErrorP("Use: rtc sa <AlarmType> <DayOfWeek> <dayOfMonth> <HH:MM:SS> <AlarmMode>");
                openknx.logger.begin();
                openknx.logger.color(CONSOLE_HEADLINE_COLOR);

                // Allgemeine Hilfe
                openknx.logger.log("rtc sa <AlarmType> <DayOfWeek> <DayOfMonth> <HH:MM:SS> <AlarmMode>");
                openknx.logger.log("  AlarmType: 1 or 2 for Alarm1 or Alarm2");
                openknx.logger.log("  DayOfWeek: 0-6 for Sunday to Saturday");
                openknx.logger.log("  DayOfMonth: 1-31 or 0 for every day");
                openknx.logger.log("  AlarmMode: Modes vary for Alarm1 and Alarm2");

                // Alarm 1 Modi
                openknx.logger.log("Alarm 1 Modes:");
                openknx.logger.log("  15 Alarm once per second");
                openknx.logger.log("  14 Alarm when seconds match");
                openknx.logger.log("  12 Alarm when minutes and seconds match");
                openknx.logger.log("   8 Alarm when hours, minutes and seconds match");
                openknx.logger.log("   0 Alarm when date (day of month), hours, minutes and seconds match");
                openknx.logger.log("  16 Alarm when day (day of week), hours, minutes and seconds match");

                // Beispiel für Alarm 1
                openknx.logger.log("Example: rtc sa 1 2 31 11:12:13 14");
                openknx.logger.log("         (Alarm 1 when seconds match)");

                // Alarm 2 Modi
                openknx.logger.log("Alarm 2 Modes:");
                openknx.logger.log("   7 Alarm once per minute (when seconds are 0)");
                openknx.logger.log("   6 Alarm when minutes match");
                openknx.logger.log("   4 Alarm when hours and minutes match");
                openknx.logger.log("   0 Alarm when date (day of month), hours and minutes match");
                openknx.logger.log("   8 Alarm when day (day of week), hours and minutes match");

                // Beispiel für Alarm 2
                openknx.logger.log("Example: rtc sa 2 1 0 23:30:00 6");
                openknx.logger.log("         (Alarm 2 when minutes match)");

                openknx.logger.color(0);
                openknx.logger.end();
                return true;
            }
        }
        else if (command.compare(4, 3, "ga ") == 0) // rtc ga 1
        {

            uint8_t alarm;
            const uint8_t retScanf = sscanf(command.c_str() + 7, "%hhu", &alarm);
            if (retScanf == 1)
            {
                logInfoP("Getting Alarm %d", alarm);
                logAlarm(static_cast<AlarmType>(alarm));
            }
            else
            {
                // Show both alarm information if no alarm number is provided
                logAlarm(static_cast<AlarmType>(1));
                logAlarm(static_cast<AlarmType>(2));
            }
        }
        else if (command.compare(4, 3, "clr") == 0) // rtc clr 1
        {
            uint8_t force = 0;
            sscanf(command.c_str() + 7, "%hhu", &force);
            std::function<void(AlarmType, const char *)> logAlarmStatus = [&](AlarmType type, const char *alarmName) {
                if (checkAndClearAlarm(type, force == 1))
                    logInfoP("%s was triggered and %s.", alarmName, force == 1 ? "cleared" : "not cleared");
                else
                    logInfoP("%s was not triggered.", alarmName);
            };
            logAlarmStatus(AlarmType::Alarm1, "Alarm 1");
            logAlarmStatus(AlarmType::Alarm2, "Alarm 2");
        }

#endif // ENABLE_ALARMS
        else
        {
            openknx.logger.begin();
            openknx.logger.log("");
            openknx.logger.color(CONSOLE_HEADLINE_COLOR);
            openknx.logger.log("======================= Help: Device RTC Control ===========================");
            openknx.logger.log("Command(s)               Description");
            openknx.logger.color(0);
            openknx.console.printHelpLine("rtc time", "Show the current time from RTC");
            openknx.console.printHelpLine("rtc stime HH:MM:SS", "Set the time (24-hour format)");
            openknx.console.printHelpLine("rtc sdate DD:MM:YYYY", "Set the date");
#ifdef ENABLE_TEMPERATURE
            openknx.console.printHelpLine("rtc temp", "Get the temperature from RTC");
#endif
#ifdef ENABLE_EEPROM
            openknx.console.printHelpLine("rtc eepromw <adr> <dt>", "Write data to RTC EEPROM (adress 0-55 - data 0-255)");
            openknx.console.printHelpLine("rtc eepromr <adr>", "Read data from RTC EEPROM (0-55)");
            openknx.console.printHelpLine("rtc eeprom", "Test internal EEPROM with read/write");
#endif
#ifdef ENABLE_ALARMS
            openknx.console.printHelpLine("rtc sa", "Set an alarm. Use 'sa ?' for more.");
            openknx.console.printHelpLine("rtc ga <AlarmType>", "Show alarm 1 or 2 settings");
            openknx.console.printHelpLine("rtc clr <force>", "Check and Clear the alarm flag!");
#endif
            openknx.console.printHelpLine("rtc scan", "Scan the I2C bus for devices");
            openknx.console.printHelpLine("rtc test", "Will test if the RTC is running");
            openknx.logger.color(CONSOLE_HEADLINE_COLOR);
            openknx.logger.log("--------------------------------------------------------------------------------");
            openknx.logger.color(0);
            openknx.logger.end();
        }
    } // Command starts with "rtc "
    return bRet;
}

/**
 * @brief Set I2C settings
 * @param i2cInst I2C instance
 * @param scl SCL pin
 * @param sda SDA pin
 * @param address I2C address
 */
void DeviceRTC::setI2CSettings(i2c_inst_t* i2cInst, uint8_t scl, uint8_t sda, uint8_t address,
                               uint8_t addresseeprom, uint16_t eepromSize)
{
    i2cRTC::RTCSettings settings;
    settings.scl = scl;
    settings.sda = sda;
    settings.i2cAddress = address;
    settings.i2cAddressEEPROM = addresseeprom;
    settings.i2cEEPROMSize = eepromSize;
    settings.i2cInst = i2cInst;
    rtcI2C->initRTC(settings);
}

/**
 * @brief Test the RTC functionality
 */
void DeviceRTC::testRTC()
{
    logInfoP("Testing RTC...");
    // Check if the RTC lost power
    DateTime now = rtc.now();
    if (rtc.lostPower()) logErrorP("RTC lost power!");
    else
        logInfoP("RTC power OK.");
    // Check if the RTC is running
    if (!rtc.begin(rtcI2C->customI2C.get())) logErrorP("RTC not running!");
    else
        logInfoP("RTC running.");

    logInfoP("Unixtime: %lu", rtc.now().unixtime());
    logInfoP("Current time: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
    logInfoP("Current date: %02d-%02d-%02d", now.day(), now.month(), now.year());
    logInfoP("RTC square wave output: %s", rtc.readSqwPinMode() == DS3231_OFF ? "Off" : "On");
#ifdef ENABLE_TEMPERATURE
    logInfoP("Temperature: %.2f C", rtc.getTemperature());
#endif
#ifdef ENABLE_EEPROM
    logInfoP("EEPROM Size: %d bytes", rtcI2C->rtcSettings.i2cEEPROMSize);
#endif
#ifdef ENABLE_ALARMS
    logAlarm(AlarmType::Alarm1);
    logAlarm(AlarmType::Alarm2);
#endif

    logInfoP("RTC test complete.");
}

/**
 * @brief Log the current time from the RTC
 */
void DeviceRTC::logCurrentTime()
{
    DateTime now = rtc.now();
    logInfoP("RTC current time: %02d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
}

/**
 * @brief Set the time
 * @param time Time string in HH:MM:SS format
 */
void DeviceRTC::setTime(const std::string &time)
{
    uint8_t hour, minute, second;
    if (sscanf(time.c_str(), "%hhu:%hhu:%hhu", &hour, &minute, &second) == 3)
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
 * @param date Date string in DD:MM:YYYY format
 */
void DeviceRTC::setDate(const std::string &date)
{
    uint8_t day, month, year;
    if (sscanf(date.c_str(), "%hhu:%hhu:%hhu", &day, &month, &year) == 3)
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

/**
 * @brief Read the current time from the RTC
 * @return time_t in Unix time format (seconds since 1970). i.e. 05.06.2025 00:00:01 = 1740000001
 */
time_t DeviceRTC::getTime()
{
    return rtc.now().unixtime();
}

#ifdef ENABLE_TEMPERATURE
/**
 * @brief Get the temperature from the RTC
 * @return float Temperature in Celsius
 */
float DeviceRTC::getTemperature()
{
    return rtc.getTemperature();
}
#endif // ENABLE_TEMPERATURE
#ifdef ENABLE_EEPROM
/**
 * @brief Test the EEPROM functionality
 * @warning This test will overwrite data in the EEPROM!!
 */
void DeviceRTC::testEEPROM()
{
    logInfoP("EEPROM Read/Write test...");
    logInfoP("EEPROM Size: %d bytes", rtcI2C->rtcSettings.i2cEEPROMSize);
    uint32_t startTime = millis();
    bool errorFound = false;
    uint32_t bytesWritten = 0;

    // Define our test ranges of the EEPROM. We will test the first 256 bytes,
    // the quarter mark, the half mark, and the last 256 bytes of the EEPROM.
    uint16_t testRanges[][2] = {
        {0x0000, 0x00FF},                                                                                                                    // Test first 256 bytes
        {static_cast<uint16_t>(rtcI2C->rtcSettings.i2cEEPROMSize / 4), static_cast<uint16_t>(rtcI2C->rtcSettings.i2cEEPROMSize / 4 + 0xFF)}, // Test quarter mark
        {static_cast<uint16_t>(rtcI2C->rtcSettings.i2cEEPROMSize / 2), static_cast<uint16_t>(rtcI2C->rtcSettings.i2cEEPROMSize / 2 + 0xFF)}, // Test half mark
        {static_cast<uint16_t>(rtcI2C->rtcSettings.i2cEEPROMSize - 0x100), static_cast<uint16_t>(rtcI2C->rtcSettings.i2cEEPROMSize - 1)}     // Test last 256 bytes
    };

    for (uint16_t i = 0; i < sizeof(testRanges) / sizeof(testRanges[0]); i++)
    {
        uint16_t *range = testRanges[i];
        for (uint16_t i = range[0]; i <= range[1]; i++)
        {
            writeEEPROM(i, i % 256); // Write test data to EEPROM. Data will be 0x00 to 0xFF
            delay(10);               // Ensure write delay, to prevent data corruption
            bytesWritten++;          // Increment total bytes written
            if (i % 256 == 0)        // Log every 256 bytes written
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

    for (uint16_t i = 0; i < sizeof(testRanges) / sizeof(testRanges[0]); i++)
    {
        uint16_t *range = testRanges[i];
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
 * @param address EEPROM address to write to
 * @param data Data to write
 */
void DeviceRTC::writeEEPROM(uint16_t address, uint8_t data)
{
    if (address >= rtcI2C->rtcSettings.i2cEEPROMSize)
    { // Check against defined EEPROM size
        logErrorP("Invalid EEPROM address. Must be between 0x000 and 0xFFF.");
        return;
    }
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddressEEPROM);
    rtcI2C->customI2C->write((address >> 8) & 0xFF); // MSB
    rtcI2C->customI2C->write(address & 0xFF);        // LSB
    rtcI2C->customI2C->write(data);
    rtcI2C->customI2C->endTransmission();
    delay(10); // EEPROM write delay
    //logDebugP("Written 0x%02X to EEPROM address 0x%04X", data, address);
}

/**
 * @brief Read data from the external EEPROM
 * @param address EEPROM address to read from
 * @return uint8_t Data read from EEPROM
 */
uint8_t DeviceRTC::readEEPROM(uint16_t address)
{
    if (address >= rtcI2C->rtcSettings.i2cEEPROMSize)
    { // Check against defined EEPROM size
        logErrorP("Invalid EEPROM address. Must be between 0x000 and 0xFFF.");
        return 0;
    }
    rtcI2C->customI2C->beginTransmission(rtcI2C->rtcSettings.i2cAddressEEPROM);
    rtcI2C->customI2C->write((address >> 8) & 0xFF); // MSB
    rtcI2C->customI2C->write(address & 0xFF);        // LSB
    rtcI2C->customI2C->endTransmission();
    rtcI2C->customI2C->requestFrom(rtcI2C->rtcSettings.i2cAddressEEPROM, (uint8_t)1);
    if (rtcI2C->customI2C->available())
    {
        uint8_t data = rtcI2C->customI2C->read();
        // logDebugP("Read 0x%02X from EEPROM address 0x%04X", data, address);
        return data;
    }
    else
    {
        logErrorP("EEPROM read error at address 0x%04X", address);
        return 0;
    }
}
#endif // ENABLE_EEPROM

#ifdef ENABLE_ALARMS

/**
 * @brief Return the alarm information to the console
 */
void DeviceRTC::logAlarm(AlarmType alarmType)
{
    if (alarmType != AlarmType::Alarm1 && alarmType != AlarmType::Alarm2)
    {
        logInfoP("Invalid alarm type: %d", alarmType);
        return;
    }
    Alarm alarm = getAlarm(alarmType);

    logInfoP("Setting Alarm : %d", alarmType);
    String alarmMode = "";
    if (alarmType == AlarmType::Alarm1)
    {
        logInfoP("Clock         : %02d:%02d:%02d (HH:MM:SS)", alarm.hour, alarm.minute, alarm.second);
        if (alarm.modeAlarm1 == Ds3231Alarm1Mode::DS3231_A1_PerSecond) alarmMode += "PerSecond";
        else if (alarm.modeAlarm1 == Ds3231Alarm1Mode::DS3231_A1_Second)
            alarmMode += "Second";
        else if (alarm.modeAlarm1 == Ds3231Alarm1Mode::DS3231_A1_Minute)
            alarmMode += "Minute";
        else if (alarm.modeAlarm1 == Ds3231Alarm1Mode::DS3231_A1_Hour)
            alarmMode += "Hour";
        else if (alarm.modeAlarm1 == Ds3231Alarm1Mode::DS3231_A1_Date)
            alarmMode += "Date";
        else if (alarm.modeAlarm1 == Ds3231Alarm1Mode::DS3231_A1_Day)
            alarmMode += "Day";
    }
    else
    {
        logInfoP("Clock         : %02d:%02d", alarm.hour, alarm.minute);
        if (alarm.modeAlarm2 == Ds3231Alarm2Mode::DS3231_A2_Minute) alarmMode += "Minute";
        else if (alarm.modeAlarm2 == Ds3231Alarm2Mode::DS3231_A2_Hour)
            alarmMode += "Hour";
        else if (alarm.modeAlarm2 == Ds3231Alarm2Mode::DS3231_A2_Date)
            alarmMode += "Date";
        else if (alarm.modeAlarm2 == Ds3231Alarm2Mode::DS3231_A2_Day)
            alarmMode += "Day";
    }
    // Day of Week is only used for Alarm1
    const char *dayOfWeek[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    logInfoP("Mode          : %s", alarmMode.c_str());
    logInfoP("Day of Week   : %s", dayOfWeek[alarm.dayOfWeek]);
    logInfoP("Day of Month  : %d", alarm.dayOfMonth);
    logInfoP("Alarm Status  : %s", alarm.alarmEnabled ? "Triggered" : "Not Triggered");
}

/**
 * @brief Set the alarm to the given time and mode (Alarm1 or Alarm2)
 * @param alarm Filled Alarm struct
 * @return setAlarm from RTC
 */
bool DeviceRTC::setAlarm(Alarm alarm)
{
    if (alarm.type == AlarmType::Alarm1)
    {
        return rtc.setAlarm1(
            DateTime(
                rtc.now().year(),
                rtc.now().month(),
                ((alarm.modeAlarm1 == Ds3231Alarm1Mode::DS3231_A1_Day) ? alarm.dayOfWeek : alarm.dayOfMonth),
                alarm.hour,
                alarm.minute,
                alarm.second),
            alarm.modeAlarm1);
    }
    else if (alarm.type == AlarmType::Alarm2)
    {
        return rtc.setAlarm2(
            DateTime(
                rtc.now().year(),
                rtc.now().month(),
                ((alarm.modeAlarm2 == Ds3231Alarm2Mode::DS3231_A2_Day) ? alarm.dayOfWeek : alarm.dayOfMonth),
                alarm.hour,
                alarm.minute, 0),
            alarm.modeAlarm2);
    }

    return false; // Return false if an unknown alarm type is passed
}

/**
 * @brief Get the alarm settings
 * @param alarmType AlarmType (Alarm1 or Alarm2)
 * @return DeviceRTC::Alarm
 */
DeviceRTC::Alarm DeviceRTC::getAlarm(AlarmType alarmType)
{
    if (alarmType != AlarmType::Alarm1 && alarmType != AlarmType::Alarm2)
    {
        logInfoP("Invalid alarm type: %d", alarmType);
        return {};
    }
    Alarm alarm;
    if (alarmType == AlarmType::Alarm1)
    {
        alarm.type = AlarmType::Alarm1;
        alarm.hour = rtc.getAlarm1().hour();
        alarm.minute = rtc.getAlarm1().minute();
        alarm.second = rtc.getAlarm1().second();
        alarm.dayOfMonth = rtc.getAlarm1().day();
        alarm.dayOfWeek = static_cast<Alarm::DayOfWeek>(rtc.getAlarm1().dayOfTheWeek());
        alarm.modeAlarm1 = rtc.getAlarm1Mode();
        alarm.alarmEnabled = rtc.alarmFired(Alarm1);
    }
    else
    {
        alarm.type = AlarmType::Alarm2;
        alarm.hour = rtc.getAlarm2().hour();
        alarm.minute = rtc.getAlarm2().minute();
        alarm.second = 0;
        alarm.dayOfMonth = rtc.getAlarm2().day();
        alarm.dayOfWeek = static_cast<Alarm::DayOfWeek>(rtc.getAlarm2().dayOfTheWeek());
        alarm.modeAlarm2 = rtc.getAlarm2Mode();
        alarm.alarmEnabled = rtc.alarmFired(Alarm2);
    }
    return alarm;
}

/**
 * @brief Check if an alarm was triggered and clear the alarm status
 * @param type AlarmType (Alarm1 or Alarm2)
 * @param force Force clear the alarm status
 * @return bool True if the alarm was triggered, false otherwise
 */
bool DeviceRTC::checkAndClearAlarm(AlarmType type, bool force)
{
    if (type == AlarmType::Alarm1)
    {
        if (rtc.alarmFired(Alarm1) || force)
        {
            rtc.clearAlarm(Alarm1);
            return true;
        }
    }
    else
    {
        if (rtc.alarmFired(Alarm2) || force)
        {
            rtc.clearAlarm(Alarm2);
            return true;
        }
    }
    return false;
}
#endif // ENABLE_ALARMS