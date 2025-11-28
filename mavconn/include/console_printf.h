#pragma once

// 这个是ros系统库，一般在/usr/include/console_bridge文件下
#include <console_bridge/console.h>

#ifndef CONSOLE_BRIDGE_logDebug
#define CONSOLE_BRIDGE_logDebug(fmt, ...) \
            console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, fmt, ##__VA_ARGS__)
#endif

#ifndef CONSOLE_BRIDGE_logInform
#define CONSOLE_BRIDGE_logInfrom(fmt, ...) \
            console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_INFO, fmt, ##__VA_ARGS__)
#endif

#ifndef CONSOLE_BRIDGE_logWarn
#define CONSOLE_BRIDGE_logWarn(fmt, ...) \
            console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_WARN, fmt, ##__VA_ARGS__)
#endif

#ifndef CONSOLE_BRIDGE_logError
#define CONSOLE_BRIDGE_logError(fmt, ...) \
            console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)
#endif

/**
 * 日志初始化
 * @param CONSOLE_BRIDGE_LOG_DEBUG (0)
 * @param CONSOLE_BRIDGE_LOG_INFO  (1)    default
 * @param CONSOLE_BRIDGE_LOG_WARN  (2)
 * @param CONSOLE_BRIDGE_LOG_ERROR (3)
 * @param CONSOLE_BRIDGE_LOG_NONE  (4)
 */
void ConsoleInit(console_bridge::LogLevel level) {
    if (level == console_bridge::CONSOLE_BRIDGE_LOG_DEBUG) {
        console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    }
    else if(level == console_bridge::CONSOLE_BRIDGE_LOG_INFO) {
         console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_INFO);
    }
    else if(level == console_bridge::CONSOLE_BRIDGE_LOG_WARN) {
         console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_WARN);
    }
    else if(level == console_bridge::CONSOLE_BRIDGE_LOG_ERROR) {
         console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_ERROR);
    }
    else {
        console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_ERROR, "Ardupilot: %s", "Console log initialize failed ! \n Console log set default configure: [INFO].");
        console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_INFO);
    }
}

