// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <utility>

#include <fmt/format.h>

namespace wpi::util {

/**
 * Log levels for Logger.
 *
 * Log levels are used to filter log messages by severity. Higher numeric
 * values represent more critical messages.
 */
enum LogLevel {
  WPI_LOG_CRITICAL = 50,  ///< Critical errors that may cause system failure
  WPI_LOG_ERROR = 40,     ///< Errors that need attention
  WPI_LOG_WARNING = 30,   ///< Warning messages
  WPI_LOG_INFO = 20,      ///< Informational messages
  WPI_LOG_DEBUG = 10,     ///< General debug messages
  WPI_LOG_DEBUG1 = 9,     ///< Verbose debug messages (level 1)
  WPI_LOG_DEBUG2 = 8,     ///< Verbose debug messages (level 2)
  WPI_LOG_DEBUG3 = 7,     ///< Verbose debug messages (level 3)
  WPI_LOG_DEBUG4 = 6      ///< Verbose debug messages (level 4)
};

/**
 * A logger that calls a function with log messages.
 *
 * The Logger class provides a flexible logging mechanism that allows
 * custom log handling through a callback function. Log messages can be
 * filtered by minimum log level.
 */
class Logger {
 public:
  /**
   * Log callback function type.
   *
   * @param level Log level
   * @param file Source file name
   * @param line Line number in source file
   * @param msg Log message
   */
  using LogFunc = std::function<void(unsigned int level, const char* file,
                                     unsigned int line, const char* msg)>;

  /**
   * Constructs a Logger with no logging function.
   *
   * Log messages will be discarded until SetLogger() is called.
   */
  Logger() = default;

  /**
   * Constructs a Logger with the given logging function.
   *
   * @param func Function to call for log messages
   */
  explicit Logger(LogFunc func) : m_func(std::move(func)) {}

  /**
   * Constructs a Logger with the given logging function and minimum level.
   *
   * @param func Function to call for log messages
   * @param min_level Minimum log level to process
   */
  Logger(LogFunc func, unsigned int min_level)
      : m_func(std::move(func)), m_min_level(min_level) {}

  /**
   * Sets the logger function.
   *
   * @param func Function to call for log messages
   */
  void SetLogger(LogFunc func) { m_func = func; }

  /**
   * Sets the minimum log level.
   *
   * Log messages with a level lower than this will be discarded.
   *
   * @param level Minimum log level to process
   */
  void set_min_level(unsigned int level) { m_min_level = level; }

  /**
   * Gets the minimum log level.
   *
   * @return Minimum log level
   */
  unsigned int min_level() const { return m_min_level; }

  /**
   * Logs a message.
   *
   * This is the core logging function that calls the registered logger
   * function if the level meets the minimum threshold.
   *
   * @param level Log level
   * @param file Source file name
   * @param line Line number in source file
   * @param msg Log message
   */
  void DoLog(unsigned int level, const char* file, unsigned int line,
             const char* msg);

  /**
   * Logs a formatted message using fmt format string.
   *
   * @param level Log level
   * @param file Source file name
   * @param line Line number in source file
   * @param format Format string (fmt syntax)
   * @param args Format arguments
   */
  void LogV(unsigned int level, const char* file, unsigned int line,
            fmt::string_view format, fmt::format_args args);

  /**
   * Logs a formatted message using fmt format string.
   *
   * This template function provides type-safe formatted logging using
   * the fmt library syntax.
   *
   * @tparam Args Format argument types
   * @param level Log level
   * @param file Source file name
   * @param line Line number in source file
   * @param format Format string (fmt syntax)
   * @param args Format arguments
   */
  template <typename... Args>
  void Log(unsigned int level, const char* file, unsigned int line,
           fmt::string_view format, Args&&... args) {
    if (m_func && level >= m_min_level) {
      LogV(level, file, line, format, fmt::make_format_args(args...));
    }
  }

  /**
   * Returns whether a logger function is set.
   *
   * @return True if a logger function is set, false otherwise
   */
  bool HasLogger() const { return m_func != nullptr; }

 private:
  LogFunc m_func;
  unsigned int m_min_level = 20;
};

// C++20 relaxed the number of arguments to variadics, but Apple Clang's
// warnings haven't caught up yet: https://stackoverflow.com/a/67996331
#ifdef __clang__
#pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#endif

/**
 * Log a message with the specified level.
 *
 * This macro logs a message if the logger has a function set and the level
 * meets the minimum threshold. The file name and line number are
 * automatically captured.
 *
 * Example:
 * @code
 * Logger logger{[](unsigned int level, const char* file, unsigned int line,
 *                  const char* msg) {
 *   std::cout << msg << std::endl;
 * }};
 * WPI_LOG(logger, WPI_LOG_INFO, "Value is {}", 42);
 * @endcode
 *
 * @param logger_inst Logger instance
 * @param level Log level
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_LOG(logger_inst, level, format, ...)                            \
  if ((logger_inst).HasLogger() && level >= (logger_inst).min_level()) {    \
    (logger_inst)                                                           \
        .Log(level, __FILE__, __LINE__, format __VA_OPT__(, ) __VA_ARGS__); \
  }

/**
 * Log an error message.
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_ERROR(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_ERROR, format __VA_OPT__(, ) __VA_ARGS__)

/**
 * Log a warning message.
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_WARNING(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_WARNING, format __VA_OPT__(, ) __VA_ARGS__)

/**
 * Log an informational message.
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_INFO(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_INFO, format __VA_OPT__(, ) __VA_ARGS__)

/**
 * Log a debug message.
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_DEBUG(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_DEBUG, format __VA_OPT__(, ) __VA_ARGS__)

/**
 * Log a verbose debug message (level 1).
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_DEBUG1(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_DEBUG1, format __VA_OPT__(, ) __VA_ARGS__)

/**
 * Log a verbose debug message (level 2).
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_DEBUG2(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_DEBUG2, format __VA_OPT__(, ) __VA_ARGS__)

/**
 * Log a verbose debug message (level 3).
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_DEBUG3(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_DEBUG3, format __VA_OPT__(, ) __VA_ARGS__)

/**
 * Log a verbose debug message (level 4).
 *
 * @param inst Logger instance
 * @param format Format string (fmt syntax)
 * @param ... Format arguments (optional)
 */
#define WPI_DEBUG4(inst, format, ...) \
  WPI_LOG(inst, ::wpi::util::WPI_LOG_DEBUG4, format __VA_OPT__(, ) __VA_ARGS__)

}  // namespace wpi::util
