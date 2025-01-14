#pragma once

#include <deque>
#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>

enum class LogLevel
{
    Info,
    Warn,
    Debug,
    Error,
	  Raw,
    EE_LOG
};

class LogBackend
{
  public:
    virtual void log(const std::string &message, LogLevel level) = 0;  // Add LogLevel parameter here
    virtual ~LogBackend() = default;
};

class Logger
{
  public:
    static void set_subsystem(const std::string &subsystem);

    static void info(const std::string & message);
    static void warn(const std::string &message);
    static void debug(const std::string &message);
    static void error(const std::string &message);

    static void raw(const std::string &message);
    static void ee_log(const std::string &message);

    static void add_backend(std::shared_ptr<LogBackend> backend);
    static void remove_backend(std::shared_ptr<LogBackend> backend);

	static const std::vector<std::shared_ptr<LogBackend>>& get_backends() {
        return backends;
    }

    static std::string current_subsystem;

  private:
    static void log(LogLevel level, const std::string &message);
    static std::string format_message(LogLevel level, const std::string &message);
    static const char *level_to_string(LogLevel level);

    static std::vector<std::shared_ptr<LogBackend>> backends;
    static std::deque<std::string> ee_log_buffer;
};
