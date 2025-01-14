#include <log/log.hh>
#include <vector>

std::string Logger::current_subsystem = "";
std::vector<std::shared_ptr<LogBackend>> Logger::backends;
std::deque<std::string> Logger::ee_log_buffer;

void Logger::set_subsystem(const std::string &subsystem)
{
    current_subsystem = subsystem;
}

void Logger::info(const std::string & message)
{
    log(LogLevel::Info, message);
}

void Logger::warn(const std::string &message)
{
    log(LogLevel::Warn, message);
}

void Logger::debug(const std::string &message)
{
    log(LogLevel::Debug, message);
}

void Logger::error(const std::string &message)
{
    log(LogLevel::Error, message);
}

void Logger::raw(const std::string &message)
{
    for (const auto &backend : backends)
    {
        backend->log(message, LogLevel::Raw);
    }
}

void Logger::ee_log(const std::string &message)
{
    for (const auto &backend : backends)
    {
        backend->log(message, LogLevel::EE_LOG);
    }

    ee_log_buffer.push_back(message);
    if (ee_log_buffer.size() > 100) {
        ee_log_buffer.pop_front();
    }
}

void Logger::log(LogLevel level, const std::string &message)
{
    std::string formatted_message = format_message(level, message);
    for (const auto &backend : backends)
    {
        backend->log(formatted_message, level);
    }
}

void Logger::add_backend(std::shared_ptr<LogBackend> backend)
{
    backends.push_back(backend);
}

void Logger::remove_backend(std::shared_ptr<LogBackend> backend)
{
    backends.erase(std::remove(backends.begin(), backends.end(), backend), backends.end());
}

std::string Logger::format_message(LogLevel level, const std::string &message)
{
    return "[" + current_subsystem + "] " + std::string(level_to_string(level)) + ": " + message;
}

const char *Logger::level_to_string(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warn:
        return "WARN";
    case LogLevel::Debug:
        return "DEBUG";
    case LogLevel::Error:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}
