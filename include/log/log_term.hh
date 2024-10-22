#pragma once
#include <log/log.hh>
#include <iostream>

class TerminalLogBackend : public LogBackend
{
  public:
    void log(const std::string &message, LogLevel level) override
    {
        std::string color;

        switch (level)
        {
        case LogLevel::Info:
            color = BOLDWHITE;
            break;
        case LogLevel::Warn:
            color = BOLDYELLOW;
            break;
        case LogLevel::Debug:
            color = BOLDCYAN;
            break;
        case LogLevel::Error:
            color = BOLDRED;
            break;
        default:
            color = WHITE;
        }

        if (level != LogLevel::Raw)
		{
			std::cout << color << message << RESET << std::endl;
		}
		else
		{
			std::cout << message;
		}
    }
};
