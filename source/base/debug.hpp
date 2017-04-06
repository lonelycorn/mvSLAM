#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cstdio>
#include <cstdarg>
#include <cassert>

namespace mvSLAM
{
#define LOGGING_FORMAT_STRING_BUFFER_SIZE 1024
#define LOGGING_TAG_DELIMITER ' '

enum class LoggingLevel
{
    NONE = 0,
    ERROR = 1,
    INFO = 2,
    DEBUG = 3,
    COUNT = 4,
};

template <typename OutputStreamType, typename T>
void __do_stream_write(OutputStreamType &output_stream, T t)
{
    output_stream << t;
}

template <typename OutputStreamType, typename T, typename ... Args>
void __do_stream_write(OutputStreamType &output_stream, T t, Args... args)
{
    output_stream << t;
    __do_stream_write(output_stream, args...);
}

/// write formatted data to stream. atomic operation.
template <typename OutputStreamType>
void __stream_format_write_data(OutputStreamType &output_stream,
                                const char *format, ...)
{
    char buffer[LOGGING_FORMAT_STRING_BUFFER_SIZE+1]; // extra byte for \n
    va_list args;
    va_start(args, format);
    int result = vsnprintf(buffer, LOGGING_FORMAT_STRING_BUFFER_SIZE, format, args);
    va_end(args);

    assert(result >= 0);
    if (result < LOGGING_FORMAT_STRING_BUFFER_SIZE)
    {
        buffer[result] = '\n';
        buffer[result+1] = '\0';
    }

    __do_stream_write(output_stream, buffer);
}

/// write data to stream. atomic operation.
template <typename OutputStreamType, typename ... Args>
void __stream_write_data(OutputStreamType &output_stream, Args... args)
{
    std::stringstream ss;
    __do_stream_write(ss, args...);
    __do_stream_write(ss, '\n');

    __do_stream_write(output_stream, ss.str());
}

class Logging
{
public:
    // only messages smaller or equal to @p level will be actually logged
    static void set_logging_level(LoggingLevel level)
    {
        assert((LoggingLevel::NONE <= level) && (level < LoggingLevel::COUNT));
        m_level = level;
    }

    // redirect
    static void set_debug_stream(std::ostream &debug_stream)
    {
        m_debug_stream = &debug_stream;
    }

    static void set_info_stream(std::ostream &info_stream)
    {
        m_info_stream = &info_stream;
    }

    static void set_error_stream(std::ostream &error_stream)
    {
        m_error_stream = &error_stream;
    }

    template <typename... Args>
    static void debug(Args... args)
    {
        if (m_level < LoggingLevel::DEBUG)
        {
            return;
        }
        __stream_write_data(*m_debug_stream, args...);
    }

    template <typename... Args>
    static void info(Args... args)
    {
        if (m_level < LoggingLevel::INFO)
        {
            return;
        }
        __stream_write_data(*m_info_stream, args...);
    }

    template <typename... Args>
    static void error(Args... args)
    {
        if (m_level < LoggingLevel::ERROR)
        {
            return;
        }
        __stream_write_data(*m_error_stream, args...);
    }

    static void debugf(const char *fmt, ...)
    {
        if (m_level < LoggingLevel::DEBUG)
        {
            return;
        }
        va_list args;
        va_start(args, fmt);
        __stream_format_write_data(*m_debug_stream, fmt, args);
        va_end(args);
    }

    static void infof(const char *fmt, ...)
    {
        if (m_level < LoggingLevel::INFO)
        {
            return;
        }
        va_list args;
        va_start(args, fmt);
        __stream_format_write_data(*m_info_stream, fmt, args);
        va_end(args);
    }

    static void errorf(const char *fmt, ...)
    {
        if (m_level < LoggingLevel::ERROR)
        {
            return;
        }
        va_list args;
        va_start(args, fmt);
        __stream_format_write_data(*m_error_stream, fmt, args);
        va_end(args);
    }
private:
    static LoggingLevel m_level;
    static std::ostream *m_debug_stream;
    static std::ostream *m_info_stream;
    static std::ostream *m_error_stream;
};


class Logger
{
public:
    Logger(const char *tag, bool enabled = true):
        m_tag(tag), m_enabled(enabled)
    {
        m_tag += LOGGING_TAG_DELIMITER;
    }
    ~Logger()
    {
    }
#define MAKE_LOGGING_CHANNEL(CHANNEL)\
    template <typename ...Args> \
    void CHANNEL(Args... args) \
    { \
        if (m_enabled) \
        { \
            Logging::CHANNEL(m_tag, args...); \
        } \
    }

    MAKE_LOGGING_CHANNEL(debug);
    MAKE_LOGGING_CHANNEL(info);
    MAKE_LOGGING_CHANNEL(error);
#undef MAKE_LOGGING_CHANNEL

#define MAKE_LOGGING_CHANNEL(CHANNEL) \
    void CHANNEL(const char *fmt, ...) \
    { \
        if (m_enabled) \
        { \
            std::string fmt_with_tag(fmt); \
            fmt_with_tag = m_tag + fmt_with_tag; \
            va_list args; \
            va_start(args, fmt); \
            Logging::CHANNEL(fmt_with_tag.c_str(), args); \
            va_end(args); \
        } \
    }
    MAKE_LOGGING_CHANNEL(debugf);
    MAKE_LOGGING_CHANNEL(infof);
    MAKE_LOGGING_CHANNEL(errorf);
#undef MAKE_LOGGING_CHANNEL

private:
    std::string m_tag;
    bool m_enabled;
};

}
