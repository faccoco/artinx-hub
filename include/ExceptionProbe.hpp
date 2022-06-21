#pragma once
#include "Common.hpp"
#include <exception>
#include <fmt/format.h>

class ExceptionProbe final {
    const char* mFile;
    const char* mFunction;
    const uint32_t mLine;

public:
    ExceptionProbe(const char* file, const char* function, const uint32_t line)
        : mFile{ file }, mFunction{ function }, mLine{ line } {}
    ExceptionProbe(const ExceptionProbe& rhs) = delete;
    ExceptionProbe& operator=(const ExceptionProbe& rhs) = delete;
    ExceptionProbe(ExceptionProbe&& rhs) = delete;
    ExceptionProbe& operator=(ExceptionProbe&& rhs) = delete;

    ~ExceptionProbe() {
#ifdef ARTINXHUB_DEBUG
        if(std::uncaught_exceptions()) {
#ifdef ARTINXHUB_WINDOWS
            __debugbreak();
#endif
        }
#endif
    }
};

#define ACTOR_EXCEPTION_PROBE()          \
    ExceptionProbe __probe {             \
        __FILE__, __FUNCTION__, __LINE__ \
    }

using namespace std::chrono_literals;

class LatencyProbe final {
    const char* mFile;
    const char* mFunction;
    const uint32_t mLine;

    static constexpr auto highLatency = 50ms;
    Clock::time_point mStart;

public:
    LatencyProbe(const char* file, const char* function, const uint32_t line)
        : mFile{ file }, mFunction{ function }, mLine{ line }, mStart{ Clock::now() } {}
    ~LatencyProbe() {
        if(Clock::now() - mStart > highLatency) {
            logWarning(fmt::format("High latency detected {} {} {}", mFile, mFunction, mLine));
        }
    }
};

#define ACTOR_LATENCY_PROBE()            \
    LatencyProbe __probe__ {             \
        __FILE__, __FUNCTION__, __LINE__ \
    }
