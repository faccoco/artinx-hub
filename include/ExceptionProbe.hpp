#pragma once
#include "Common.hpp"
#include <exception>

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
