#include <fstream>
#include <mutex>
#include <string>

static std::ostream& getTestResult() {
    static std::ofstream out{ "testResult.txt", std::ios::app };
    return out;
}

void appendTestResult(const std::string& message) {
    static std::mutex mutex;
    std::lock_guard guard{ mutex };
    getTestResult() << message << std::endl;
}
