#include <fstream>
#include <string>

extern std::string globalConfigName;

void appendTestResult(const std::string& message) {
    std::ofstream out{ "testResult.log", std::ios::app };
    out << "========== " << globalConfigName << " ==========" << std::endl;
    out << message << std::endl;
    out.flush();
}
