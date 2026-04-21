#pragma once
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

class ProcessingLog {
public:
    ProcessingLog(const std::string& log_file);
    ~ProcessingLog();

    void log(const std::string& operation, size_t original_count, size_t current_count);

private:
    std::ofstream log_stream_;
    std::string getCurrentTime() const;
};
