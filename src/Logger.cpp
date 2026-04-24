#include "Logger.hpp"

#include <iostream>

ProcessingLog::ProcessingLog(const std::string& log_file) {
    log_stream_.open(log_file, std::ios::app);
    if (!log_stream_.is_open()) {
        std::cerr << "Failed to open log file: " << log_file << "\n";
    } else {
        log_stream_ << "--- Processing Session Started at " << getCurrentTime() << " ---" << "\n";
    }
}

ProcessingLog::~ProcessingLog() {
    if (log_stream_.is_open()) {
        log_stream_ << "--- Processing Session Ended at " << getCurrentTime() << " ---\n\n";
        log_stream_.close();
    }
}

void ProcessingLog::log(const std::string& operation, size_t original_count, size_t current_count) {
    if (log_stream_.is_open()) {
        log_stream_ << "[" << getCurrentTime() << "] Operation: " << operation << " | Pts before: " << original_count
                    << " | Pts after: " << current_count
                    << " | Diff: " << static_cast<long long>(current_count) - static_cast<long long>(original_count)
                    << "\n";
        log_stream_.flush();
    }
}

std::string ProcessingLog::getCurrentTime() const {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm bt{};
#if defined(_WIN32)
    localtime_s(&bt, &time_t_now);
#else
    localtime_r(&time_t_now, &bt);
#endif
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}
