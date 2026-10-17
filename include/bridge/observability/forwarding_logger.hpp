#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

namespace bridge::observability {

class ForwardingLogger {
 public:
  void info(const std::string& line) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensure_open_locked();
    rotate_if_needed_locked(line.size() + 1U);
    stream_ << line << '\n';
  }

  void flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
      stream_.flush();
    }
  }

 private:
  void ensure_open_locked() {
    if (stream_.is_open()) {
      return;
    }
    std::filesystem::create_directories(kLogDirectory);
    stream_.open(kLogPath, std::ios::out | std::ios::app);
  }

  void rotate_if_needed_locked(const std::size_t next_write_size) {
    if (!stream_.is_open()) {
      return;
    }
    stream_.flush();

    std::error_code error;
    const std::uintmax_t current_size = std::filesystem::file_size(kLogPath, error);
    if (error || current_size + next_write_size <= kMaxFileSizeBytes) {
      return;
    }

    stream_.close();
    std::filesystem::remove(kThirdRotatedLogPath, error);
    std::filesystem::rename(kSecondRotatedLogPath, kThirdRotatedLogPath, error);
    std::filesystem::rename(kFirstRotatedLogPath, kSecondRotatedLogPath, error);
    std::filesystem::rename(kLogPath, kFirstRotatedLogPath, error);
    stream_.open(kLogPath, std::ios::out | std::ios::trunc);
  }

  static constexpr const char* kLogDirectory = "logs";
  static constexpr const char* kLogPath = "logs/g1_forwarding.log";
  static constexpr const char* kFirstRotatedLogPath = "logs/g1_forwarding.1.log";
  static constexpr const char* kSecondRotatedLogPath = "logs/g1_forwarding.2.log";
  static constexpr const char* kThirdRotatedLogPath = "logs/g1_forwarding.3.log";
  static constexpr std::uintmax_t kMaxFileSizeBytes = 2U * 1024U * 1024U;

  std::mutex mutex_;
  std::ofstream stream_;
};

[[nodiscard]] inline std::shared_ptr<ForwardingLogger> forwarding_logger() {
  static const std::shared_ptr<ForwardingLogger> logger =
      std::make_shared<ForwardingLogger>();
  return logger;
}

}  // namespace bridge::observability
