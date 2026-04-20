/*
 * Copyright 2026 Manifold Tech Ltd.(manifoldtech.cn)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "logger.hpp"
#include <iostream>
#include <filesystem>


namespace odin_rtk_fusion {

SpdlogManager& SpdlogManager::getInstance() {
  static SpdlogManager instance;
  return instance;
}

void SpdlogManager::initialize(const std::string& log_file, bool clear_old_logs) {
  if (initialized_) {
    return;
  }

  try {
    // If clearing old logs, delete all related log files
    if (clear_old_logs) {
      clearOldLogFiles(log_file);
    }

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] %v");

    // Create file output sink (rotating file)
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_file, 1024 * 1024 * 100 , 5);  // 100MB per file, 5 files max
    file_sink->set_level(spdlog::level::trace);
    file_sink->set_pattern("[%L] [%T] %v");

    // Create default logger
    default_logger_ = std::make_shared<spdlog::logger>("default", spdlog::sinks_init_list{file_sink});

    // Set default logger
    spdlog::set_default_logger(default_logger_);

    // Set global log level
    spdlog::set_level(spdlog::level::trace);

    // Set flush policy
    spdlog::flush_on(spdlog::level::info);

    initialized_ = true;

    std::cout << "Spdlog initialized successfully!" << std::endl;
    std::cout << "Log file: " << log_file << std::endl;

  } catch (const spdlog::spdlog_ex& ex) {
    std::cerr << "Log initialization failed: " << ex.what() << std::endl;
  }
}

std::shared_ptr<spdlog::logger> SpdlogManager::getLogger(const std::string& name) {
  if (!initialized_) {
    initialize();
  }

  auto logger = spdlog::get(name);
  if (!logger) {
    logger = spdlog::default_logger()->clone(name);
    spdlog::register_logger(logger);
  }

  return logger;
}

void SpdlogManager::setLevel(spdlog::level::level_enum level) {
  if (initialized_) {
    std::cout << "Setting log level to " << level << std::endl;
    spdlog::set_level(level);
    spdlog::get("default")->set_level(level);
    default_logger_->set_level(level);
  }
}

void SpdlogManager::flush() {
  if (initialized_) {
    spdlog::get("default")->flush();
  }
}

void SpdlogManager::clearOldLogFiles(const std::string& log_file) {
  try {
    std::cout << "Clearing old log files: " << log_file << std::endl;
    std::filesystem::path log_path(log_file);
    std::filesystem::path log_dir = log_path.parent_path();
    std::string log_name = log_path.stem().string();
    std::string log_ext = log_path.extension().string();
    
    // Create directory if it doesn't exist
    if (!log_dir.empty() && !std::filesystem::exists(log_dir)) {
      std::filesystem::create_directories(log_dir);
    }
    
    // Delete main log file
    if (std::filesystem::exists(log_file)) {
      std::filesystem::remove(log_file);
      std::cout << "Deleted old log file: " << log_file << std::endl;
    }
    
    // Delete all rotated log files (spdlog rotating file sink naming convention)
    // Rotated file format: xxx.1.log, xxx.2.log, ...
    for (int i = 1; i <= 10; ++i) {  // Check up to 10 rotated files
      std::string rotated_file = log_name + "." + std::to_string(i) + log_ext;
      if (std::filesystem::exists(rotated_file)) {
        std::filesystem::remove(rotated_file);
        std::cout << "Deleted old rotated log file: " << rotated_file << std::endl;
      }
    }
    
  } catch (const std::filesystem::filesystem_error& ex) {
    std::cerr << "Error clearing old log files: " << ex.what() << std::endl;
  }
}

}  // namespace odin_rtk_fusion
