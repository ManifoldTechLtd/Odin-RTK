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
#pragma once

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace odin_rtk_fusion {

class SpdlogManager {
 public:
  static SpdlogManager& getInstance();

  // Initialize logging system
  void initialize(const std::string& log_file = "spdlog_test.log", bool clear_old_logs = false);

  // Get default logger
  std::shared_ptr<spdlog::logger> getLogger(const std::string& name = "default");

  // Set log level
  void setLevel(spdlog::level::level_enum level);

  // Flush logs
  void flush();

 private:
  // Clear old log files
  void clearOldLogFiles(const std::string& log_file);
  SpdlogManager()                                = default;
  ~SpdlogManager()                               = default;
  SpdlogManager(const SpdlogManager&)            = delete;
  SpdlogManager& operator=(const SpdlogManager&) = delete;

  bool                            initialized_ = false;
  std::shared_ptr<spdlog::logger> default_logger_;
};

}  // namespace odin_rtk_fusion

// Convenience macros
#define LOG_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define LOG_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

// Module-specific log macros
#define MODULE_LOG_TRACE(module, ...) SPDLOG_TRACE("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_DEBUG(module, ...) SPDLOG_DEBUG("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_INFO(module, ...) SPDLOG_INFO("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_WARN(module, ...) SPDLOG_WARN("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_ERROR(module, ...) SPDLOG_ERROR("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_CRITICAL(module, ...) SPDLOG_CRITICAL("[{}] " __VA_ARGS__, module)

// Initialization macros
#define INIT_SPDLOG(log_file) odin_rtk_fusion::SpdlogManager::getInstance().initialize(log_file)
#define INIT_SPDLOG_CLEAR(log_file) odin_rtk_fusion::SpdlogManager::getInstance().initialize(log_file, true)
#define SET_LOG_LEVEL(level) odin_rtk_fusion::SpdlogManager::getInstance().setLevel(level)
