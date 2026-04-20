/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
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

class SpdlogManager {
 public:
  static SpdlogManager& getInstance();

  // 初始化日志系统
  void initialize(const std::string& log_file = "spdlog_test.log", bool clear_old_logs = false);

  // 获取默认日志器
  std::shared_ptr<spdlog::logger> getLogger(const std::string& name = "default");

  // 设置日志级别
  void setLevel(spdlog::level::level_enum level);

  // 刷新日志
  void flush();

 private:
  // 清除旧的日志文件
  void clearOldLogFiles(const std::string& log_file);
  SpdlogManager()                                = default;
  ~SpdlogManager()                               = default;
  SpdlogManager(const SpdlogManager&)            = delete;
  SpdlogManager& operator=(const SpdlogManager&) = delete;

  bool                            initialized_ = false;
  std::shared_ptr<spdlog::logger> default_logger_;
};

// 便捷的宏定义
#define LOG_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define LOG_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

// 模块特定的日志宏
#define MODULE_LOG_TRACE(module, ...) SPDLOG_TRACE("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_DEBUG(module, ...) SPDLOG_DEBUG("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_INFO(module, ...) SPDLOG_INFO("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_WARN(module, ...) SPDLOG_WARN("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_ERROR(module, ...) SPDLOG_ERROR("[{}] " __VA_ARGS__, module)
#define MODULE_LOG_CRITICAL(module, ...) SPDLOG_CRITICAL("[{}] " __VA_ARGS__, module)

// 初始化宏
#define INIT_SPDLOG(log_file) SpdlogManager::getInstance().initialize(log_file)
#define INIT_SPDLOG_CLEAR(log_file) SpdlogManager::getInstance().initialize(log_file, true)
#define SET_LOG_LEVEL(level) SpdlogManager::getInstance().setLevel(level)
