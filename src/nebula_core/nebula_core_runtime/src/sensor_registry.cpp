// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <nebula_core_runtime/sensor_registry.hpp>

#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <dlfcn.h>
#include <fstream>
#include <iostream>

namespace nebula::drivers
{
namespace fs = boost::filesystem;

SensorRegistry::~SensorRegistry()
{
  instantiated_plugins_.clear();
  for (auto & pair : loaded_libraries_) {
    if (pair.second) {
      dlclose(pair.second);
    }
  }
}

void SensorRegistry::load_registry(const std::vector<std::string> & search_paths)
{
  std::vector<std::string> descriptor_files;

  // 1. Explicit search paths
  for (const auto & path : search_paths) {
      if (fs::exists(path) && fs::is_directory(path)) {
          for (fs::directory_iterator it(path); it != fs::directory_iterator(); ++it) {
              if (fs::is_regular_file(it->status()) && it->path().extension() == ".json") {
                  descriptor_files.push_back(it->path().string());
              }
          }
      }
  }

  // 2. Add paths from NEBULA_PLUGINS_PATH
  char* nebula_env = std::getenv("NEBULA_PLUGINS_PATH");
  if (nebula_env) {
      std::vector<std::string> env_paths;
      boost::split(env_paths, nebula_env, boost::is_any_of(":"));
      for (const auto & path : env_paths) {
          if (fs::exists(path) && fs::is_directory(path)) {
              for (fs::directory_iterator it(path); it != fs::directory_iterator(); ++it) {
                  if (fs::is_regular_file(it->status()) && it->path().extension() == ".json") {
                      descriptor_files.push_back(it->path().string());
                  }
              }
          }
      }
  }

  // 3. Auto-discover from AMENT_PREFIX_PATH and COLCON_PREFIX_PATH
  std::vector<std::string> prefix_envs = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"};
  for (const auto & env_name : prefix_envs) {
      char* env_val = std::getenv(env_name.c_str());
      if (env_val) {
          std::vector<std::string> prefixes;
          boost::split(prefixes, env_val, boost::is_any_of(":"));
          for (const auto & prefix : prefixes) {
              if (prefix.empty()) continue;
              
              std::vector<fs::path> share_paths;
              // Add <prefix>/share (merged install or ament prefix)
              share_paths.push_back(fs::path(prefix) / "share");
              
              // Add <prefix>/*/share (isolated colcon install)
              if (fs::exists(prefix) && fs::is_directory(prefix)) {
                  for (fs::directory_iterator it(prefix); it != fs::directory_iterator(); ++it) {
                      if (fs::is_directory(it->status())) {
                          share_paths.push_back(it->path() / "share");
                      }
                  }
              }

              for (const auto & share_path : share_paths) {
                  if (fs::exists(share_path) && fs::is_directory(share_path)) {
                      for (fs::directory_iterator it(share_path); it != fs::directory_iterator(); ++it) {
                          if (fs::is_directory(it->status())) {
                              for (fs::directory_iterator pkg_it(it->path()); pkg_it != fs::directory_iterator(); ++pkg_it) {
                                  if (fs::is_regular_file(pkg_it->status()) && 
                                      (pkg_it->path().extension() == ".json") &&
                                      (pkg_it->path().filename().string().find("plugin") != std::string::npos)) {
                                      descriptor_files.push_back(pkg_it->path().string());
                                  }
                              }
                          }
                      }
                  }
              }
          }
      }
  }

  for (const auto & file_path : descriptor_files) {
    try {
      std::ifstream ifs(file_path);
      nlohmann::json j = nlohmann::json::parse(ifs);
      
      if (!j.contains("vendor") || !j.contains("package") || !j.contains("library")) {
          continue;
      }

      SensorPluginMetadata metadata;
      metadata.vendor = j.at("vendor").get<std::string>();
      metadata.package_name = j.at("package").get<std::string>();
      metadata.library_path = j.at("library").get<std::string>();
      metadata.factory_symbol = j.value("factory", "create_nebula_sensor_plugin");
      
      for (const auto & m : j.at("models")) {
        metadata.supported_models.push_back(sensor_model_from_string(m.get<std::string>()));
      }

          if (fs::path(metadata.library_path).is_relative()) {
              bool found = false;
              // 1. Try relative to the descriptor file's directory
              fs::path rel_to_desc = fs::path(file_path).parent_path() / metadata.library_path;
              if (fs::exists(rel_to_desc)) {
                  metadata.library_path = rel_to_desc.string();
                  found = true;
              }
              // 2. Try in the install prefix of the *target* package (metadata.package_name)
              if (!found) {
                  // The descriptor is usually in <prefix>/<common_pkg>/share/<common_pkg>/...
                  // Or <prefix>/share/<common_pkg>/...
                  // We need to find <prefix>/<metadata.package_name>/lib/...
                  // We can search the known prefix environments
                  std::vector<std::string> prefix_envs = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"};
                  for (const auto & env_name : prefix_envs) {
                      char* env_val = std::getenv(env_name.c_str());
                      if (env_val) {
                          std::vector<std::string> prefixes;
                          boost::split(prefixes, env_val, boost::is_any_of(":"));
                          for (const auto & prefix : prefixes) {
                              if (prefix.empty()) continue;
                              
                              // Try common lib directory
                              fs::path p1 = fs::path(prefix) / "lib" / metadata.library_path;
                              if (fs::exists(p1)) {
                                  metadata.library_path = p1.string();
                                  found = true;
                                  break;
                              }
                              
                              // Try isolated package lib directory
                              fs::path p2 = fs::path(prefix) / metadata.package_name / "lib" / metadata.library_path;
                              if (!metadata.package_name.empty() && fs::exists(p2)) {
                                  metadata.library_path = p2.string();
                                  found = true;
                                  break;
                              }
                          }
                          if (found) break;
                      }
                  }
              }
          }

      registered_plugins_[metadata.package_name] = metadata;
    } catch (const std::exception & e) {
       std::cerr << "Failed to parse plugin descriptor " << file_path << ": " << e.what() << std::endl;
    }
  }
}

std::optional<SensorPluginMetadata> SensorRegistry::find_plugin_for_model(SensorModel model) const
{
  for (const auto & pair : registered_plugins_) {
    for (const auto & supported_model : pair.second.supported_models) {
      if (supported_model == model) {
        return pair.second;
      }
    }
  }
  return std::nullopt;
}

std::shared_ptr<SensorPlugin> SensorRegistry::load_plugin(const SensorPluginMetadata & metadata)
{
  if (instantiated_plugins_.count(metadata.package_name)) {
    return instantiated_plugins_[metadata.package_name];
  }

  void * handle = load_library(metadata.library_path, metadata.package_name);
  if (!handle) {
    std::cerr << "Failed to load library " << metadata.library_path << " for plugin " << metadata.package_name << ": " << dlerror() << std::endl;
    return nullptr;
  }

  using CreateFunc = SensorPlugin * (*)();
  auto create_func = reinterpret_cast<CreateFunc>(dlsym(handle, metadata.factory_symbol.c_str()));
  if (!create_func) {
    std::cerr << "Failed to find factory symbol '" << metadata.factory_symbol << "' in " << metadata.library_path << ": " << dlerror() << std::endl;
    return nullptr;
  }

  std::shared_ptr<SensorPlugin> plugin(create_func());
  if (plugin) {
    instantiated_plugins_[metadata.package_name] = plugin;
  } else {
      std::cerr << "Factory function '" << metadata.factory_symbol << "' returned nullptr for plugin " << metadata.package_name << std::endl;
  }
  return plugin;
}

const std::map<std::string, SensorPluginMetadata> & SensorRegistry::get_registered_plugins() const
{
  return registered_plugins_;
}

void * SensorRegistry::load_library(const std::string & library_path, const std::string & package_name)
{
  if (loaded_libraries_.count(library_path)) {
    return loaded_libraries_[library_path];
  }

  void * handle = dlopen(library_path.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  
  if (!handle && fs::path(library_path).is_relative()) {
      std::vector<std::string> prefix_envs = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"};
      for (const auto & env_name : prefix_envs) {
          char* env_val = std::getenv(env_name.c_str());
          if (env_val) {
              std::vector<std::string> prefixes;
              boost::split(prefixes, env_val, boost::is_any_of(":"));
              for (const auto & prefix : prefixes) {
                  if (prefix.empty()) continue;
                  // Try common prefix lib
                  fs::path p1 = fs::path(prefix) / "lib" / library_path;
                  if (fs::exists(p1)) {
                      handle = dlopen(p1.c_str(), RTLD_LAZY | RTLD_GLOBAL);
                      if (handle) break;
                  }
                  // Try package-specific lib (isolated layouts)
                  fs::path p2 = fs::path(prefix) / package_name / "lib" / library_path;
                  if (!package_name.empty() && fs::exists(p2)) {
                      handle = dlopen(p2.c_str(), RTLD_LAZY | RTLD_GLOBAL);
                      if (handle) break;
                  }
              }
              if (handle) break;
          }
      }
  }

  if (handle) {
      loaded_libraries_[library_path] = handle;
  }
  return handle;
}

}  // namespace nebula::drivers
