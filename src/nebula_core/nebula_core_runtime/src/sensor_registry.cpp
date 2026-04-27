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
  for (const auto & path : search_paths) {
    if (!fs::exists(path) || !fs::is_directory(path)) {
      continue;
    }

    for (fs::directory_iterator it(path); it != fs::directory_iterator(); ++it) {
      if (fs::is_regular_file(it->status()) && it->path().extension() == ".json") {
        try {
          std::ifstream ifs(it->path().string());
          nlohmann::json j = nlohmann::json::parse(ifs);

          SensorPluginMetadata metadata;
          metadata.vendor = j.at("vendor").get<std::string>();
          metadata.package_name = j.at("package").get<std::string>();
          metadata.library_path = j.at("library").get<std::string>();
          
          for (const auto & m : j.at("models")) {
            metadata.supported_models.push_back(sensor_model_from_string(m.get<std::string>()));
          }

          registered_plugins_[metadata.package_name] = metadata;
        } catch (const std::exception & e) {
          std::cerr << "Failed to parse plugin descriptor " << it->path() << ": " << e.what() << std::endl;
        }
      }
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

  void * handle = load_library(metadata.library_path);
  if (!handle) {
    return nullptr;
  }

  // Expecting a factory function: extern "C" nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
  using CreateFunc = SensorPlugin * (*)();
  auto create_func = reinterpret_cast<CreateFunc>(dlsym(handle, "create_nebula_sensor_plugin"));
  if (!create_func) {
    std::cerr << "Failed to find factory symbol in " << metadata.library_path << ": " << dlerror() << std::endl;
    return nullptr;
  }

  std::shared_ptr<SensorPlugin> plugin(create_func());
  if (plugin) {
    instantiated_plugins_[metadata.package_name] = plugin;
  }
  return plugin;
}

const std::map<std::string, SensorPluginMetadata> & SensorRegistry::get_registered_plugins() const
{
  return registered_plugins_;
}

void * SensorRegistry::load_library(const std::string & library_path)
{
  if (loaded_libraries_.count(library_path)) {
    return loaded_libraries_[library_path];
  }

  void * handle = dlopen(library_path.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!handle) {
    std::cerr << "Failed to load library " << library_path << ": " << dlerror() << std::endl;
    return nullptr;
  }

  loaded_libraries_[library_path] = handle;
  return handle;
}

}  // namespace nebula::drivers
