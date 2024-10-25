#include "frc846/ntinf/fstore.h"

#include <fmt/core.h>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

namespace frc846::ntinf {

std::vector<std::string> FunkyStore::changes = std::vector<std::string>();
frc846::base::Loggable FunkyStore::fstore_loggable{"FunkyStore"};

bool FunkyStore::hasChanged = false;

std::map<std::string, std::variant<int, double, std::string, bool>>
    FunkyStore::prefs = {{"Default.int", 0.0}};

std::unordered_set<std::string> FunkyStore::keys_accessed{};

std::string FunkyStore::variantToString(
    std::variant<int, double, std::string, bool>& variant) {
  return std::visit(
      [](const auto& arg) -> std::string {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, std::string>) {
          return arg;
        } else if constexpr (std::is_same_v<T, bool>) {
          return arg ? "true" : "false";
        } else {
          return std::to_string(arg);
        }
      },
      variant);
}

std::variant<int, double, std::string, bool> FunkyStore::stringToVariant(
    const std::string& str, const std::string& data) {
  size_t dotPos = str.rfind('.');

  if (dotPos != std::string::npos && dotPos < str.length() - 1) {
    std::string type = str.substr(dotPos + 1);

    std::string valueStr = data;

    if (type.compare("int") == 0) {
      int intValue;
      std::istringstream(valueStr) >> intValue;
      return intValue;
    } else if (type.compare("double") == 0) {
      double doubleValue;
      std::istringstream(valueStr) >> doubleValue;
      return doubleValue;
    } else if (type.compare("bool") == 0) {
      bool boolValue;
      std::istringstream(valueStr) >> std::boolalpha >> boolValue;
      return boolValue;
    } else if (type.compare("string") == 0) {
      return valueStr;
    }
  }

  return "";
}

std::string trim(const std::string& str) {
  auto start = std::find_if(str.begin(), str.end(),
                            [](int ch) { return !std::isspace(ch); });

  auto end = std::find_if(str.rbegin(), str.rend(),
                          [](int ch) { return !std::isspace(ch); });

  return (start < end.base()) ? std::string(start, end.base()) : std::string();
}

bool FunkyStore::ContainsKey(std::string key) {
  keys_accessed.insert(key);
  if (!hasReadPrefs) {
    return false;
  }
  try {
    if (prefs.empty()) return false;
    return (prefs.find(key) != prefs.end());
  } catch (const std::exception& exc) {
    (void)exc;
  }
  return false;
}

void FunkyStore::SetDouble(std::string key, double val) {
  FunkyStore::SetString(key, std::to_string(val), true);
  keys_accessed.insert(key + ".double");
  hasChanged = true;
}

double FunkyStore::GetDouble(std::string key) {
  if (FunkyStore::ContainsKey(key + ".double")) {
    if (auto val = std::get_if<std::string>(&prefs[key + ".double"])) {
      try {
        double doubleValue;
        std::istringstream(*val) >> doubleValue;
        return doubleValue;
      } catch (const std::exception& exc) {
        (void)exc;
      }
    }
  }

  return 0.0;
}

double FunkyStore::GetDouble(std::string key, double fallback) {
  if (FunkyStore::ContainsKey(key + ".double")) {
    if (auto val = std::get_if<double>(&prefs[key + ".double"])) return *val;
  }

  return fallback;
}

void FunkyStore::SetInt(std::string key, int val) {
  prefs[key + ".int"] = val;
  keys_accessed.insert(key + ".int");
  hasChanged = true;
}

int FunkyStore::GetInt(std::string key) {
  if (FunkyStore::ContainsKey(key + ".int")) {
    if (auto val = std::get_if<int>(&prefs[key + ".int"])) return *val;
  }

  return 0;
}

int FunkyStore::GetInt(std::string key, int fallback) {
  if (FunkyStore::ContainsKey(key + ".int")) {
    if (auto val = std::get_if<int>(&prefs[key + ".int"])) return *val;
  }

  return fallback;
}

void FunkyStore::SetBoolean(std::string key, bool val) {
  prefs[key + ".bool"] = val;
  keys_accessed.insert(key + ".bool");
  hasChanged = true;
}

bool FunkyStore::GetBoolean(std::string key) {
  if (FunkyStore::ContainsKey(key + ".bool")) {
    if (auto val = std::get_if<bool>(&prefs[key + ".bool"])) return *val;
  }

  return false;
}

bool FunkyStore::GetBoolean(std::string key, bool fallback) {
  if (FunkyStore::ContainsKey(key + ".bool")) {
    if (auto val = std::get_if<bool>(&prefs[key + ".bool"])) return *val;
  }

  return fallback;
}

void FunkyStore::SetString(std::string key, std::string val, bool isDouble) {
  if (!isDouble) {
    prefs[key + ".string"] = val;
    keys_accessed.insert(key + ".string");
  } else {
    prefs[key + ".double"] = val;
    keys_accessed.insert(key + ".double");
  }

  hasChanged = true;
}

std::string FunkyStore::GetString(std::string key) {
  if (FunkyStore::ContainsKey(key + ".string")) {
    if (std::string* val = std::get_if<std::string>(&prefs[key + ".string"]))
      return *val;
  }

  return "";
}

std::string FunkyStore::GetString(std::string key, std::string fallback) {
  if (FunkyStore::ContainsKey(key + ".string")) {
    if (auto val = std::get_if<std::string>(&prefs[key + ".string"]))
      return *val;
  }

  return fallback;
}

bool FunkyStore::hasReadPrefs = false;

void FunkyStore::HardReadPrefs() {
  auto filename = "/home/lvuser/preferences.nform";

  std::ifstream t_file(filename);

  std::string line;
  while (std::getline(t_file, line)) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;

    std::string token;
    while (std::getline(iss, token, '|')) {
      tokens.push_back(trim(token));
    }

    if (tokens.size() == 2) {
      prefs[tokens[0]] = stringToVariant(tokens[0], tokens[1]);
    }
  }

  t_file.close();
}

void FunkyStore::FP_HardReadPrefs() {
  if (!hasReadPrefs) {
    auto filename = "/home/lvuser/preferences.nform";

    std::map<std::string, std::variant<int, double, std::string, bool>> temp{};

    std::ifstream t_file(filename);

    std::string line;
    while (std::getline(t_file, line)) {
      std::istringstream iss(line);
      std::vector<std::string> tokens;

      std::string token;
      while (std::getline(iss, token, '|')) {
        tokens.push_back(trim(token));
      }

      if (tokens.size() == 2) {
        if (temp.find(tokens.at(0)) == temp.end()) {
          // not found
          temp.insert(
              {tokens.at(0), stringToVariant(tokens.at(0), tokens.at(1))});
        } else {
          // found
          temp[tokens.at(0)] = stringToVariant(tokens.at(0), tokens.at(1));
        }
      }

      prefs = temp;
    }

    t_file.close();
  }
  hasReadPrefs = true;
}

void FunkyStore::WriteToDisk() {
  auto filename = "/home/lvuser/preferences.nform.tmp";
  std::map<std::string, std::variant<int, double, std::string, bool>>::iterator
      it;
  std::ofstream t_file(
      filename, std::fstream::in | std::fstream::out | std::fstream::trunc);
  for (it = prefs.begin(); it != prefs.end(); ++it) {
    t_file << trim(it->first) << " | " << trim(variantToString(it->second))
           << std::endl;
  }
  t_file.close();
  std::rename("/home/lvuser/preferences.nform.tmp",
              "/home/lvuser/preferences.nform");
  AppendToChangeLog();
}

std::vector<std::string> FunkyStore::GetPruneList() {
  std::vector<std::string> result{};
  for (std::map<std::string,
                std::variant<int, double, std::string, bool>>::iterator it =
           FunkyStore::prefs.begin();
       it != FunkyStore::prefs.end(); ++it) {
    bool foundKey = false;
    for (const std::string& x : FunkyStore::keys_accessed) {
      if (it->first == x) {
        foundKey = true;
        break;
      }
    }
    if (!foundKey) result.push_back(it->first);
  }
  return result;
}

void FunkyStore::Prune() {
  for (const std::string& x : GetPruneList()) {
    prefs.erase(x);
  }
  hasChanged = true;
}

void FunkyStore::AppendToChangeLog() {
  // auto filename = "/home/lvuser/preferences.changelog";
  // std::map<std::string, std::variant<int, double, std::string,
  // bool>>::iterator
  //     it;
  // std::ofstream t_file(
  //     filename, std::fstream::in | std::fstream::out | std::fstream::app);
  // t_file << "#SAVE#" << std::endl;
  // for (auto line : changes) {
  //   t_file << line << std::endl;
  // }
  changes.clear();
  // t_file.close();
}

}  // namespace frc846::ntinf