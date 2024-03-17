#include "frc846/fstore.h"
#include <exception>
#include <variant>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include "frc/DataLogManager.h"
#include <fmt/core.h>

namespace frc846 {

    std::vector<std::string> FunkyStore::changes = std::vector<std::string>();

    bool FunkyStore::hasChanged = false;
    
    std::string FunkyStore::variantToString(std::variant<int, double, std::string, bool>& variant) {
        return std::visit([](const auto& arg) -> std::string {
            using T = std::decay_t<decltype(arg)>;

            if constexpr (std::is_same_v<T, std::string>) {
                return arg;
            } else if constexpr (std::is_same_v<T, bool>) {
                return arg ? "true" : "false";
            } else {
                return std::to_string(arg);
            }
        }, variant);
    }

    std::variant<int, double, std::string, bool> FunkyStore::stringToVariant(const std::string& str, const std::string& data) {
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
        auto start = std::find_if(str.begin(), str.end(), [](int ch) {
            return !std::isspace(ch);
        });

        auto end = std::find_if(str.rbegin(), str.rend(), [](int ch) {
            return !std::isspace(ch);
        });

        return (start < end.base()) ? std::string(start, end.base()) : std::string();
    }
    

    std::map<std::string, std::variant<int, double, std::string, bool>> FunkyStore::prefs
        = {{"Default.int", 0.0}};

    bool FunkyStore::ContainsKey(std::string key) {
        return (prefs.find(key) != prefs.end());
    }

    void FunkyStore::SetDouble(std::string key, double val) {
        FunkyStore::SetString(key, std::to_string(val), true);
        hasChanged = true;
    }

    double FunkyStore::GetDouble(std::string key) {
        if (FunkyStore::ContainsKey(key+".double")) {
            if (auto val = std::get_if<std::string>(&prefs[key+".double"])) {
                try {
                    double doubleValue;
                    std::istringstream(*val) >> doubleValue;
                    return doubleValue;
                } catch (std::exception const&) {}
            }
        }

        return 0.0;
    }

    double FunkyStore::GetDouble(std::string key, double fallback) {
        if (FunkyStore::ContainsKey(key+".double")) {
            if (auto val = std::get_if<double>(&prefs[key+".double"])) return *val;
        }

        return fallback;
    }

    void FunkyStore::SetInt(std::string key, int val) {
        prefs[key + ".int"] = val;
        hasChanged = true;
    }

    int FunkyStore::GetInt(std::string key) {
        if (FunkyStore::ContainsKey(key+".int")) {
            if (auto val = std::get_if<int>(&prefs[key+".int"])) return *val;
        }
        
        return 0;
    }

    int FunkyStore::GetInt(std::string key, int fallback) {
        if (FunkyStore::ContainsKey(key+".int")) {
            if (auto val = std::get_if<int>(&prefs[key+".int"])) return *val;
        }
        
        return fallback;
    }

    void FunkyStore::SetBoolean(std::string key, bool val) {
        while (prefs.empty()) {}
        prefs[key + ".bool"] = val;
        hasChanged = true;
    }

    bool FunkyStore::GetBoolean(std::string key) {
        if (FunkyStore::ContainsKey(key+".bool")) {
            if (auto val = std::get_if<bool>(&prefs[key+".bool"])) return *val;
        }
        
        return false;
    }

    bool FunkyStore::GetBoolean(std::string key, bool fallback) {
        if (FunkyStore::ContainsKey(key+".bool")) {
            if (auto val = std::get_if<bool>(&prefs[key+".bool"])) return *val;
        }
        
        return fallback;
    }

    void FunkyStore::SetString(std::string key, std::string val, bool isDouble) {
        if (!isDouble) {
            prefs[key + ".string"] = val;
        } else {
            prefs[key + ".double"] = val;
        }

        hasChanged = true;
    }

    std::string FunkyStore::GetString(std::string key) {
        if (FunkyStore::ContainsKey(key+".string")) {
            if (std::string* val = std::get_if<std::string>(&prefs[key+".string"])) return *val;
        }
        
        return "";
    }

    std::string FunkyStore::GetString(std::string key, std::string fallback) {
        if (FunkyStore::ContainsKey(key+".string")) {
            if (auto val = std::get_if<std::string>(&prefs[key+".string"])) return *val;
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
        hasReadPrefs = true;
    }

    void FunkyStore::WriteToDisk() {
        auto filename = "/home/lvuser/preferences.nform.tmp";
        std::map<std::string, std::variant<int, double, std::string, bool>>::iterator it;
        std::ofstream t_file(filename,  std::fstream::in | std::fstream::out | std::fstream::trunc);
        for (it = prefs.begin(); it != prefs.end(); it++) {
                t_file << trim(it->first) << " | "
                        << trim(variantToString(it->second))
                        << std::endl;
        }
        t_file.close();
        std::rename("/home/lvuser/preferences.nform.tmp", "/home/lvuser/preferences.nform");
        AppendToChangeLog();
    }

    void FunkyStore::AppendToChangeLog() {
        auto filename = "/home/lvuser/preferences.changelog";
        std::map<std::string, std::variant<int, double, std::string, bool>>::iterator it;
        std::ofstream t_file(filename,  std::fstream::in | std::fstream::out | std::fstream::app);
        t_file << "#SAVE#" << std::endl;
        for (auto line : changes) {
                t_file << line << std::endl;
        }
        changes.clear();
        t_file.close();
    }

}