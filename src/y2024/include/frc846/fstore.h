#ifndef frcLib846_NCPROT_H_
#define frcLib846_NCPROT_H_

#include <csignal>

#include <type_traits>
#include <map>
#include <iostream>
#include <variant>
#include <vector>
#include <functional>
#include <chrono>

#include "frc/DataLogManager.h"
#include <fmt/core.h>
#include <fcntl.h>


namespace frc846 {

class FunkyStore {
    public:
        static std::map<std::string, std::variant<int, double, std::string, bool>> prefs;

        static std::vector<std::string> changes;

        static bool ContainsKey(std::string key);
        static void SetInt(std::string key, int val);
        static void SetBoolean(std::string key, bool val);
        static void SetString(std::string key, std::string val, bool isDouble = false);
        static void SetDouble(std::string key, double val);
        static double GetDouble(std::string key);
        static int GetInt(std::string key);
        static bool GetBoolean(std::string key);
        static std::string GetString(std::string key);
        static double GetDouble(std::string key, double fallback);
        static int GetInt(std::string key, int fallback);
        static bool GetBoolean(std::string key, bool fallback);
        static std::string GetString(std::string key, std::string fallback);

        static std::string variantToString(std::variant<int, double, std::string, bool>& variant);
        static std::variant<int, double, std::string, bool> stringToVariant(const std::string& str, const std::string& data);


        FunkyStore() { HardReadPrefs(); };

        void Refresh() {
            if (counter % 30 == 0) {
                if (hasChanged) {
                    frc::DataLogManager::Log("SAVE triggered");
                    WriteToDisk();

                    hasChanged = false;
                }

                counter = 0;
            }
            counter ++;
        }


        void WriteToDisk();
        void AppendToChangeLog();

        static void HardReadPrefs();
        static void FP_HardReadPrefs();

        private:

        int counter = 0;

        static bool hasReadPrefs;

        static bool hasChanged;

};

class FPointer {
    public:
    std::string key_;

    FPointer(std::string key) : 
        key_(key) {}
    
    void set(int val) {
        FunkyStore::SetInt(key_, val);
    }

    void set(double val) {
        FunkyStore::SetDouble(key_, val);
    }

    void set(bool val) {
        FunkyStore::SetBoolean(key_, val);
    }

    void set(std::string val) {
        FunkyStore::SetString(key_, val);
    }

    int int_value(int val) {
        return FunkyStore::GetInt(key_, val);
    }

    double double_value(double val) {
        return FunkyStore::GetDouble(key_, val);
    }

    bool bool_value(bool val) {
        return FunkyStore::GetBoolean(key_, val);
    }

    std::string string_value(std::string val) {
        return FunkyStore::GetString(key_, val);
    }

};

}

#endif