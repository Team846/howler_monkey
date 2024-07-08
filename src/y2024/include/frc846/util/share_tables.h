#pragma once

#include <networktables/NetworkTableInstance.h>

#include <exception>
#include <map>
#include <string>

namespace frc846::util {

class ShareTables {
 public:
  static std::map<std::string, int> imap;
  static std::map<std::string, double> fmap;
  static std::map<std::string, bool> bmap;
  static std::map<std::string, std::string> smap;

  static double GetDouble(std::string key);
  static int GetInt(std::string key);
  static bool GetBoolean(std::string key);
  static std::string GetString(std::string key);

  static void SetDouble(std::string key, double val);
  static void SetInt(std::string key, int val);
  static void SetBoolean(std::string key, bool val);
  static void SetString(std::string key, std::string val);

 private:
  static std::shared_ptr<nt::NetworkTable> nt_table_;
};

class SharePointer {
 public:
  SharePointer(std::string key, std::string type = "double") {
    key_ = key;
    type_ = type;
  }

  std::variant<double, int, std::string, bool> value() {
    if (type_.compare("double") == 0) {
      return ShareTables::GetDouble(key_);
    } else if (type_.compare("int") == 0) {
      return ShareTables::GetInt(key_);
    } else if (type_.compare("string") == 0) {
      return ShareTables::GetString(key_);
    } else {
      return ShareTables::GetBoolean(key_);
    }
  }

 private:
  std::string key_;
  std::string type_;
};

}  // namespace frc846::util
