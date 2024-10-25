#include "frc846/robot/RobotState.h"

namespace frc846::robot {

RSTable::RSTable(std::string table_name)
    : frc846::base::Loggable{table_name}, table_{} {}

std::vector<std::string> RSTable::ListKeys() {
  std::vector<std::string> keys;
  for (const auto& [key, value] : table_) {
    keys.push_back(key);
  }
  return keys;
}

std::unordered_map<std::string, RSTable*> RobotState::tables_{};

RSTable* RobotState::getTable(std::string table_name) {
  if (tables_.find(table_name) == tables_.end()) {
    tables_.insert({table_name, new RSTable{table_name}});
  }
  return tables_[table_name];
}

};  // namespace frc846::robot