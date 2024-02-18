#include "frc846/util/share_tables.h"

std::map<std::string, double> frc846::util::ShareTables::fmap{};

double frc846::util::ShareTables::GetVal(std::string key) {
    if (fmap.find(key) != fmap.end()) {
        return fmap[key];
    } else {
        return 0.0;
    }
}

void frc846::util::ShareTables::SetVal(std::string key, double val) {
        fmap[key] = val;
        nt_table_->PutNumber(key, val);
}

std::shared_ptr<nt::NetworkTable> frc846::util::ShareTables::nt_table_
    = nt::NetworkTableInstance::GetDefault().GetTable("ShareTables");