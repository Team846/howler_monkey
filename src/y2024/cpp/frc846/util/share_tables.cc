#include "frc846/util/share_tables.h"

std::map<std::string, int> frc846::util::ShareTables::imap{};
std::map<std::string, double> frc846::util::ShareTables::fmap{};
std::map<std::string, bool> frc846::util::ShareTables::bmap{};
std::map<std::string, std::string> frc846::util::ShareTables::smap{};

double frc846::util::ShareTables::GetDouble(std::string key) {
    if (fmap.find(key) != fmap.end()) {
        return fmap[key];
    } else {
        return 0.0;
    }
}

void frc846::util::ShareTables::SetDouble(std::string key, double val) {
    fmap[key] = val;
    nt_table_->PutNumber(key, val);
}

int frc846::util::ShareTables::GetInt(std::string key) {
    if (imap.find(key) != imap.end()) {
        return imap[key];
    } else {
        return 0;
    }
}

void frc846::util::ShareTables::SetInt(std::string key, int val) {
    imap[key] = val;
    nt_table_->PutNumber(key, val);
}

bool frc846::util::ShareTables::GetBoolean(std::string key) {
    if (bmap.find(key) != bmap.end()) {
        return bmap[key];
    } else {
        return false;
    }
}

void frc846::util::ShareTables::SetBoolean(std::string key, bool val) {
    bmap[key] = val;
    nt_table_->PutBoolean(key, val);
}

std::string frc846::util::ShareTables::GetString(std::string key) {
    if (smap.find(key) != smap.end()) {
        return smap[key];
    } else {
        return "";
    }
}

void frc846::util::ShareTables::SetString(std::string key, std::string val) {
    smap[key] = val;
    nt_table_->PutString(key, val);
}

std::shared_ptr<nt::NetworkTable> frc846::util::ShareTables::nt_table_
    = nt::NetworkTableInstance::GetDefault().GetTable("ShareTables");