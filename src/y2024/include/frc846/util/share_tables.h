#ifndef frcRev846_SHARE_TABLES_H_
#define frcRev846_SHARE_TABLES_H_

#include <exception>
#include <map>
#include <string>
#include <networktables/NetworkTableInstance.h>

namespace frc846::util {

class ShareTables {
 public:
    static std::map<std::string, double> fmap;

    static double GetVal(std::string key);

    static void SetVal(std::string key, double val);

 private:
 static std::shared_ptr<nt::NetworkTable> nt_table_;
};

class SharePointer {
    public:
    SharePointer(std::string key) {
        key_ = key;
    }

    double value() {
        return ShareTables::GetVal(key_);
    }

    private:
    std::string key_;
};

}  // namespace frc846::util

#endif  // frcRev846_SHARE_TABLES_H_