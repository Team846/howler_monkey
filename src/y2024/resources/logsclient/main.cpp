#include "log.h"
#include "k.h"
#include "compression.h"

#include <algorithm>

auto HandleLogMessage(int type, std::string fmt) {
    LogMessage msg;

    msg.type = type;
    msg.sender = "Sending thing";

    std::string temp_content = fmt;
    temp_content.erase(
        std::remove(temp_content.begin(), temp_content.end(), ';'),
        temp_content.end());

    msg.content = temp_content;

    msg.char_count = msg.sender.size() + msg.content.size();
    msg.timestamp = 100.0;

    msg.period = 1;
    msg.period_timestamp =
        200.0;

    return msg;
}

int main() {
    frc846::base::LoggingServer server;
    server.Start(5808);

    int counter = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::string ts = HandleLogMessage(0, "Hello world " + std::to_string(counter) + ". This is a very very very very very very very very very long message that is very long, therefore, it should take up multiple lines. In fact, it should take up so many lines that it takes up a lot of lines. Because of that, it is a good test to test displaying a large number a lines of stuff.").pack() + "\n";
        ts += HandleLogMessage(1, "Hello warning " + std::to_string(counter) + ".").pack() + "\n";
        ts += HandleLogMessage(2, "Hello error " + std::to_string(counter) + ".").pack() + "\n";
        server.AddMessage(frc846::base::Compression::compress(ts));
        counter += 1;
        counter %= 400;
    }
    return 0;
}