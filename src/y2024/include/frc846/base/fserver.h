#pragma once

#include <cstdint>
#include <vector>

#ifdef _WIN32

namespace frc846::base {

class LoggingServer {
 public:
  /*
  THIS FUNCTION IS NOT IMPLEMENTED FOR WINDOWS.
  */
  LoggingServer() {};

  /*
  THIS FUNCTION IS NOT IMPLEMENTED FOR WINDOWS.

  Starts a UDP logging server on the specified port. Make sure port
  is FRC legal in FMS. As of 2024, these include 5800...5810. Spawns threads for
  logging and watching clients.
  @param port port to start the server on
  */
  void Start(int port) {};

  /*
  THIS FUNCTION IS NOT IMPLEMENTED FOR WINDOWS.

  Adds message to the sending queue. Messages will not be sent out unless server
  has been started. Blocking operation while server is reading message.
  */
  void AddMessage(std::vector<uint8_t> message) {};
};

};  // namespace frc846::base

#else

#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <queue>
#include <thread>
#include <unordered_set>

struct ClientAddress {
  sockaddr_in address;
  bool operator==(const ClientAddress &other) const {
    return address.sin_addr.s_addr == other.address.sin_addr.s_addr &&
           address.sin_port == other.address.sin_port;
  }
};

namespace std {
template <>
struct hash<ClientAddress> {
  std::size_t operator()(const ClientAddress &addr) const {
    return hash<std::string>()(std::string((char *)&addr.address.sin_addr,
                                           sizeof(addr.address.sin_addr))) ^
           hash<uint16_t>()(addr.address.sin_port);
  }
};
};  // namespace std

namespace frc846::base {

class LoggingServer {
 public:
  LoggingServer() : messages{}, msg_mtx{}, client_mtx{} {};

  /*
  Starts a UDP logging server on the specified port. Make sure port is FRC legal
  in FMS. As of 2024, these include 5800...5810. Spawns threads for logging and
  watching clients.
  @param port port to start the server on
  */
  void Start(int port) {
    int sockfd;
    sockaddr_in servaddr, cliaddr;
    std::unordered_set<ClientAddress> clients;
    socklen_t len = sizeof(cliaddr);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) return;

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
      return;

    std::thread sender([&]() {
      while (true) {
        while (messages.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        msg_mtx.lock();
        auto msg = messages.front();
        messages.pop();
        msg_mtx.unlock();

        client_mtx.lock();
        for (const auto &client : clients) {
          sendto(sockfd, msg.data(), msg.size(), 0,
                 (const struct sockaddr *)&client, len);
        }
        client_mtx.unlock();
      }
    });

    std::thread receiver([&]() {
      char buffer[1024];
      while (true) {
        recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr,
                 &len);
        client_mtx.lock();
        clients.insert({cliaddr});
        client_mtx.unlock();
      }
    });

    sender.detach();
    receiver.detach();
  }

  /*
  Adds message to the sending queue. Messages will not be sent out unless server
  has been started. Blocking operation while server is reading message.
  */
  void AddMessage(std::vector<uint8_t> message) {
    msg_mtx.lock();
    messages.push(message);
    msg_mtx.unlock();
  }

 private:
  std::queue<std::vector<uint8_t> > messages;
  std::mutex msg_mtx;
  std::mutex client_mtx;
};

};  // namespace frc846::base

#endif