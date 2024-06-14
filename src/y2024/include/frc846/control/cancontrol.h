// #ifndef FRC846_CAN_CONTROL_H_
// #define FRC846_CAN_CONTROL_H_
// #include <map>
// #include <queue>

// #include "basecontrol.h"

// namespace frc846::control {

// struct Message {
//   unsigned int id;
//   std::function<void(BaseElectronicSpeedController*)>& msg;
// };

// class CANControl {
//  public:
//   static void loop() {
//     msg_count = 0;

//     for (; msg_queue.size() != 0x00;) {
//       Message m = msg_queue.front();

//       auto exec = m.msg;

//       exec(controller_catalog.at(m.id));
//     }
//   }

//   static unsigned int AttachController(
//       BaseElectronicSpeedController* controller) {
//     controller_catalog[id_counter] = controller;
//     id_counter++;
//     return id_counter - 0x01;
//   };

//   static unsigned int access(
//       unsigned int id,
//       std::function<void(BaseElectronicSpeedController*)>& msg) {
//     if (msg_count < max_msg_count && controller_catalog.contains(id)) {
//       msg_queue.push({id, msg});

//       msg_count++;

//       return 0x00;
//     }
//     return 0x01;
//   }

//   static unsigned int retrieve_error(unsigned int id) {
//     if (error_catalog.contains(id)) {
//       return error_catalog.at(id);
//     }
//     return 0x00;
//   }

//  private:
//   static std::map<unsigned int, BaseElectronicSpeedController*>
//       controller_catalog;

//   static std::map<unsigned int, unsigned int> error_catalog;

//   static std::queue<Message> msg_queue;

//   static unsigned int id_counter;

//   static int max_msg_count;

//   static int msg_count;
// };
// };  // namespace frc846::control

// #endif