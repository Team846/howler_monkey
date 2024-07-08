#pragma once

#include <functional>
#include <initializer_list>
#include <variant>

#include "frc846/loggable.h"

namespace frc846::control {

using SequenceSegment = std::pair<std::function<void()>, std::function<bool()>>;
using Sequence = std::vector<SequenceSegment>;

class Sequencer : Loggable {
 public:
  Sequencer(std::string name) : Loggable(name) {};

  void execute(std::string sequence_id, Sequence sequence,
               int forcedStage = -1) {
    if (sequence_id.compare(lastSequenceId) != 0) sequencePos = 0;
    SequenceSegment sequenceSegment = sequence.at(sequencePos);
    if ((sequenceSegment.second)()) {
      sequencePos += 1;
      sequencePos = std::min(sequencePos, (int)sequence.size() - 1);
    }
    SequenceSegment updatedSequenceSegment;
    if (forcedStage != -1) {
      updatedSequenceSegment = sequence.at(forcedStage);
    } else {
      updatedSequenceSegment = sequence.at(sequencePos);
    }

    (updatedSequenceSegment.first)();
  }

  void execute(std::string sequence_id, Sequence sequence,
               std::function<bool()>& preCondition,
               bool preConditionPersistent = false, int forcedStage = -1) {
    if (sequence_id.compare(lastSequenceId) != 0) sequencePos = 0;

    if ((!preConditionPersistent && sequencePos != 0) || preCondition()) {
      SequenceSegment sequenceSegment = sequence.at(sequencePos);
      if ((sequenceSegment.second)()) {
        sequencePos += 1;
        sequencePos = std::min(sequencePos, (int)sequence.size() - 1);
      }
      SequenceSegment updatedSequenceSegment;
      if (forcedStage != -1) {
        updatedSequenceSegment = sequence.at(forcedStage);
      } else {
        updatedSequenceSegment = sequence.at(sequencePos);
      }

      (updatedSequenceSegment.first)();
    }
  }

  void execute(std::string sequence_id, Sequence sequence,
               std::function<void()> onInit, int forcedStage = -1) {
    if (sequence_id.compare(lastSequenceId) != 0) {
      onInit();
      sequencePos = 0;
    }
    SequenceSegment sequenceSegment = sequence.at(sequencePos);
    if ((sequenceSegment.second)()) {
      sequencePos += 1;
      sequencePos = std::min(sequencePos, (int)sequence.size() - 1);
    }
    SequenceSegment updatedSequenceSegment;
    if (forcedStage != -1) {
      updatedSequenceSegment = sequence.at(forcedStage);
    } else {
      updatedSequenceSegment = sequence.at(sequencePos);
    }

    (updatedSequenceSegment.first)();
  }

  void execute(std::string sequence_id, Sequence sequence,
               std::function<bool()>& preCondition,
               std::function<void()> onInit,
               bool preConditionPersistent = false, int forcedStage = -1) {
    if (sequence_id.compare(lastSequenceId) != 0) {
      onInit();
      sequencePos = 0;
    }

    if ((!preConditionPersistent && sequencePos != 0) || preCondition()) {
      SequenceSegment sequenceSegment = sequence.at(sequencePos);
      if ((sequenceSegment.second)()) {
        sequencePos += 1;
        sequencePos = std::min(sequencePos, (int)sequence.size() - 1);
      }
      SequenceSegment updatedSequenceSegment;
      if (forcedStage != -1) {
        updatedSequenceSegment = sequence.at(forcedStage);
      } else {
        updatedSequenceSegment = sequence.at(sequencePos);
      }

      (updatedSequenceSegment.first)();
    }
  }

 private:
  std::string lastSequenceId{};
  int sequencePos{};
};

};  // namespace frc846::control