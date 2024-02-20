#include <frc/RobotBase.h>
#include "frc/Filesystem.h"

#include "FunkyRobot.h"

#include <signal.h>
#include <execinfo.h>
#include <dirent.h>
#include <algorithm> 
#include <cctype>
#include <locale>
#include <unistd.h>
#include <sys/stat.h>
#include <chrono>

inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

#define A 54059
#define B 76963
#define C 86969
#define FIRSTH 37
unsigned hash_str(const char* s)
{
   unsigned h = FIRSTH;
   while (*s) {
     h = (h * A) ^ (s[0] * B);
     s++;
   }
   return h % C;
}

void handler(int sig) {
  std::map<int, std::string> sigErrors;

  sigErrors[SIGFPE] = "FATAL ERROR >> Arithmetic Error, SIGFPE";
  sigErrors[SIGILL] = "FATAL ERROR >> Illegal Instruction, SIGILL";
  sigErrors[SIGSEGV] = "FATAL ERROR >> Illegal Memory Access, SIGSEGV";
  sigErrors[SIGBUS] = "FATAL ERROR >> Bus Error, SIGBUS";
  sigErrors[SIGABRT] = "FATAL ERROR >> Abort, SIGABRT";
  sigErrors[SIGSYS] = "FATAL ERROR >> Invalid system call, SIGSYS";
  sigErrors[SIGTRAP] = "FATAL ERROR >> Exception Occured, SIGTRAP";

  void *callstack[24];

  int frames = backtrace(callstack, 24);


  std::cerr << "Backtrace:\n";
  char** symbols = backtrace_symbols(callstack, frames);

  if (symbols == nullptr) {
      std::cerr << "Error obtaining backtrace symbols\n" << std::endl;
      return;
  }

  for (int i = 0; i < frames; ++i) {
      std::cerr << symbols[i] << std::endl;
  }

  if (sigErrors.contains(sig)) {
    std::cerr << sigErrors[sig] << std::endl;
  } else {
    std::cerr << "? Unknown Exception Occured" << std::endl;
  }
  // exit(1);
}


int main() {
  for (int i = 1; i < NSIG; ++i) {
    signal(i, handler);
  }

  unsigned int validDevIds[]{72214, 69948,};
  bool devIdFound = false;

  std::string file_path = frc::filesystem::GetDeployDirectory()+"/dev.id";
  try {
      std::cout << "Trying to find dev.id" << std::endl;
      std::ifstream ifs(file_path);
      std::ostringstream oss;
      oss << ifs.rdbuf();

      std::string devId = oss.str();
      rtrim(devId);
      ltrim(devId);

      auto devIdHash = hash_str(devId.c_str());

      std::cout << "Dev.id hash key found: " << devIdHash << std::endl;

      for (auto id : validDevIds) {
        if (id == devIdHash) {
          devIdFound = true;
        }
      }
  } catch (std::exception exc) {
    std::cout << "Error processing dev.id." << std::endl;
  }

  if (!devIdFound) {
    std::cerr << "Dev ID not set or did not match, exiting" << std::endl;
    exit(0);
  } else {
    std::cout << "Valid dev ID found" << std::endl;
    std::cout << "Last deploy: ";

    struct stat file_stat;

    if (stat(file_path.c_str(), &file_stat) == 0) {
        char time_buffer[256];
        strftime(time_buffer, sizeof(time_buffer), "%F %T", localtime(&file_stat.st_mtime));
        std::cout << time_buffer << std::endl;
        auto current_time = time(0);
        std::cout << "Current Time: " << std::ctime(&current_time) << std::endl;
    } else {
        std::cout << "Error getting file status: "  << std::endl;
    }
  }

  std::cout << "Starting robot code [2024]..." << std::endl;
  return frc::StartRobot<FunkyRobot>();
}