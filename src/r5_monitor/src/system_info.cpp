#include "r5_monitor/system_info.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>

extern char ** environ;

namespace r5_monitor
{

namespace fs = std::filesystem;

namespace
{

std::optional<std::string> read_file(const fs::path & path)
{
  std::ifstream in(path);
  if (!in) {
    return std::nullopt;
  }
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  return s.substr(begin, s.find_last_not_of(" \t\r\n") - begin + 1);
}

std::optional<fs::path> find_repository(const std::vector<fs::path> & paths)
{
  for (const auto & path : paths) {
    for (fs::path p = path; !p.empty(); p = p.parent_path()) {
      std::error_code ec;
      if (fs::exists(p / ".git", ec)) {
        return p;
      }
      if (p == p.parent_path()) {
        break;
      }
    }
  }
  return std::nullopt;
}

// Runs a command without a shell and returns its standard output, or empty if it can't be run,
// fails or takes longer than timeout_s.
std::optional<std::string> run(const std::vector<std::string> & args, double timeout_s)
{
  int pipe_fds[2];
  if (::pipe2(pipe_fds, O_CLOEXEC) != 0) {
    return std::nullopt;
  }
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, pipe_fds[1], STDOUT_FILENO);
  posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, "/dev/null", O_WRONLY, 0);
  std::vector<char *> argv;
  for (const auto & a : args) {
    argv.push_back(const_cast<char *>(a.c_str()));
  }
  argv.push_back(nullptr);
  pid_t pid = 0;
  const int spawned = posix_spawnp(&pid, argv[0], &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  ::close(pipe_fds[1]);
  if (spawned != 0) {
    ::close(pipe_fds[0]);
    return std::nullopt;
  }

  std::string output;
  bool timed_out = false;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_s));
  while (true) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {
      timed_out = true;
      break;
    }
    pollfd pfd{pipe_fds[0], POLLIN, 0};
    const int ready = ::poll(&pfd, 1, static_cast<int>(left));
    if (ready < 0 && errno == EINTR) {
      continue;
    }
    if (ready <= 0) {
      timed_out = ready == 0;
      break;
    }
    char buffer[256];
    const ssize_t n = ::read(pipe_fds[0], buffer, sizeof(buffer));
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n <= 0) {
      break;  // End of output.
    }
    output.append(buffer, static_cast<std::size_t>(n));
  }
  ::close(pipe_fds[0]);
  if (timed_out) {
    ::kill(pid, SIGKILL);
  }
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  if (timed_out || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    return std::nullopt;
  }
  return output;
}

}  // namespace

std::optional<double> read_temperature_c(const fs::path & thermal_dir)
{
  std::optional<double> hottest;
  std::error_code ec;
  for (const auto & entry : fs::directory_iterator(thermal_dir, ec)) {
    if (entry.path().filename().string().rfind("thermal_zone", 0) != 0) {
      continue;
    }
    const auto text = read_file(entry.path() / "temp");
    if (!text) {
      continue;
    }
    char * end = nullptr;
    const std::string value = trim(*text);
    const long millidegrees = std::strtol(value.c_str(), &end, 10);
    if (value.empty() || *end != '\0') {
      continue;
    }
    const double c = millidegrees / 1000.0;
    hottest = hottest ? std::max(*hottest, c) : c;
  }
  return hottest;
}

std::optional<WifiSignal> parse_wireless(const std::string & text, const std::string & interface)
{
  std::istringstream lines(text);
  std::string line;
  // Two header lines.
  for (int i = 0; i < 2 && std::getline(lines, line); ++i) {
  }
  while (std::getline(lines, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string name = trim(line.substr(0, colon));
    std::istringstream fields(line.substr(colon + 1));
    std::string status, link, level;
    if (!(fields >> status >> link >> level) || (!interface.empty() && name != interface)) {
      continue;
    }
    // The level is written like "-52." .
    char * end = nullptr;
    double dbm = std::strtod(level.c_str(), &end);
    if (end == level.c_str()) {
      continue;
    }
    // Some drivers report an unsigned byte, where 256 means 0 dBm.
    if (dbm > 0) {
      dbm -= 256;
    }
    return WifiSignal{name, dbm};
  }
  return std::nullopt;
}

std::optional<WifiSignal> read_wifi(const fs::path & wireless, const std::string & interface)
{
  const auto text = read_file(wireless);
  if (!text) {
    return std::nullopt;
  }
  return parse_wireless(*text, interface);
}

std::string git_version(const std::vector<fs::path> & paths, double timeout_s)
{
  const auto repository = find_repository(paths);
  if (!repository) {
    return "unknown";
  }
  const auto output = run(
    {"git", "-c", "safe.directory=*", "-C", repository->string(), "describe", "--always",
      "--dirty", "--tags"},
    timeout_s);
  return output ? trim(*output) : "unknown";
}

}  // namespace r5_monitor
