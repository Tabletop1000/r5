#include "r5_dashboard/files.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <system_error>
#include <vector>

namespace r5_dashboard
{

namespace fs = std::filesystem;

namespace
{

[[noreturn]] void fail(const std::string & what, const fs::path & path)
{
  throw fs::filesystem_error(what, path, std::error_code(errno, std::generic_category()));
}

}  // namespace

void write_private_file(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::string tmp = (path.parent_path() / ("." + path.filename().string() + ".XXXXXX")).string();
  std::vector<char> name(tmp.begin(), tmp.end());
  name.push_back('\0');
  const int fd = ::mkstemp(name.data());  // Created with permissions 0600.
  if (fd < 0) {
    fail("Can't create a file next to", path);
  }
  const fs::path tmp_path(name.data());
  const char * p = text.data();
  std::size_t left = text.size();
  while (left > 0) {
    const ssize_t n = ::write(fd, p, left);
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0) {
      const int err = errno;
      ::close(fd);
      ::unlink(tmp_path.c_str());
      errno = err;
      fail("Can't write", tmp_path);
    }
    p += n;
    left -= static_cast<std::size_t>(n);
  }
  if (::fsync(fd) != 0 || ::close(fd) != 0) {
    const int err = errno;
    ::unlink(tmp_path.c_str());
    errno = err;
    fail("Can't write", tmp_path);
  }
  if (::rename(tmp_path.c_str(), path.c_str()) != 0) {
    const int err = errno;
    ::unlink(tmp_path.c_str());
    errno = err;
    fail("Can't replace", path);
  }
}

std::optional<std::string> read_file(const fs::path & path)
{
  std::error_code ec;
  if (!fs::exists(path, ec) && !ec) {
    return std::nullopt;
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    errno = errno ? errno : EIO;
    fail("Can't read", path);
  }
  std::ostringstream out;
  out << in.rdbuf();
  if (in.bad()) {
    fail("Can't read", path);
  }
  return out.str();
}

fs::path expand_user(const std::string & path)
{
  if (path == "~" || path.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    if (home) {
      return fs::path(home) / path.substr(path.size() > 1 ? 2 : 1);
    }
  }
  return path;
}

}  // namespace r5_dashboard
