#ifndef R5_DASHBOARD__FILES_HPP_
#define R5_DASHBOARD__FILES_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace r5_dashboard
{

// Writes text to path atomically, with permissions 0600, creating the directory if needed. Throws
// std::filesystem::filesystem_error.
void write_private_file(const std::filesystem::path & path, const std::string & text);

// The whole file, or empty if it doesn't exist. Throws std::filesystem::filesystem_error if it
// exists but can't be read.
std::optional<std::string> read_file(const std::filesystem::path & path);

// "~/x" to "$HOME/x".
std::filesystem::path expand_user(const std::string & path);

}  // namespace r5_dashboard

#endif  // R5_DASHBOARD__FILES_HPP_
