#ifndef TEMP_DIR_HPP_
#define TEMP_DIR_HPP_

#include <gtest/gtest.h>

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

// A test fixture with an empty directory, removed afterwards.
class TempDir : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    dir_ = std::filesystem::temp_directory_path() /
      ("r5_dashboard_" + std::to_string(::getpid()) + "_" + info->name());
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override {std::filesystem::remove_all(dir_);}

  void write(const std::string & name, const std::string & text)
  {
    std::filesystem::create_directories((dir_ / name).parent_path());
    std::ofstream(dir_ / name) << text;
  }

  std::string read(const std::string & name)
  {
    std::ifstream in(dir_ / name);
    return std::string(std::istreambuf_iterator<char>(in), {});
  }

  std::filesystem::path dir_;
};

#endif  // TEMP_DIR_HPP_
