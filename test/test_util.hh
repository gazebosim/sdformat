#include "sdf/Filesystem.hh"

#include "test_config.h"

#define SDF_TMP_DIR "tmp-sdf/"

namespace sdf
{
  namespace testing
  {
    bool TestDataPath(std::string &_dataDir)
    {
      std::string dataDir;
      if(char* dataDir = std::getenv("TEST_SRCDIR"))
      {
        _dataDir = sdf::filesystem::append(dataDir, "__main__/sdformat");
        std::cout << "1\n";
        return true;
      }
      else
      {
        _dataDir = PROJECT_SOURCE_PATH;
        return true;
      }

      return false;
    }

    bool TestTmpPath(std::string &_tmpDir)
    {
      std::string tmpDir;

      if (const char* tmpDir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR"))
      {
        _tmpDir = tmpDir;
        return true;
      }

      std::string homeDir;
      if (char* homeDir = std::getenv("HOME"))
      {
        _tmpDir = sdf::filesystem::append(homeDir, SDF_TMP_DIR);
        return true;
      }

      return false;
    }
  }
}
