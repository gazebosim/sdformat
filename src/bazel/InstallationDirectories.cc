// Generate an installation directories file for bazel specifically

#include <sdf/InstallationDirectories.hh>

namespace sdf
{

inline namespace SDF_VERSION_NAMESPACE {

std::string getInstallPrefix()
{
  return ".";
}
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf