# A re-implementation of upstream's sdf/embedSdf.rb tool in Python.
# Copied from the Drake project:
# https://github.com/RobotLocomotion/drake/blob/17423f8fb6f292b4af0b4cf3c6c0f157273af501/tools/workspace/sdformat/embed_sdf.py
import sys

assert __name__ == '__main__'

print("""
#include "EmbeddedSdf.hh"
namespace sdf { inline namespace SDF_VERSION_NAMESPACE {
const std::map<std::string, std::string>& GetEmbeddedSdf() {
  static const std::map<std::string, std::string> result{
""")
for filename in sorted(sys.argv[1:]):
    _, relative_path = filename.split('/sdf/')
    print('{')
    print(f'"{relative_path}",')
    with open(filename, 'r', encoding='utf-8') as data:
        print('R"raw(')
        sys.stdout.flush()
        sys.stdout.buffer.write(data.read().encode('utf-8'))
        print(')raw"')
    print('},')
print("""
  };
  return result;
}}}
""")
