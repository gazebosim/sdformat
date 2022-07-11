#!/usr/bin/env ruby

# The list of supported SDF specification versions. This will let us drop
# versions without removing the directories.
supportedSdfVersions = ['1.10', '1.9', '1.8', '1.7', '1.6', '1.5', '1.4', '1.3', '1.2']

# The list of supported SDF conversions. This list includes versions that
# a user can convert an existing SDF version to.
supportedSdfConversions = ['1.10', '1.9', '1.8', '1.7', '1.6', '1.5', '1.4', '1.3']

puts %q!
#include "EmbeddedSdf.hh"

namespace sdf {
inline namespace SDF_VERSION_NAMESPACE {

const std::map<std::string, std::string> &GetEmbeddedSdf() {
  static const std::map<std::string, std::string> result{
!

# Stores the contents of the file in the map.
def embed(pathname)
  puts "{\"#{pathname}\", R\"__sdf_literal__("
  infile = File.open(pathname)
  puts infile.read
  puts ")__sdf_literal__\"},"
end

# Embed the supported *.sdf files.
supportedSdfVersions.each do |version|
  Dir.glob("#{version}/*.sdf").sort.each { |file| embed(file) }
end

# Embed the supported *.convert files.
supportedSdfConversions.each do |version|
  Dir.glob("#{version}/*.convert").sort.each { |file| embed(file) }
end
puts %q!
  };
  return result;
}

}
}
!
