#!/usr/bin/env ruby

# The list of supported SDF specification versions. This will let us drop
# versions without removing the directories.
supportedSdfVersions = ['1.6', '1.5', '1.4', '1.3', '1.2']

# The list of supported SDF conversions. This list includes versions that
# a user can convert an existing SDF version to.
supportedSdfConversions = ['1.6', '1.5', '1.4', '1.3']

puts %q!
#ifndef SDF_INTERNAL_EMBEDDEDSDF_HH_
#define SDF_INTERNAL_EMBEDDEDSDF_HH_

// An empty SDF string is returned if a query into the embeddedSdf map fails.
static const std::string emptySdfString = "";

// A map of maps where the keys in the first/parent map are SDF version
// strings, keys in the second/child map are SDF specification filenames and
// values are the contents of the SDF specification files.
static const std::map<std::string, std::map<std::string, std::string>> embeddedSdf = {
!

# Iterate over each version
supportedSdfVersions.each do |version|
  # Make sure the directory exists. Quietly fail so that we don't pollute
  # the output, which gets included in EmbeddedSdf.hh
  if Dir.exist?(version)
    puts "{\"#{version}\", {"

    # Iterate over each .sdf file in the version directory
    Dir.glob("#{version}/*.sdf") do |file|

      # Store the contents of the file in the child map
      puts "{\"#{File.basename(file)}\", R\"__sdf_literal__("
      infile = File.open(file)
      puts infile.read
      puts ")__sdf_literal__\"},"
    end
    puts "}},"
  end
end

puts "};"

puts "static const std::map<std::string, std::pair<std::string, std::string>> conversionMap = {"

# Iterate over each version
supportedSdfConversions.each do |version|
  # from-to
  # Make sure the directory exists. Quietly fail so that we don't pollute
  # the output, which gets included in EmbeddedSdf.hh
  if Dir.exist?(version)

    # Iterate over each .sdf file in the version directory
    Dir.glob("#{version}/*.convert") do |file|

      basename = File.basename(file, ".*").gsub(/_/, '.')
      # Store the contents of the file in the child map
      puts "{\"#{basename}\", {\"#{version}\", R\"__sdf_literal__("
      infile = File.open(file)
      puts infile.read
      puts ")__sdf_literal__\"}},"
    end
  end
end
puts %q!
};
#endif
!
