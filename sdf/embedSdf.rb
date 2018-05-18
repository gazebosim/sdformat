#!/usr/bin/env ruby

supportedSdfVersions = ['1.6', '1.5', '1.4', '1.3', '1.2']

puts "#ifndef SDF_EMBEDDEDSDF_HH_"
puts "#define SDF_EMBEDDEDSDF_HH_"

puts "const std::string emptySdfString = \"\";";
puts "const std::map<std::string, std::map<std::string, std::string>> embeddedSdf = {"

supportedSdfVersions.each do |version|
  puts "{\"#{version}\", {"
  Dir.glob("#{version}/*.sdf") do |file|
    puts "{\"#{File.basename(file)}\", R\"__sdf_literal__("
    infile = File.open(file)
    puts infile.read
    puts ")__sdf_literal__\"},"
  end
  puts "}},"
end

puts "};"
puts "#endif"
