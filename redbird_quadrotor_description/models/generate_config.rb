#!/usr/bin/env ruby

require 'erb'
require 'fileutils'

def usage
  puts 'Usage: %s <model display name> <description>' % $PROGRAM_NAME
end

def main
  if ARGV.count != 2
    usage()
    return
  end

  model = {
    :display_name => ARGV[0],
    :description  => ARGV[1],
  }

  puts 'generating model.config file with options:'
  puts '  model = %s' % model

  File.open('model.config' % @output_path, 'w') do |file|
    template = ERB.new File.read('model.config.erb')
    file.write template.result(binding)
  end
end

main()
