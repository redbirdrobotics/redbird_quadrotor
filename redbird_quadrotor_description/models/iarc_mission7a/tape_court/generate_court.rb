#!/usr/bin/env ruby

# This script will generate an arena (tape strip grid) based on IARC specs.
# For more detail, see https://github.com/redbirdrobotics/corvus/raw/bd86581eacb976aad8b599dfbe0c9c6f48c775e6/docs/iarc_mission7a_arena_layout.jpg

require 'erb'
require 'fileutils'

def degrees_to_radians(degrees)
  return degrees * Math::PI / 180.0
end


# %%%%%%%%%%%%%%% CONFIGURATIONS %%%%%%%%%%%%%%%
COLUMN_ATTITUDE_RADIANS = {
  :roll  => 0.0,
  :pitch => 0.0,
  # notice: this yaw must correlate to relative (x,y) coordinate shift between columns
  :yaw   => degrees_to_radians(90.0),
}
ROW_ATTITUDE_RADIANS = {
  :roll  => 0.0,
  :pitch => 0.0,
  # row yaw is orthogonal to column yaw:
  :yaw   => COLUMN_ATTITUDE_RADIANS[:yaw] + degrees_to_radians(90.0),
}
WIDTH_COURT = 20   # integral meters
WIDTH_TILE = 1     # integral meters
WIDTH_TAPE = 0.05  # meters
COURT_ALTITUDE = 0.0
# %%%%%%%%%%%%%%% END CONFIGURATIONS %%%%%%%%%%%%%%%

TAPE_THICKNESS = 0.0003 # (meters) 0.3mm ~ 12 mils ~ duct tape thickness

module TapeStripUri
  WHITE =  'model://tape_strip'
  RED =    'model://tape_strip_red'
  GREEN =  'model://tape_strip_green'
end



# where do these formulas (below) come from? see link in header above

# (x,y) coordinates of column i
def column_position_impl(i)
  raise ArgumentError, 'must be i = 0,1,2,...' unless i >= 0
  x = WIDTH_TAPE / 2 + i * WIDTH_TILE
  y = (0.0 + WIDTH_COURT + WIDTH_TAPE) / 2.0
  return x, y
end

# (x,y) coordinates of column i
def column_position(i)
  x, y = column_position_impl(i)
  return x, y
end

# (x,y) coordinates of row i
def row_position(i)
  # notice we can just flip (x,y) -> (y,x) from column -> row
  y, x = column_position_impl(i)
  return x, y
end


def main
  columns = []
  rows = []
  
  column_attitude_str = "#{COLUMN_ATTITUDE_RADIANS[:roll]} #{COLUMN_ATTITUDE_RADIANS[:pitch]} #{COLUMN_ATTITUDE_RADIANS[:yaw]}"
  row_attitude_str = "#{ROW_ATTITUDE_RADIANS[:roll]} #{ROW_ATTITUDE_RADIANS[:pitch]} #{ROW_ATTITUDE_RADIANS[:yaw]}"
  z = COURT_ALTITUDE
  num_columns_and_rows = WIDTH_COURT + 1
  
  num_columns_and_rows.times do |i|
    c_x, c_y = column_position(i)
    r_x, r_y = row_position(i)

    columns << {
      :pose  => "#{c_x} #{c_y} #{z} #{column_attitude_str}",
      :uri => TapeStripUri::WHITE,
    }
    
    # row 0: GREEN and lays atop all other strips
    # row n: RED   and lays atop all other strips
    # the rest are WHITE, at court height
    row_properties = lambda do
      uri = TapeStripUri::WHITE
      altitude = z

      if i == 0 then
        uri = TapeStripUri::GREEN
        altitude += TAPE_THICKNESS
      elsif i == (num_columns_and_rows - 1)
        uri = TapeStripUri::RED
        altitude += TAPE_THICKNESS
      end

      return uri, altitude
    end
    
    row_uri, row_z = row_properties.call
    rows << {
      :pose  => "#{r_x} #{r_y} #{row_z} #{row_attitude_str}",
      :uri => row_uri,
    }
  end

  File.open('model.sdf' % @output_path, 'w') do |file|
    template = ERB.new File.read('model.sdf.erb')
    file.write template.result(binding)
  end
end

main()
