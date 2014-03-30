#****************************************************************************************
# File: maze_converter.cpp
#
# Description: TODO
#
# Created: 03/28/2014, by Kyle McGahee
#****************************************************************************************

import os
import sys
import struct

__author__  = "Kyle McGahee"
__version__ = "1.0"
     
#----------------------------------------------------------------------------------------
class MazeCell:
    
    #------------------------------------------------------------------------------------
    def __init__(self):
        self.has_north_wall = False
        self.has_east_wall  = False
        self.has_south_wall = False
        self.has_west_wall  = False
     
#----------------------------------------------------------------------------------------
class MazeConverter:

    #------------------------------------------------------------------------------------
    def __init__(self, maze_writer):
        self.maze_writer = maze_writer

    #------------------------------------------------------------------------------------
    def convert(self, filename):
        with open(filename, 'r') as file:
            maze_lines = file.readlines()
            
        # Filter out comment lines
        maze_lines = [line for line in maze_lines if not line.startswith('#')]
            
        number_rows = len(maze_lines) - 1 
        number_columns = (len(maze_lines[1].strip()) - 1) / 2
        
        print '\nNumber of rows: ' + str(number_rows)
        print 'Number of cols: ' + str(number_columns)
        
        cells = []
        
        for i, line in enumerate(maze_lines):

            # First line is not really a row.  It's the top of the first row.
            if i == 0: continue
        
            line = line.strip()

            for column, character in enumerate(line):
                # Only process odd numbered columns since those are the actual cell positions.
                if column % 2 == 0: continue

                cell = MazeCell()
                cell.has_north_wall = self.is_wall(maze_lines[i-1], column)
                cell.has_east_wall  = self.is_wall(maze_lines[i], column+1)
                cell.has_south_wall = self.is_wall(maze_lines[i], column)
                cell.has_west_wall  = self.is_wall(maze_lines[i], column-1)
        
                cells.append(cell)

        self.maze_writer.write_to_file(cells, 'maze_hex.h')
                
        return cells
        
    #------------------------------------------------------------------------------------
    def is_wall(self, maze_line, column):
        return maze_line[column] == '|' or maze_line[column] == '_'

#----------------------------------------------------------------------------------------
class HexMazeWriter:
    
    #------------------------------------------------------------------------------------
    def write_to_file(self, cells, filename):
        self.directory = os.path.splitext(filename)[0] + '/'

        with open(filename, 'w') as file:
            for cell in cells:
                cell_hex_value = 0x00
                if cell.has_north_wall: cell_hex_value |= 0x08
                if cell.has_east_wall:  cell_hex_value |= 0x04
                if cell.has_south_wall: cell_hex_value |= 0x02
                if cell.has_west_wall:  cell_hex_value |= 0x01
                file.write(hex(cell_hex_value) + ',\n')
                
#----------------------------------------------------------------------------------------
def main():
    filename = sys.argv[1]
    maze_converter = MazeConverter(HexMazeWriter())
    print "\nAttempting to convert maze file..."
    cells = maze_converter.convert(filename)
    print "\nConversion complete.  Wrote " + str(len(cells)) + " cells to header file."
    
    

#----------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()