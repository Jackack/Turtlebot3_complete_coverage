import numpy as np
from numpy import savetxt

class readPGM():
    def __init__(self):
        f = open('map.pgm', 'rb')
        im = read_pgm(f)
        f.close()

    def read_pgm(pgmf):
    """Return a raster of integers from a PGM as a list of lists."""    
    print(pgmf.readline().decode('utf-8'))
    print(pgmf.readline().decode('utf-8'))    
    (width, height) = [int(i) for i in pgmf.readline().decode('utf-8').split()]

    print(width, height)

    depth = int(pgmf.readline().decode('utf-8'))
    assert depth <= 255

    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            val = ord(pgmf.read(1))
            if val == 254:
              val = 0
            else:
              val = 1
            row.append(val)
        raster.append(row)
    return raster
  
