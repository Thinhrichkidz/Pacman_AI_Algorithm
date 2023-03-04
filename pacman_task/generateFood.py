import numpy as np
import os
from collections import deque

lay_files = [f for f in os.listdir('./layouts') if f.endswith('.lay')]

for file_name in lay_files:
    with open('./layouts/' + file_name) as f:
        content = f.readlines()

    content = [x.strip() for x in content]

    maze = np.empty((len(content), len(content[0])), dtype=str)

    for i in range(len(content)):
        for j in range(len(content[i])):
            maze[i][j] = content[i][j]

    maze[maze == ' '] = '.'
    file_name = file_name.replace('.lay', '_multi.lay')
    file_name = './layouts/' + file_name
    np.savetxt(file_name, maze, delimiter='', fmt='%s')