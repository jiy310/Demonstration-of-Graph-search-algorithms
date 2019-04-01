## Demonstration-of-Graph-search-algorithms:

This program implements some graph search algorithms, such as DFS, BFS, Dijkstra and A*.

For DFS and BFS, they are used to find shortest path in an undirected graph. 

On the contrary, for Dijkstra and A*, they are mainly focused on solving shortest path question in a directed graph.

Note that for A*, we use a herustic function to find a "fairly" good path between start and end point. 

Compare to the "naive" Dijkstra, it runs faster but not neccessarily return the best result.

## How to run this project

Simply run `python gridworld.py` in the current program directory.
(For mac user, python 2 recommended)

After the pygame GUI initializes, follow the instructions on the board:
Press Enter to find path or pause, press 'c' to clear board
Press 1 for DFS, 2 for BFS, 3 for UCS, 4 for A*
