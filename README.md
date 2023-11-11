## AI Lab

### Search Algorithm Implementation

This Python script implements various search algorithms (BFS, DFS, UCS, A*) to find the shortest path in a graph. The search algorithms are applied to a problem where the optimal path is the one with the shortest travel time.

### Requirements

- Python 3.11.5

### Usage

Create an `input.txt` file with the following format:

``` <ALGO>
   <START STATE>
   <GOAL STATE>
   <NUMBER OF LIVE TRAFFIC LINES>
   <... LIVE TRAFFIC LINES ...>
   <NUMBER OF SUNDAY TRAFFIC LINES>
   <... SUNDAY TRAFFIC LINES ...>
```

### Run the script
```python search_algorithm.py```

### Output Format
```<STATE> <ACCUMULATED TRAVEL TIME FROM START TO STATE>```


