# Selfless Traffic Routing Problem

### Problem Description

This problem defines a "real-time" traffic routing policy for a set of vehicles in a given map. We have to design an algorithm to this optimization problem such that each vehicle should be able to reach their destination within their corresponding deadlines as soon as possible.

### Algorithm Used

I decided the use the A* Search Algorithm to solve this problem. The reason why I chose this algorithm is because it can reliably find the shortest path using several heuristic functions:
- g: The distance between the successor and the current edge
- h: Distance between the successor and the goal node
- f: g + h

A* Search will choose which path to take based on the minima of `f`. A* Search is derived from Dijkstra's algorithm, but it is classified as a "smart" algorithm because of it's use of additional heuristics to solve graph traversal problems reliably.

### How to use

1. Install dependencies by running `pip install -r requirements.txt`
2. Ensure SUMO is installed properly: https://sumo.dlr.de/docs/Installing/index.html
3. Simply run `python main.py` to see the results
