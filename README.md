<h1>Pacman Project</h1>

This is a project that implements different search algorithms to solve the classic Pacman game. The project uses the A\* search algorithm, Breadth-First Search (BFS), Depth-First Search (DFS), and Uniform Cost Search (UCS) to help Pacman find the shortest path to eat one or multiple food in the provided map without using the keyboard to control him and with no ghost in the game.

---

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for testing purposes.

**Prerequisites**

To run this project, you need to have Python 3 installed on your machine. You can download and install Python 3 from the [official website](https://www.python.org/downloads/).

### Installing

To install the project, you can clone the repository using the following command:

   git clone https://github.com/your\_username/pacman.git

### Running the project

To run the project, you can navigate to the directory where you cloned the repository and run the following command:

   python pacman.py -l \<layout\_file\> -p \<search agent\> -z \<zoom level\>

where **\<**** search agent ****\>** is the search agents algorithm you want to use ( **astar** , **bfs** , **dfs** , or **ucs** ) and **\<**** layout\_file ****\>** is the map you want to choose

For example, to run the project using the A\* search algorithm with the mediumClassic maze, you can run the following command:

   run pacman.py -l mediumClassic -p BFSFoodSearchAgent

### Files in the repository

- **search.py** : contains the implementation of the search algorithms.
- **searchAgents.py** : contains the implementation of the search agents (i.e., Pacman and Ghosts).
- **pacman.py** : contains the main code for running the game.
- **layouts/** : contains the different maps for the game.
- **images/** : contains the images used in the game.

### Acknowledgments

This project is based on the classic Pacman game and was implemented as part of a course assignment. The game engine and some of the code were provided by the instructors. The implementation of the search algorithms and the search agents were done by the author of this README file.
