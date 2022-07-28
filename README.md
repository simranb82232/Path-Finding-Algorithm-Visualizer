# Path Finding Algorithm Visualizer
This program allows the user to select between a visualiztion of either the A* Search algorithm or the Dijkstra algorithm. After the user makes a selection, a window with a 50 x 50 grid of white squares is opened. To set the start node, the user clicks any square on the grid. The second click by the user sets the end node. After these first two clicks, any additional clicks by the user set obstacles on the grid that the program is not allowed to travel through. After this, the user can start the search by clicking the spacebar key, and the program will find the shortest path between the start and end nodes using the algorithm from the user's selection. This search pattern will also be visualized on the grid. At any time, the user can completely reset the grid by pressing the 'r' key.

# GUI
This image displays the result of an execution of this program using the A* Search algorithm. The blue square indicates the start point set by the user. The red square is the end point and the black squares are obstacle points set by the user. The orange squares represent all of the points which are looked at and weighted during the search. The purple squares are the end points of the search. The green squares show the shortest path determined by the search algorithm.

<img width="378" alt="PathFindingExample" src="https://user-images.githubusercontent.com/25395914/181396798-a80c3b25-f446-4a8a-af7b-496c50a2bb5e.PNG">

