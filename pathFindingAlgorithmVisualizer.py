# ********** NOTE ***************
# to set start we click on any node
# to set end we click on any node
# drag mouse click to set obstacles
# to start the algorithm press the spacebar key
# to restart press the 'r' key


import pygame
import math
from queue import PriorityQueue
import tkinter
from tkinter.messagebox import *

width = 500
rows = 50
window = pygame.display.set_mode((width, width))

# define square colors and their corresponding definitions
startBlue = (0, 0, 255)
endRed = (255, 0, 0)
obstacleBlack = (0, 0, 0)
backgroundWhite = (255, 255, 255)
enclosedOrange = (255, 150, 0)
openPurple = (120, 0, 120)
pathGreen = (0, 255, 0)
gridGrey = (128, 128, 128)

# create setters for all the colors and create getters for each color and position
class Square:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.width = width
        self.total_rows = total_rows
        self.x = row * width
        self.y = col * width
        self.color = backgroundWhite
        self.connectedSquares = []

    def reset(self):
        self.color = backgroundWhite

    def setClosed(self):
        self.color = enclosedOrange

    def setOpen(self):
        self.color = openPurple

    def setObstacle(self):
        self.color = obstacleBlack

    def setEnd(self):
        self.color = endRed

    def setStart(self):
        self.color = startBlue

    def setPath(self):
        self.color = pathGreen

    def getPosition(self):
        return self.row, self.col

    def getColor(self):
        return self.color

    def draw(self, window):
        pygame.draw.rect(window, self.color, (self.x, self.y, self.width, self.width))

    # method to add neighbors to the set and keep discovering nodes to get to the end node
    def updateConnectedSquares(self, grid):
        # create a set for the connected squares from the start node
        self.connectedSquares = []
        # if loop to check if we can discover nodes downwards
        # if there are no obstacles (black) we can append it to the set of connected squares
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].getColor() == obstacleBlack:
            self.connectedSquares.append(grid[self.row + 1][self.col])

        # if loop to check if we can discover nodes upwards
        # if there are no obstacles (black) we can append it to the set of connected squares
        if self.row > 0 and not grid[self.row - 1][self.col].getColor() == obstacleBlack:
            self.connectedSquares.append(grid[self.row - 1][self.col])

        # if loop to check if we can discover nodes to the left
        # if there are no obstacles (black) we can append it to the set of connected squares
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].getColor() == obstacleBlack:
            self.connectedSquares.append(grid[self.row][self.col + 1])

        # if loop to check if we can discover nodes to the right
        # if there are no obstacles (black) we can append it to the set of connected squares
        if self.col > 0 and not grid[self.row][self.col - 1].getColor() == obstacleBlack:
            self.connectedSquares.append(grid[self.row][self.col - 1])


# heuristic defined for A* pathfinder
# Manhattan distance used for the formula
def heuristic(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


# after setting all the nodes + obstacles,
# pathfinding algorithm will discover neighboring nodes and search for end node
# after searching through neighbors and reaching the end,
# a path will be drawn depicting the shortest distance used from start node to end node
def reconstructPath(came_from, end, draw):
    currentNode = end
    while currentNode in came_from:
        currentNode = came_from[currentNode]
        currentNode.setPath()
        draw()


# A* algorithm
# create a queue
# put function used to initialize f score and the node we start on
# create sets to store g score as well as f square (initialized to infinity)
# create g score of start node to 0
# create f score of start node using the heuristic function (f_score used to find best path)
# Resource used: https://www.redblobgames.com/pathfinding/a-star/introduction.html
def aStar(draw, grid, start, end):
    insert_count = 0
    frontier = PriorityQueue()
    frontier.put((0, insert_count, start))
    frontierSet = {start}
    came_from = dict()
    g = {square: float("inf") for rows in grid for square in rows}
    f = {square: float("inf") for rows in grid for square in rows}
    g[start] = 0
    f[start] = heuristic(start.getPosition(), end.getPosition())

    # while our frontier is not empty (discovering nodes)
    # we set our currentNode to our start node and add ti to our visited set
    # loop through all nodes which are connected to our current node,
    # and create a temporary g score which is one more than the previous node
    # if our new g score is less than the g score we currently have initialized,
    # set the g score to the new g score
    # set f score by adding the g score to the heuristic of the current to end position
    # update path or the node where we came from
    # loop through the neighbors and check if it is already visited
    # update the f count and add the neighbor into the set
    # repeat each step till we get to the end
    # once we get to the end, we set the start and end nodes and construct the shortest path
    while not frontier.empty():
        currentNode = frontier.get()[2]
        frontierSet.remove(currentNode)

        if currentNode == end:
            end.setEnd()
            reconstructPath(came_from, end, draw)
            start.setStart()
            return True

        for next in currentNode.connectedSquares:
            new_g = g[currentNode] + 1

            if next not in g or new_g < g[next]:
                g[next] = new_g
                priority = new_g + heuristic(next.getPosition(), end.getPosition())
                f[next] = priority
                came_from[next] = currentNode
                if next not in frontierSet:
                    insert_count += 1
                    frontier.put((f[next], insert_count, next))
                    frontierSet.add(next)
                    next.setOpen()

        draw()

        if currentNode != start:
            currentNode.setClosed()

    return False


# dijkstra algorithm
# very similar to A* explanation given above
# does not take a heuristic into consideration
# it only takes the g_score(given distance) into consideration
# discovers all connected neighbors rather than using a heuristic to skip some nodes that we do not need
# Resource used: https://www.redblobgames.com/pathfinding/a-star/introduction.html
def dijkstra(draw, grid, start, end):
    insert_count = 0
    frontier = PriorityQueue()
    frontier.put((0, insert_count, start))
    frontierSet = {start}
    came_from = dict()
    g = {square: float("inf") for rows in grid for square in rows}
    g[start] = 0

    while not frontier.empty():
        currentNode = frontier.get()[2]
        frontierSet.remove(currentNode)

        if currentNode == end:
            end.setEnd()
            reconstructPath(came_from, end, draw)
            start.setStart()
            return True

        for next in currentNode.connectedSquares:
            new_g = g[currentNode] + 1

            if next not in g or new_g < g[next]:
                g[next] = new_g
                came_from[next] = currentNode
                if next not in frontierSet:
                    insert_count += 1
                    frontier.put((g[next], insert_count, next))
                    frontierSet.add(next)
                    next.setOpen()

        draw()

        if currentNode != start:
            currentNode.setClosed()

    return False


def setGrid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            square = Square(i, j, gap, rows)
            grid[i].append(square)
    return grid


def drawGrid(window, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(window, gridGrey, (0, i * gap), (width, i * gap))
        pygame.draw.line(window, gridGrey, (i * gap, 0), (i * gap, width))


def draw(window, grid, rows, width):
    window.fill(backgroundWhite)

    for row in grid:
        for cub in row:
            cub.draw(window)

    drawGrid(window, rows, width)
    pygame.display.update()


def getClickedPos(pos, rows, width):
    x, y = pos
    gap = width // rows
    rows = x // gap
    col = y // gap
    return rows, col


# keyboard clicks customized
# to set start we click on any node
# to set end we click on any node
# drag mouse click to set obstacles
# to start the algorithm press the spacebar key
# to restart press the 'r' key
def main(window, width, rows):
    grid = setGrid(rows, width)

    run = True
    started = False

    start = None
    end = None

    while run:
        draw(window, grid, rows, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if started:
                continue

            elif pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = getClickedPos(pos, rows, width)
                square = grid[row][col]
                if not start and square != end:
                    start = square
                    square.setStart()
                    square.draw(window)
                elif not end and square != start:
                    end = square
                    square.setEnd()
                    square.draw(window)
                elif square != end and square != start:
                    square.setObstacle()
                    square.draw(window)
            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = getClickedPos(pos, rows, width)
                square = grid[row][col]
                if square == start:
                    start = None
                elif square == end:
                    end = None
                square.reset()
                square.draw(window)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for square in row:
                            square.updateConnectedSquares(grid)
                    if (VisualizeAStar):
                        aStar(lambda: draw(window, grid, rows, width), grid, start, end)
                    else:
                        dijkstra(lambda: draw(window, grid, rows, width), grid, start, end)
                if event.key == pygame.K_r:
                    start = None
                    end = None
                    grid = setGrid(rows, width)


root = tkinter.Tk()
root.withdraw()

# message box included to select between dijkstra's algorithm and A* algorithm
msg = tkinter.messagebox.askquestion('Selection',
                                     'Do you want to visualize the A star search algorithm?')

VisualizeAStar = False
if msg == "yes":
    VisualizeAStar = True
else:
    msg = tkinter.messagebox.askquestion('Selection',
                                         'Do you want to visualize the Dijkstra algorithm?')
    if msg == "no":
        exit(2)

main(window, width, rows)