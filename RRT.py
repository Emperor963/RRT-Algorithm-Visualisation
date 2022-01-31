import pygame
from base import *


def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obsdim = 30
    obsnum = 50
    iteration = 0

    pygame.init()
    map = TreeMap(start, goal, dimensions, obsdim, obsnum)
    graph = TreeGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobstacle()
    
    map.drawMap(obstacles)

    while (not graph.path_to_goal()):

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgethickness)
        else:
            X,Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgethickness)
            
        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1   


    map.drawPath(graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)



if __name__ == '__main__':
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False