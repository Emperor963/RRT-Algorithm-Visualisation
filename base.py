import random 
import math
import pygame

class TreeMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions 

        #color settings
        self.white = (255, 255, 255)
        self.blue = (0, 0,255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.grey = (70, 70, 70)

        #window settings
        self.MapWindowName = 'RRT Path Planning Algorithm'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgethickness = 1

        self.obstacles = []
        self.obsdim = obsdim 
        self.obsNumber = obsnum


    def drawMap(self, obstacles): # draws the map
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad+5, 0)
        pygame.draw.circle(self.map, self.green, self.goal, self.nodeRad+20, 1)
        self.drawObstacles(obstacles)

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node, 3, 0)

    def drawObstacles(self, obstacles): # draws the obstacles one by one
        obstaclesList = obstacles.copy()
        while(len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)

class TreeGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x,y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.Maph, self.Mapw = MapDimensions 
        self.x = []
        self.y = []
        self.parent = []

        #tree initialisation
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        #this is to create obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum

        #the path itself 
        self.goalstate = None
        self.path = []


    def makeRandomRect(self): # to generate the coordinate of the upper left corner of an obstacle
        uppercornerx = int(random.uniform(0, self.Mapw-self.obsDim))
        uppercornery = int(random.uniform(0, self.Maph-self.obsDim))
    

        return(uppercornerx, uppercornery)


    def makeobstacle(self): # makes the obstacle
        obs = []

        for i in range(0,self.obsNum):
            rectangle = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectangle = pygame.Rect(upper, (self.obsDim, self.obsDim))

                startgoalcol = rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal)
            obs.append(rectangle)

        self.obstacles = obs.copy()  
        return obs       


    def add_node(self, n, x, y):
        self.x.insert(n,x)
        self.y.append(y)


    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)


    def add_edge(self, parent, child):
        self.parent.insert(child, parent)


    def remove_edge(self, n):
        self.parent.pop(n)


    def number_of_nodes(self):
        return len(self.x)


    def distance(self, n1, n2): 
        (x1,y1) = (self.x[n1], self.y[n1])
        (x2,y2) = (self.x[n2], self.y[n2])
        px = (float(x2) - float(x1))**2
        py = (float(y2) - float(y1))**2
        return (px+py)**0.5


    def sample_envir(self):
        x = int(random.uniform(0, self.Mapw))
        y = int(random.uniform(0, self.Maph))
        return x,y 


    def nearest(self, n):
        dmin = self.distance(0,n)
        n_near = 0
        for i in range (0,n):
            if self.distance(i,n) < dmin: 
                dmin = self.distance(i,n)
                n_near = i 
        return n_near


    def isFree(self):
        n = self.number_of_nodes()-1
        (x,y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectangle = obs.pop(0)
            if rectangle.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True       


    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rectangle = obs.pop(0)
            for i in (0, 101):
                u = i / 100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if rectangle.collidepoint(x,y):
                    return True
        return False           


    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True


    def step(self, n_near, n_rand, dmax=35):
        d = self.distance(n_near, n_rand)
        if d > dmax: 
            u = dmax/d
            (xnear, ynear) = (self.x[n_near], self.y[n_near])
            (xrand, yrand) = (self.x[n_rand], self.y[n_rand])
            (px, py) = (xrand-xnear, yrand-ynear)
            theta = math.atan2(px,py)
            (x,y) = (int(xnear + dmax*math.sin(theta)), int(ynear + dmax*math.cos(theta)))
            self.remove_node(n_rand)
            if abs(x-self.goal[0]) <= dmax and abs(y-self.goal[1]) <= dmax:
                self.add_node(n_rand, self.goal[0], self.goal[1])
                self.goalstate = n_rand
                self.goalFlag = True 
            else:
                self.add_node(n_rand, x, y)
             

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            new_position = self.parent(self.goalstate)
            while (new_position != 0):
                self.path.append(new_position)
                new_position = self.parent(new_position)
            self.path.append(0)
        return self.goalFlag

    
    def getPathCoords(self):
        pathcoords = []
        for node in self.path:
            x,y = (self.x[node], self.y[node])
            pathcoords.append((x,y))
        return pathcoords


    def bias(self, n_goal):
        n = self.number_of_nodes()
        self.add_node(n, n_goal[0], n_goal[1])
        n_near = self.nearest(n)
        self.step(n_near, n)
        self.connect(n_near, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x,y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent


    def cost(self):
        pass




