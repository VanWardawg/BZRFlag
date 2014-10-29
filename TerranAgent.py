import sys
import math
import time
from numpy import *
from random import randrange
from fields import PotentialFieldPlotter
from grid_filter_gl import GridFilter
import threading
from bzrc import BZRC, Command, UnexpectedResponse, Location

class Marine(threading.Thread):
    def __init__(self,marine, index, commandCenter,constants):
        threading.Thread.__init__(self)
        self.me = marine
        self.myIndex = index
        self.commandCenter = commandCenter
        self.constants = constants
        self.sensorTower = SensorTower(constants, commandCenter)
        self.commands = []
        self.swapTeams = 0
        self.shootTime = 0
        self.recalculateTime = 0
        self.moveTime = 0
        #self.sensorTower.visualize_potential_field()
        self.frozenTime = 0
        self.calculateGridTime = 0
        self.new = True
        print "Marine " + str(index) + " ready to go!"

    def move_next(self):
        self.sensorTower.move_next()
        print "Marine moving to new location"

    def run(self):
        prev_time = time.time()
        while True:
            time_diff = time.time() - prev_time
            self.tick(time_diff)

    def tick(self, time_diff):
        if self.shootTime ==0 and self.recalculateTime == 0 and self.swapTeams == 0:
            self.shootTime = time_diff
            self.recalculateTime = time_diff
            self.swapTeams = time_diff

        if self.calculateGridTime == 0:
            self.calculateGridTime = time_diff
        self.commands = []
        oldme = self.me
        self.me = self.commandCenter.get_marine(self.myIndex)

        if time_diff - self.shootTime > 1:
            self.shoot()
            self.shootTime = time_diff

        if self.me.x == oldme.x and self.me.y == oldme.y:
            self.frozenTime+=1
        else:
            self.frozenTime = 0
        if self.frozenTime > 100:
            print "Marine " + str(self.myIndex) + " is stuck... moving to better location!"
            self.frozenTime -=1
            command = Command(self.me.index, 1, math.radians(randrange(0,180)), False)
            self.commandCenter.do_commands([command])

        if time_diff - self.recalculateTime > .10:
            self.sensorTower.recalculate()
            self.recalculateTime = time_diff

        if time_diff - self.swapTeams > 25:
            self.swapTeams = time_diff
            self.move_next()
            self.sensorTower.recalculate()

        if time_diff - self.calculateGridTime > .1:
            self.hitGrid()
            self.calculateGridTime = time_diff

        self.move_by_potential_field()
        results = self.commandCenter.do_commands(self.commands)

    def shoot(self):
        command = Command(self.me.index, 0, 0,True)
        self.commands.append(command)

    def move_by_potential_field(self):
        flag = self.me.flag != "-" and self.me.flag != self.constants["team"]
        deltaX,deltaY = self.sensorTower.calculate_potential_field_value(self.me.x,self.me.y,flag)
        velocity = math.sqrt(math.pow(deltaX,2) + math.pow(deltaY,2))
        if velocity > 1:
            velocity = 1
        elif velocity < -1:
            velocity = -1
        angle = math.atan2(deltaY, deltaX)
        angle = self.normalize_angle(angle - self.me.angle)
        if math.fabs(angle) < .4:
            angle = angle/2
        if math.fabs(angle) < .1:
            angle = 0
        command = Command(self.me.index, 1, angle, False)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def hitGrid(self):
        if self.me.status == 'dead':
            return
        occGridPos, occGrid = self.commandCenter.get_occgrid(self.myIndex)
        for i in range(0,len(occGrid)):
            for j in range(0,len(occGrid[0])):
                self.commandCenter.grid.hitSquare(occGrid[i][j] == 1, occGridPos[0] + i + 400,occGridPos[1] + j + 400)
        self.commandCenter.gridPlotter.update_grid(self.commandCenter.grid.filterGrid)
        self.commandCenter.gridPlotter.draw_grid()

class CommandCenter(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc,true,false, gridPlotter):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.bases = self.bzrc.get_bases()

        self.foodSupply = 1
        self.army = []
        self.prev_time = 0
        self.lock = threading.Lock()
        self.mytanks,self.othertanks,self.flags,self.shots = self.bzrc.get_lots_o_stuff()
        
        self.grid = Grid(self.constants["worldsize"],true,false)
        self.gridPlotter = gridPlotter

        print "Command Center Beginning Attack"
        print "Target firing on Location -350, -350"
        print "Deploying " + str(min(self.foodSupply,len(self.mytanks))) + " marines"
        for i in range(0,min(self.foodSupply,len(self.mytanks))):
            marine = Marine(self.mytanks[i],i,self,self.constants)
            marine.start()
            self.army.append(marine)
        self.prev_time1 = time.time()

    def get_occgrid(self,index):
        return self.use_bzrc('get_occgrid',index)

    def get_marine(self, index):
        return self.mytanks[index]

    def get_teams(self):
        return self.teams

    def tick(self):
        """Some time has passed; decide what to do next."""
        self.time_diff = time.time() - self.prev_time1
        if self.prev_time == 0:
            self.prev_time = self.time_diff
        if self.time_diff - self.prev_time > .01:
            self.prev_time = self.time_diff
            self.mytanks,self.othertanks,self.flags,self.shots = self.use_bzrc('get_lots_o_stuff',None)
    
    def use_bzrc(self,command,commands):
        self.lock.acquire()
        try:
            if command == 'get_lots_o_stuff':
                return self.bzrc.get_lots_o_stuff()
            elif command == 'do_commands':
                try:
                    if not commands == None and len(commands) > 0:
                        results = self.bzrc.do_commands(commands)
                except UnexpectedResponse:
                    print "error"
            elif command == 'get_occgrid':
                return self.bzrc.get_occgrid(commands)
        finally:
            self.lock.release()

    def do_commands(self,commands):
        # self.commands.append(commands)
        # if len(self.commands) > 10:
        self.use_bzrc('do_commands',commands)

    def get_team_index(self):
        return self.index
    def get_bases(self):
        return self.bases

    def get_obstacles(self):
        return self.obstacles

    def get_lots_o_stuff(self):
        return self.mytanks,self.othertanks,self.flags, self.shots

class SensorTower(object):
    """Class handles all potential field logic for an agent"""
    def __init__(self, constants, commandCenter):
        self.constants = constants
        self.commandCenter = commandCenter
        self.bases = self.commandCenter.get_bases()
        self.static_repulsive_field = []
        self.plotter = PotentialFieldPlotter()
        self.points = [(-350,-350),(-350,350),(350,350),(350,-350),(0,-350),(0,350),(0,0),(350,0),(-350,0),(-350,-350),(350,350),(-350,350),(350,-350)]
        self.index = 1
        self.initialize_fields()

    def initialize_fields(self):

        self.recalculate()

    def move_next(self):
        self.index+=1;
        if(self.index == len(self.points)):
            print "Completed"
            self.index = 0

    def recalculate(self):
        self.dynamic_repulsive_field = []
        self.enemy_flag = Location()
        self.enemy_flag.middle_x = self.points[self.index][0]
        self.enemy_flag.middle_y = self.points[self.index][1]
        self.enemy_flag.radius = 1
        self.enemy_flag.size = 10
        self.enemy_flag.weight = 1000


    def calculate_potential_field_value(self, x, y, flag):
        sumDeltaX = 0
        sumDeltaY = 0
        attractive_field = []
        attractive_field.append(self.enemy_flag)
        for item in attractive_field:
            goalX = item.middle_x
            goalY = item.middle_y
            goalRadius = item.radius
            goalSize = item.size
            goalWeight = item.weight
            distance = math.sqrt(math.pow((goalX - x),2) + math.pow((goalY - y),2))
            angle = math.atan2(goalY-y, goalX-x)
            deltaX,deltaY = self.positive_potential_field_values(distance,goalRadius, angle, goalSize, goalWeight)
            sumDeltaX += deltaX
            sumDeltaY += deltaY
        return sumDeltaX, sumDeltaY

    def calculate_full_potential_field(self):
        arr = []
        for i in range(-int(self.constants["worldsize"])/2, int(self.constants["worldsize"])/2):
            x = []
            for j in range(-int(self.constants["worldsize"])/2, int(self.constants["worldsize"])/2):
                deltaX,deltaY,angle = self.calculate_potential_field_value(i,j,False)
                x.append(PotentialFieldValue(deltaX, deltaY, angle))
            arr.append(x)
        return arr

    def positive_potential_field_values(self,distance,radius,angle,size,weight):
        if distance < radius:
            deltaX = 0
            deltaY = 0
        elif distance >= radius and distance <= size + radius:
            deltaX = weight*(distance - radius)*math.cos(angle)
            deltaY = weight*(distance - radius)*math.sin(angle)
        else:
            deltaX = weight*(size)*math.cos(angle)
            deltaY = weight*(size)*math.sin(angle)
        return deltaX, deltaY

    def negative_potential_field_values(self,distance,radius,angle,size,weight):
        if distance < radius:
            deltaX = -float(1e3000)
            deltaY = -float(1e3000)
        elif distance >= radius and distance <= size + radius:
            deltaX = -weight*(distance - radius + size)*math.cos(angle)
            deltaY = -weight*(distance - radius + size)*math.sin(angle)
        else:
            deltaX = 0
            deltaY = 0
        return deltaX, deltaY

    def negative_tangential_field_values(self,distance,radius,angle,size,weight):
        angle = angle + math.radians(90)
        if distance < radius:
            deltaX = -float(1e3000)
            deltaY = -float(1e3000)
        elif distance >= radius and distance <= size + radius:
            deltaX = -weight*(distance - radius + size)*math.cos(angle)
            deltaY = -weight*(distance - radius + size)*math.sin(angle)
        else:
            deltaX = 0
            deltaY = 0
        return deltaX, deltaY

    def visualize_potential_field(self):
        print "Visualize it"
        self.plotter.plot(self.calculate_potential_field_value,self.obstacles)

class PotentialFieldValue(object):
    def __init__(self,deltaX,deltaY,angle):
        self.deltaX = deltaX
        self.deltaY = deltaY
        self.angle = angle
    def __str__(self):
        return "(" + str(self.deltaX) + "," + str(self.deltaY) + "," + str(self.angle) + ")"

    def __repr__(self):
        return "(" + str(self.deltaX) + "," + str(self.deltaY) + "," + str(self.angle) + ")"

class Grid:
    """Class handles storing and manipulating the occupancy grid"""

    def __init__(self, worldsize,true,false):
        self.grid = []
        self.true = true
        self.false = false
        self.filterGrid = zeros((int(worldsize),int(worldsize)))

        self.filterGrid.fill(.15)
        for i in range(0,int(worldsize)):
            self.grid.append([])
            for j in range(0,int(worldsize)):
                self.grid[i].append(OccupancySquare(self.true,self.false))
        print self.grid[0][0].true,self.grid[0][0].truefalse,self.grid[0][0].false,self.grid[0][0].falsetrue

    def hitSquare(self, isOccupied, x, y):
        if len(self.grid) != 0:
            #print str(x) + " " + str(y) + " " +str(isOccupied)
            self.grid[y][x].hitSquare(isOccupied)
            self.filterGrid[y][x] = self.grid[y][x].probabilityOfOccupied
            # if self.grid[y][x].probabilityOfOccupied > .7:
            #     self.filterGrid[y][x] =  1
            # else:
            #     self.filterGrid[y][x] = 0

class OccupancySquare:
    """Class handles a single square in the occupancy grid"""

    def __init__(self,true,false):
        self.true = true
        self.false = false
        self.truefalse = 1 - true
        self.falsetrue = 1 - false
        """Start with it having a probability of 0, then as hits happen,
            calculate the new probability"""
        self.probabilityOfOccupied = 0.15

    def hitSquare(self, isOccupied):
        if isOccupied:
            self.probabilityOfOccupied = (self.true * self.probabilityOfOccupied) / ((self.true * self.probabilityOfOccupied) + (self.truefalse * (1 - self.probabilityOfOccupied)))
        else:
            self.probabilityOfOccupied = (self.truefalse * self.probabilityOfOccupied) / ((self.truefalse * self.probabilityOfOccupied) + (self.false * (1 - self.probabilityOfOccupied)))

def main():
    # Process CLI arguments.
    try:
        execname, host, port, true, false = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    gridPlotter = GridFilter()
    cc = CommandCenter(bzrc, float(true), float(false),gridPlotter)
    gridPlotter.init_window(800, 800,cc.tick)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            action = threading.Thread(target =cc.tick, args=[time_diff])
            action.start()
            action.join()
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()



if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
