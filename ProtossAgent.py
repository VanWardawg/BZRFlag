import sys
import math
import time
import matplotlib.pyplot as plt
from numpy import *
from random import randrange
from grid_filter_gl import GridFilter
from fields import PotentialFieldPlotter
import threading
from bzrc import BZRC, Command, UnexpectedResponse, Location

class Zealot(threading.Thread):
    def __init__(self,zealot, index, nexus,constants, count, noise):
        threading.Thread.__init__(self)
        self.me = zealot
        self.myIndex = index
        self.nexus = nexus
        self.constants = constants
        self.commands = []
        self.shootTime = 0
        self.calculateEnemyLocation = 0
        self.new = True
        self.enemiesFilters = {};
        self.noise = noise;
        self.deltaT = .5;
        self.printTime = 0;
        print "Zealot " + str(index) + " ready to go!"
        self.plotter = PotentialFieldPlotter()

    def run(self):
        prev_time = time.time()
        while True:
            time_diff = time.time() - prev_time
            self.tick(time_diff)

    def tick(self, time_diff):
        if self.shootTime ==0:
            self.shootTime = time_diff

        if self.calculateEnemyLocation == 0:
            self.calculateEnemyLocation = time_diff

        self.commands = []
        self.me = self.nexus.get_zealot(self.myIndex)

        if time_diff - self.shootTime > 1:
            self.shoot()
            self.shootTime = time_diff

        if time_diff - self.calculateEnemyLocation > (self.deltaT):
            #every deltaT time (.5 seconds) calculate the new location of enemies
            enemyTanks = self.nexus.get_enemies();
            enemyTank = enemyTanks[2]
            for i in range(0,len(enemyTanks)):
                self.calc_enemy_location(i,enemyTanks[i])
            self.calculateEnemyLocation = time_diff

            #see if enemy is within range, then predict location for a given time when 
            #trajetories would match
            curr_dist = math.sqrt(math.pow((enemyTank.x - self.me.x),2) + math.pow((enemyTank.y - self.me.y),2));
            bullet_speed = 0 + int(self.constants["shotspeed"]) 
            # todo: when the tank actually moves, the bullet_speed is increased by self.me.speed
            curr_time_till_shot_hits = curr_dist / bullet_speed

            #convert the current tank's angle to degrees because their angle value is as messed up as a zergling's mating rutual
            my_angle_in_degrees = 0
            if self.me.angle >= 0:
                my_angle_in_degrees = 180 * (self.me.angle / 3.14)
            else:
                my_angle_in_degrees = 180 + (180 - (180 * (-1 * self.me.angle) / 3.14))

            future_enemy_location = self.get_future_enemy_location(enemyTank, curr_time_till_shot_hits)

            #get the distance and direction of the enemy tank
            distance = [future_enemy_location[0] - self.me.x, future_enemy_location[1] - self.me.y]
            norm = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
            direction = [distance[0] / norm, distance[1] / norm]

            goal_angle_in_degrees = math.degrees(math.atan2(distance[1], distance[0]))
            if goal_angle_in_degrees < 0:
                goal_angle_in_degrees = (180 - (-1) * goal_angle_in_degrees) + 180
            angle_diff = -1 * (my_angle_in_degrees - goal_angle_in_degrees)
            relative_angle = 2 * self.normalize_angle(angle_diff)
            # if angle_diff > 180:
            #     angle_diff -= 180
            # if angle_diff < -180:
            #     angle_diff += 180

            # if(angle_diff < 0 and angle_diff > -180):


            self.rotate_to_enemy(angle_diff / 10)

            # print 'dist ' + str(distance)
            print 'ang ' + str(my_angle_in_degrees) + ' ' + str(goal_angle_in_degrees)
            print 'goal diff: ' + str(angle_diff)
            print 'relative diff ' + str(relative_angle)
            print ''

        results = self.nexus.do_commands(self.commands)

    def calc_enemy_location(self,index,enemy):
        if not index in self.enemiesFilters.keys():
            enemyFlag = self.nexus.get_enemy_flag(enemy.color);
            self.enemiesFilters[index] = KalmanFilter(self.deltaT,self.noise,enemyFlag.x,enemyFlag.y)
        enemyFuture = self.enemiesFilters[index].predict_location(self.deltaT)
        self.enemiesFilters[index].calc_location(enemy.x,enemy.y);
        self.printTime +=1
        enemyLoc = self.enemiesFilters[index].H*self.enemiesFilters[index].Ut;
        # if self.printTime %10 == 0:
        # if self.printTime >=10 and self.printTime < 20 and index == 0:
        #     print "Enemy " + str(index) + " at: " + str(enemyLoc) + "\npredicted at: " + str(enemyFuture) + "\nobserved at: " + str(str(enemy.x) + "," + str(enemy.y));
        #     Et = self.enemiesFilters[index].Et;
        #     x = math.sqrt(Et[0,0]);
        #     y = math.sqrt(Et[3,3]);
        #     rho = Et[0,3]/(x*y);
        #     self.plotter.kalman_plot('wild',self.printTime,x,y,rho)
        self.nexus.grid.hitSquare(int(enemyLoc[0][0])+400,int(enemyLoc[1])+400);
        self.nexus.gridPlotter.update_grid(self.nexus.grid.filterGrid)
        self.nexus.gridPlotter.draw_grid()

    def get_future_enemy_location(self, enemy, curr_time_till_shot_hits):
        enemyFlag = self.nexus.get_enemy_flag(enemy.color);
        enemy_filter = KalmanFilter(self.deltaT,self.noise,enemyFlag.x,enemyFlag.y)
        return enemy_filter.predict_location(curr_time_till_shot_hits)

    def shoot(self):
        command = Command(self.me.index, 0, 0,True)
        self.commands.append(command)

    def rotate_to_enemy(self, angle):
        command = Command(self.me.index, 0, angle, False)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

class Nexus(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc, psi, noise,gridPlotter):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.bases = self.bzrc.get_bases()
        self.grid = Grid(self.constants["worldsize"])
        self.gridPlotter = gridPlotter;
        self.psi = psi
        self.army = []
        self.prev_time = 0
        self.lock = threading.Lock()
        self.mytanks,self.othertanks,self.flags,self.shots = self.bzrc.get_lots_o_stuff()

        print "Nexus Beginning Warp Ins"
        print "Target firing on Location -350, -350"
        print "Deploying " + str(min(self.psi,len(self.mytanks))) + " zealots"
        for i in range(0,min(self.psi,len(self.mytanks))):
            zealot = Zealot(self.mytanks[i],i,self,self.constants,self.psi,noise)
            zealot.start()
            self.army.append(zealot)
        self.prev_time1 = time.time()

    def get_enemies(self):
        return self.othertanks;

    def get_enemy_flag(self,color):
        for flag in self.flags:
            if flag.color == color:
                return flag;

    def get_occgrid(self,index):
        return self.use_bzrc('get_occgrid',index)

    def get_zealot(self, index):
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
            elif command == 'get_onexusgrid':
                return self.bzrc.get_onexusgrid(commands)
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

class Grid:
    """Class handles storing and manipulating the occupancy grid"""

    def __init__(self, worldsize):
        self.filterGrid = zeros((int(worldsize),int(worldsize)))
        self.filterGrid.fill(.15)

    def hitSquare(self, x, y):
        if x >= 800:
            x = 799;
        if y >= 800:
            y = 799;
        if x < 0:
            x = 0;
        if y < 0:
            y = 0;
        self.filterGrid[y][x] = 10;

# this filter is a per tank thing, when we get the enemy tanks match their filter
# by index in a dictionary?
# deltaT is the time we are letting pass between calculations
# noise is a parameter we pass in from command line (sever default is 5)
# startX and startY is the original position of the tanks flag
class KalmanFilter(object):
    def __init__(self,deltaT,noise,startX,startY):
        self.deltaT = deltaT;
        self.Et = matrix([[100,0,0,0,0,0],
            [0,.1,0,0,0,0],
            [0,0,.1,0,0,0],
            [0,0,0,100,0,0],
            [0,0,0,0,.1,0],
            [0,0,0,0,0,.1]]);
        #choose 2 because 100 seemed like too large -- this is the number to tweak in 
        # refining our filter
        self.Ex = matrix([[.1,0,0,0,0,0],
            [0,.1,0,0,0,0],
            [0,0,10,0,0,0],
            [0,0,0,.1,0,0],
            [0,0,0,0,.1,0],
            [0,0,0,0,0,2]])
        self.Ez = matrix([[pow(noise,2),0],[0,pow(noise,2)]])
        self.F = matrix([[1,deltaT,pow(deltaT,2)/2,0,0,0],
            [0,1,deltaT,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,deltaT,pow(deltaT,2)/2],
            [0,0,0,0,1,deltaT],
            [0,0,0,0,0,1]]);
        self.H = matrix([[1,0,0,0,0,0],[0,0,0,1,0,0]])
        self.Ut = transpose(matrix([startX,0,0,startY,0,0]))
        self.I = matrix([[1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]]);
        self.reset = time.time()
        self.HT = transpose(self.H);
        self.FT = transpose(self.F);

    # this is called at each time step deltaT to recalculate locations based
    # on observed position x and y
    # because of mentioned error Et is reset every 100 seconds
    def calc_location(self, x, y):
        if time.time() - self.reset > 100:
            self.reset = time.time()
            self.Et = matrix([[x,0,0,0,0,0],
            [0,.1,0,0,0,0],
            [0,0,.1,0,0,0],
            [0,0,0,y,0,0],
            [0,0,0,0,.1,0],
            [0,0,0,0,0,.1]]); 
            # print "Resetting"
        Zt = transpose(matrix([x,y]));
        FT = self.FT;
        Ex = self.Ex;
        HT = self.HT;
        Et = self.Et;
        Ez = self.Ez;
        H = self.H;
        F = self.F;
        Ut = self.Ut;
        I = self.I;
        Extra = ((F*Et)*FT + Ex);
        Kt = Extra*HT*linalg.inv((H*Extra*HT + Ez));
        self.Ut = F*Ut + Kt*(Zt - H*F*Ut);
        self.Et = (I - Kt*H)*Extra;

    # this is used to predict the location of a tank a given time in the future
    def predict_location(self,time):
        F = matrix([[1,time,pow(time,2)/2,0,0,0],
            [0,1,time,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,time,pow(time,2)/2],
            [0,0,0,0,1,time],
            [0,0,0,0,0,1]]);
        return self.H*(F*self.Ut);


def main():
    # Process CLI arguments.
    try:
        #noise needs to be passed into the necessary functions
        execname, host, port, psi, noise = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port psi noise' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    gridPlotter = GridFilter()
    bzrc = BZRC(host, int(port))
    nexus = Nexus(bzrc, int(psi), int(noise),gridPlotter)
    gridPlotter.init_window(800, 800,nexus.tick)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            action = threading.Thread(target =nexus.tick, args=[time_diff])
            action.start()
            action.join()
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()



if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
