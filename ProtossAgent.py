import sys
import math
import time
from numpy import *
from random import randrange
from fields import PotentialFieldPlotter
from grid_filter_gl import GridFilter
import threading
from bzrc import BZRC, Command, UnexpectedResponse, Location

class Zealot(threading.Thread):
    def __init__(self,zealot, index, commandCenter,constants, count):
        threading.Thread.__init__(self)
        self.me = zealot
        self.myIndex = index
        self.commandCenter = commandCenter
        self.constants = constants
        self.commands = []
        self.shootTime = 0
        self.calculateEnemyLocation = 0
        self.new = True
        print "Zealot " + str(index) + " ready to go!"

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
        self.me = self.commandCenter.get_zealot(self.myIndex)

        if time_diff - self.shootTime > 1:
            #see if enemy is within range, then predict location for a given time when 
            #trajetories would match
            self.shoot()
            self.shootTime = time_diff

        if time_diff - self.calculateEnemyLocation > 500:
            #every deltaT time (.5 seconds) calculate the new location of enemies
            #within a given range
            self.calc_enemy_location()
            self.calculateEnemyLocation = time_diff

        results = self.commandCenter.do_commands(self.commands)

    def calc_enemy_location(self):
        self.enemiesFilters = {};

    def shoot(self):
        command = Command(self.me.index, 0, 0,True)
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

    def __init__(self, bzrc, psi):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.bases = self.bzrc.get_bases()

        self.psi = int(psi)
        self.army = []
        self.prev_time = 0
        self.lock = threading.Lock()
        self.mytanks,self.othertanks,self.flags,self.shots = self.bzrc.get_lots_o_stuff()

        print "Nexus Beginning Warp Ins"
        print "Target firing on Location -350, -350"
        print "Deploying " + str(min(self.psi,len(self.mytanks))) + " zealots"
        for i in range(0,min(self.psi,len(self.mytanks))):
            zealot = Zealot(self.mytanks[i],i,self,self.constants,self.psi)
            zealot.start()
            self.army.append(zealot)
        self.prev_time1 = time.time()

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

# this filter is a per tank thing, when we get the enemy tanks match their filter
# by index in a dictionary?
# deltaT is the time we are letting pass between calculations
# noise is a parameter we pass in from command line (sever default is 5)
# startX and startY is the original position of the tanks flag
class KalmanFilter(object):
    def __init__(self,deltaT,noise,startX,startY):
        self.deltaT = deltaT;
        self.Et = numpy.matrix([100,0,0,0,0,0],
            [0,.1,0,0,0,0],
            [0,0,.1,0,0,0],
            [0,0,0,100,0,0],
            [0,0,0,0,.1,0],
            [0,0,0,0,0,.1]);
        #choose 2 because 100 seemed like too large
        self.Ex = numpy.matrix([.1,0,0,0,0,0],
            [0,.1,0,0,0,0],
            [0,0,2,0,0,0],
            [0,0,0,.1,0,0],
            [0,0,0,0,.1,0],
            [0,0,0,0,0,2])
        self.Ez = numpy.matrix([pow(noise,2),0],[0,pow(noise,2)])
        self.F = numpy.matrix([1,deltaT,pow(deltaT,2)/2,0,0,0],
            [0,1,deltaT,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,deltaT,pow(deltaT,2)/2],
            [0,0,0,0,1,deltaT],
            [0,0,0,0,0,1]);
        self.H = numpy.matrix([1,0,0,0,0,0],[0,0,0,1,0,0])
        self.Ut = np.matrix([startX,0,0,startY,0,0])
        self.reset = time.time()

    # this is called at each time step deltaT to recalculate locations based
    # on observed position x and y
    # because of mentioned error Et is reset every 10 seconds
    def calc_location(self,tankId, x, y):
        if self.reset - time.time() > 10000:
            self.reset = time.time()
            self.Et = numpy.matrix([x,0,0,0,0,0],
            [0,.1,0,0,0,0],
            [0,0,.1,0,0,0],
            [0,0,0,y,0,0],
            [0,0,0,0,.1,0],
            [0,0,0,0,0,.1]); 
        Zt = np.matrix([x,y]);
        FT = np.transpose(F);
        Ex = self.Ex;
        HT = np.transpose(H);
        Et = self.Et;
        Ez = self.Ez;
        H = self.H;
        F = self.F;
        Ut = self.Ut;
        I = np.matrix([1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1])
        Kt = ((F*Et)*FT + Ex)*HT*np.linalg.inv((H*(F*Et*FT + Ex)*HT + Ez));
        self.Ut = F*Ut + Kt*(Zt - H*F*Ut);
        self.Et = (I - Kt*H)*(F*Et*FT + Ex);

    # this is used to predict the location of a tank a given time in the future
    def predict_location(self,time):
        F = numpy.matrix([1,time,pow(time,2)/2,0,0,0],
            [0,1,time,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,time,pow(time,2)/2],
            [0,0,0,0,1,time],
            [0,0,0,0,0,1]);
        return self.H*(F*self.Ut);


def main():
    # Process CLI arguments.
    try:
        #noise needs to be passed into the necessary functions
        execname, host, port, psi, noise = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))
    cc = Nexus(bzrc, psi)

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
