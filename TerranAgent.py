import sys
import math
import time
from random import randrange
from fields import PotentialFieldPlotter
import threading
from bzrc import BZRC, Command, UnexpectedResponse

class Marine(threading.Thread):
    def __init__(self,marine, index, commandCenter,constants, mainTeam):
        threading.Thread.__init__(self)
        self.me = marine
        self.myIndex = index
        self.commandCenter = commandCenter
        self.constants = constants
        self.mainTeam = mainTeam
        self.sensorTower = SensorTower(constants, commandCenter, mainTeam.color)
        self.commands = []
        self.swapTeams = 0
        self.shootTime = 0
        self.recalculateTime = 0
        self.moveTime = 0
        #self.sensorTower.visualize_potential_field()
        self.frozenTime = 0
        self.new = True
        print "Marine " + str(index) + " ready to go!"

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
        self.commands = []
        oldme = self.me
        self.me = self.commandCenter.get_marine(self.myIndex)
        if self.me.flag != '-' and self.new:
            self.new = False
            print "Marine " + str(self.myIndex) + " has acquired the " + str(self.me.flag) + " teams flag!"
        elif self.me.flag == "-":
            self.new = True
        if self.me.x == oldme.x and self.me.y == oldme.y:
            self.frozenTime+=1
        else:
            self.frozenTime = 0
        if self.frozenTime > 100:
            print "Marine " + str(self.myIndex) + " is stuck... moving to better location!"
            self.frozenTime -=1
            command = Command(self.me.index, 1, math.radians(randrange(0,180)), False)
            self.commandCenter.do_commands([command])
        if time_diff - self.shootTime > 1:
            self.shoot()
            self.shootTime = time_diff
        if time_diff - self.recalculateTime > .10:
            self.sensorTower.recalculate(self.mainTeam.color)
            self.recalculateTime = time_diff

        if time_diff - self.swapTeams > 1000:
            teams = self.commandCenter.get_teams()
            oldTeam = self.mainTeam
            self.mainTeam = randrange(0,len(teams))
            while self.mainTeam == self.commandCenter.get_team_index() and not oldTeam.color == teams[self.mainTeam].color:
                self.mainTeam = randrange(0,len(teams))
            self.mainTeam = teams[self.mainTeam]
            self.swapTeams = time_diff
            print "Marine " + str(self.myindex) + " switching to target: " + str(self.mainTeam.color)
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

class CommandCenter(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.bases = self.bzrc.get_bases()
        self.obstacles = self.bzrc.get_obstacles()
        self.teams = self.bzrc.get_teams()
        for ind,team in enumerate(self.teams):
            if team.color == self.constants["team"]:
                self.index = ind
                break
        mainTeam = randrange(0,len(self.teams))
        while mainTeam == self.index:
            mainTeam = randrange(0,len(self.teams))

        mainTeam = self.teams[mainTeam]
        self.foodSupply = 10
        self.army = []
        self.prev_time = 0
        self.lock = threading.Lock()
        self.mytanks,self.othertanks,self.flags,self.shots = self.bzrc.get_lots_o_stuff()
        print "Command Center Beginning Attack"
        print "Target firing on: " + str(mainTeam.color)
        print "Deploying " + str(min(self.foodSupply,len(self.mytanks))) + " marines"
        for i in range(0,min(self.foodSupply,len(self.mytanks))):
            marine = Marine(self.mytanks[i],i,self,self.constants,mainTeam)
            marine.start()
            self.army.append(marine);

    def get_marine(self, index):
        return self.mytanks[index]

    def get_teams(self):
        return self.teams

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        if self.prev_time == 0:
            self.prev_time = time_diff
        if time_diff - self.prev_time > .01:
            self.prev_time = time_diff
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
        return self.mytanks,self.othertanks,self.flags, self.shots;

class SensorTower(object):
    """Class handles all potential field logic for an agent"""
    def __init__(self, constants, commandCenter, choosenTeam):
        self.constants = constants
        self.commandCenter = commandCenter
        self.bases = self.commandCenter.get_bases()
        self.obstacles = self.commandCenter.get_obstacles()
        self.static_repulsive_field = []
        self.initialize_fields(choosenTeam)
        self.plotter = PotentialFieldPlotter()

    def initialize_fields(self, team):
        for base in self.bases:
            if base.color == self.constants["team"]:
                base.weight = 1000
                self.our_base = base
        for obstacle in self.obstacles:
            self.static_repulsive_field.append(obstacle)

        self.recalculate(team)

    def recalculate(self, choosenTeam):
        mytanks, othertanks, flags, shots = self.commandCenter.get_lots_o_stuff()
        self.dynamic_repulsive_field = []
        for flag in flags:
            if flag.color == choosenTeam:
                flag.weight = 1000
                self.enemy_flag= flag
        for enemy in othertanks:
            self.dynamic_repulsive_field.append(enemy)

        for shot in shots:
            self.dynamic_repulsive_field.append(shot)

    def calculate_potential_field_value(self, x, y, flag):
        sumDeltaX = 0
        sumDeltaY = 0
        attractive_field = []
        if flag == True:
            attractive_field.append(self.our_base)
        else:
            attractive_field.append(self.enemy_flag)

        #attractive field
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
        #repulsive fields + tangential
        repulsive_field = self.static_repulsive_field + self.dynamic_repulsive_field
        for item in repulsive_field:
            goalX = item.middle_x
            goalY = item.middle_y
            goalRadius = item.radius
            goalSize = item.size
            goalWeight = item.weight
            distance = math.sqrt(math.pow((goalX - x),2) + math.pow((goalY - y),2))
            angle = math.atan2(goalY-y, goalX-x)
            if item.tangential == False:
                deltaX,deltaY = self.negative_potential_field_values(distance,goalRadius, angle, goalSize, goalWeight)
            else:
                deltaX,deltaY = self.negative_tangential_field_values(distance,goalRadius, angle, goalSize, goalWeight)
            sumDeltaX += deltaX
            sumDeltaY += deltaY
        #25% change to add a random field
        if randrange(0,100) > 75:
            randomX = randrange(0,100)
            randomY = randrange(0,100)
            sumDeltaY += randomY
            sumDeltaX += randomX
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
            deltaY = 0;
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
            deltaY = 0;
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
            deltaY = 0;
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


def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    cc = CommandCenter(bzrc)

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
