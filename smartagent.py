#!/usr/bin/python -tt

# An incredibly simple agent.  All we do is find the closest enemy tank, drive
# towards it, and shoot.  Note that if friendly fire is allowed, you will very
# often kill your own tanks with this code.

#################################################################
# NOTE TO STUDENTS
# This is a starting point for you.  You will need to greatly
# modify this code if you want to do anything useful.  But this
# should help you to know how to interact with BZRC in order to
# get the information you need.
#
# After starting the bzrflag server, this is one way to start
# this code:
# python agent0.py [hostname] [port]
#
# Often this translates to something like the following (with the
# port name being printed out by the bzrflag server):
# python agent0.py localhost 49857
#################################################################

import sys
import math
import time
from random import randrange
from fields import PotentialFieldPlotter

from bzrc import BZRC, Command

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        teams = self.bzrc.get_teams()
        for ind,team in enumerate(teams):
            if team.color == self.constants["team"]:
                self.index = ind
        self.mainTeam = randrange(0,len(teams))
        while self.mainTeam == self.index:
            self.mainTeam = randrange(0,len(teams))

        self.potentialField = PotentialField(self.constants, self.bzrc, teams[self.mainTeam].color)

        self.swapTeams = 0
        self.potentialField.visualize_potential_field()
        self.commands = []
        self.shootTime = 0
        self.recalculateTime = 0
        self.moveTime = 0

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        if self.shootTime ==0 and self.recalculateTime == 0 and self.swapTeams == 0:
            self.shootTime = time_diff
            self.recalculateTime = time_diff
            self.swapTeams = time_diff
        self.commands = []
        self.mytanks = self.bzrc.get_mytanks()
        self.teams = self.bzrc.get_teams()
        if time_diff - self.shootTime > 1:
            self.shoot()
            self.shootTime = time_diff
        if time_diff - self.recalculateTime > .10:
            self.potentialField.recalculate(self.teams[self.mainTeam].color)
            self.recalculateTime = time_diff

        if time_diff - self.swapTeams > 1000:
            print "swapping teams"
            self.mainTeam = randrange(0,len(self.teams))
            while self.mainTeam == self.index:
                self.mainTeam = randrange(0,len(self.teams))
            self.swapTeams = time_diff

        self.move_by_potential_field()
        try:
            results = self.bzrc.do_commands(self.commands)
        except UnexpectedResponse:
            print "error"

    def shoot(self):
        for tank in self.mytanks:
            command = Command(tank.index, 0, 0,True)
            self.commands.append(command)

    def move_by_potential_field(self):
        for tank in self.mytanks:
            flag = tank.flag != "-" and tank.flag != self.constants["team"]
            deltaX,deltaY = self.potentialField.calculate_potential_field_value(tank.x,tank.y,flag)
            velocity = math.sqrt(math.pow(deltaX,2) + math.pow(deltaY,2))
            if velocity > 1:
                velocity = 1
            elif velocity < -1:
                velocity = -1
            angle = math.atan2(deltaY, deltaX)
            angle = self.normalize_angle(angle - tank.angle)
            if math.fabs(angle) < .4:
                angle = angle/2
            if math.fabs(angle) < .1:
                angle = 0
            command = Command(tank.index, 1, angle, False)
            self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

class PotentialField(object):
    """Class handles all potential field logic for an agent"""
    def __init__(self, constants, bzrc, choosenTeam):
        self.constants = constants
        self.bzrc = bzrc
        self.bases = self.bzrc.get_bases()
        self.obstacles = self.bzrc.get_obstacles()
        self.attractive_field = []
        self.repulsive_field = []
        self.dynamic_attractive_field = []
        self.dynamic_repulsive_field = []
        self.flag_attractive_field = []
        self.initialize_fields(choosenTeam)
        self.plotter = PotentialFieldPlotter()

    def initialize_fields(self, team):
        for base in self.bases:
            if base.color != self.constants["team"]:
                self.attractive_field.append(base)
            elif base.color == self.constants["team"]:
                self.repulsive_field.append(base)
                self.flag_attractive_field.append(base)
        for obstacle in self.obstacles:
            self.repulsive_field.append(obstacle)

        self.recalculate(team)

    def recalculate(self, choosenTeam):
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.dynamic_attractive_field = []
        self.dynamic_repulsive_field = []
        for flag in flags:
            if flag.color == choosenTeam:
                flag.weight = 1000
            if flag.color != self.constants["team"]:
                self.attractive_field.append(flag)
        for enemy in othertanks:
            self.dynamic_repulsive_field.append(enemy)
        for other in mytanks:
            self.dynamic_repulsive_field.append(other)
        for shot in shots:
            self.dynamic_repulsive_field.append(shot)

    def calculate_potential_field_value(self, x, y, flag):
        sumDeltaX = 0
        sumDeltaY = 0
        if flag == True:
            attractive_field = self.flag_attractive_field
        else:
            attractive_field = self.attractive_field
        attractive_field += self.dynamic_attractive_field

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
        repulsive_field = self.repulsive_field + self.dynamic_repulsive_field
        for item in self.repulsive_field:
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

    agent = Agent(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
