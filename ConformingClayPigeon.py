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
import random

from bzrc import BZRC, Command

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc,type):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.moveTime = randrange(3,8)
        self.time = 0
        self.type = type
        self.prevSpeed = 1
        self.brakeCountDown = 0

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        time_passed = time_diff - self.time
        self.time = time_diff
        self.moveTime -= time_passed
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]
        self.commands = []
        if self.type == 'Duck' or self.type == 'duck': #The first tank is a "sitting duck" and doesn't move
            return
        if self.type == 'Constant' or self.type == 'constant':
            for tank in mytanks:
                if tank.index != 0:
                    break
                if self.moveTime > 0:
                    self.move_forward(tank)
                    self.angleTime = 0
        if self.type == 'Wild' or self.type == 'wild':
            for tank in mytanks:
                if tank.index != 0:
                    break
                if self.moveTime > 0:
                    if self.prevSpeed < .2:
                        self.brakeCountDown -= 1
                    if self.brakeCountDown < 1:
                        randSpeed = random.random() # makes a random number between 0 and 1.0
                        if randSpeed < .2:
                            self.brakeCountDown = 3
                            self.prevSpeed = randSpeed
                        else:
                            self.prevSpeed = 1
                            # this will make the tank go full speed most of the time, 
                            # then randomly hit the brakes
                    randDegree = randrange(-60, 60)
                    # these random numbers will make it hard for the agent to predict the movements
                    self.move_rand_degrees(tank, self.prevSpeed, randDegree)
                    # self.move_forward(tank)
                    self.angleTime = 0

        if self.moveTime < 0 and (time_diff - self.angleTime > 2):
            self.moveTime = randrange(3,8)

        results = self.bzrc.do_commands(self.commands)

    def move_sixty_degrees(self, tank):
        command = Command(tank.index, 1, 60, False)
        self.commands.append(command)

    def move_rand_degrees(self, tank, rspeed, degree):
        print rspeed
        command = Command(tank.index, rspeed, degree, False)
        self.commands.append(command)

    def move_forward(self,tank):
        command = Command(tank.index, 1, 0, False)
        self.commands.append(command)

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle


def main():
    # Process CLI arguments.
    try:
        execname, host, port, pidgeonType = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port type' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = Agent(bzrc,pidgeonType)

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
