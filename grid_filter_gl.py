#!/usr/bin/env python

import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from numpy import zeros

class GridFilter:
    def __init__(self):
        self.grid = None

    def draw_grid(self):
        # This assumes you are using a numpy array for your grid
        self.width, self.height = self.grid.shape
        # for i in range(0,self.width):
        #     for j in range(0,self.height):
        #         if(self.grid[i][j] != .5):
        #             print self.grid[i][j]
        glRasterPos2f(-1, -1)
        glDrawPixels(self.width, self.height, GL_LUMINANCE, GL_FLOAT, self.grid)
        glFlush()
        glutSwapBuffers()

    def update_grid(self, new_grid):
        self.grid = new_grid
        #[[.5 for i in range(worldsize)]for i in range(worldsize)]
        glutPostRedisplay()

    def init_window(self, width, height,func):
        self.grid = zeros((width, height))
        self.grid.fill(.15)
        glutInit(())
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        glutInitWindowSize(width, height)
        glutInitWindowPosition(0, 0)
        self.window = glutCreateWindow("Grid filter")
        glutDisplayFunc(self.draw_grid)
        glutIdleFunc(func)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.draw_grid()
        glutMainLoop()



    # vim: et sw=4 sts=4
