import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import math

class DemoVisualizer:
    def __init__(self):
        self.video_flags = OPENGL | DOUBLEBUF
        pygame.init()
        self.screen = pygame.display.set_mode((960, 540), self.video_flags)
        pygame.display.set_caption("SIMCLEVER ")
        
        glViewport(0, 0, 960, 540)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0*960/540, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        
        self.frames = 0
        self.ticks = pygame.time.get_ticks() 

    def visualize_rpy(self, roll, pitch, yaw):
        self.event = pygame.event.poll()
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # 3D nesneyi dÃ¶ndÃ¼r
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)
        glRotatef(roll, 0.00, 0.00, 1.00)
        glRotatef(pitch, 1.00, 0.00, 0.00)
        glRotatef(-yaw, 0.00, 1.00, 0.00)

        # Cismi Ã§iz
        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

        # Sabit bir daire Ã§iz
        
        pygame.display.flip()
        
    def draw_ellipsoid(self, center_x=0.0, center_y=0.0, center_z=-20.0, radius_x=1.0, radius_y=1.0, radius_z=1.0, segments=10, point_x=0.0, point_y= 0.0, point_z=0.0):
        self.event = pygame.event.poll()
        """
        OpenGL kullanarak ekrana 3 boyutlu bir elipsoit Ã§izer.
        center_x, center_y, center_z: Elipsoidin merkezi
        radius_x, radius_y, radius_z: Elipsoidin x, y ve z eksenlerindeki yarÄ±Ã§aplarÄ±
        segments: Elipsoidin kaÃ§ parÃ§aya bÃ¶lÃ¼neceÄŸi (daha yÃ¼ksek deÄŸer pÃ¼rÃ¼zsÃ¼zlÃ¼k saÄŸlar)
        """
        glPushMatrix()       # Matris durumunu kaydet
        glLoadIdentity()     # DiÄŸer dÃ¶nÃ¼ÅŸÃ¼mlerden baÄŸÄ±msÄ±z hale getirmek iÃ§in matrisleri sÄ±fÄ±rla

        glPointSize(5)
        glTranslatef(center_x, center_y, center_z)  # Elipsoidin merkezini ayarla
        
        glBegin(GL_POINTS)
        glColor3f(0.0, 1.0, 1.0)  # Elipsoidi renklendir
        
        # Elipsoit Ã¼zerindeki noktalarÄ± oluÅŸtur
        glVertex2f(point_x, point_y)

        glEnd()

        glPopMatrix()  # Ã–nceki matris durumuna geri dÃ¶n


        pygame.display.flip()

# Ã–rnek kullanÄ±m:
# demo = DemoVisualizer()
# demo.visualize_rpy(0, 0, 0)  # Roll, pitch, yaw deÄŸerleri 0 olan gÃ¶rselleÅŸtirme