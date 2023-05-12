import pygame

from Track import LoopTrack
from PointsToOsmConv import PointsToOsmConverter
from pygame.locals import *


class TrackGeneratorApp:
    def __init__(self):
        pygame.init()

        self.generatorRunning = True
        
        self.track = LoopTrack()
        self.track_img = pygame.image.load("imola (1).pgm")
        self.screen = pygame.display.set_mode(self.track_img.get_size(), HWSURFACE | DOUBLEBUF | RESIZABLE)
        pygame.display.set_caption("Track Generator")
        
    def run(self):
        
        a = PointsToOsmConverter([],[])
        a.convert()
        
        while self.generatorRunning:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.generatorRunning = False
                    return
                self.handleEvent(event)
                
            self.screen.blit(self.track_img, (0, 0))
            self.drawTrack()
            pygame.display.flip()
            
            
    def drawTrack(self):
        self.track.draw(self.screen)
    
    def handleEvent(self, event):
        self.track.update(event)
        