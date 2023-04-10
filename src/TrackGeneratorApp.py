import pygame

from Track import CircleTrack
from DragablePoint import DraggablePoint

class TrackGeneratorApp:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("Track Generator")
        self.clock = pygame.time.Clock()
        
        # Set up other game state variables, such as player position, score, etc.
        self.generatorRunning = True
        self.track = CircleTrack()
        
        
    def run(self):
        while self.generatorRunning:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.generatorRunning = False
                    return
                # elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handleEvent(event)
                    
            self.screen.fill((255, 255, 255))
            self.drawTrack()
            pygame.display.flip()
            
            
    def drawTrack(self):
        self.track.draw(self.screen)
    
    def handleEvent(self, event):
        self.track.update(event)
        pass