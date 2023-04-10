import pygame
from DragablePoint import DraggablePoint

class CircleTrack:
    def __init__(self):
        self.inner_circle = []
        self.outer_circle = []
    
    def add_inner_point(self, point):
        self.inner_circle.append(point)
    
    def add_outer_point(self, point):
        self.outer_circle.append(point)
    
    def draw(self, screen):
        for i in range(len(self.inner_circle)):
            self.inner_circle[i].draw(screen)
        # Draw lines connecting the points
        for i in range(len(self.inner_circle)):
            pygame.draw.line(screen, (0, 0, 0), self.inner_circle[i].pos, self.inner_circle[(i+1)%len(self.inner_circle)].pos, 2)
        pass
    
    def update(self, event):
        if event.type == pygame.MOUSEBUTTONUP:
            for point in self.inner_circle:
                point.dragging = False
        if event.type in DraggablePoint.handledEvents:
            drag = False
            for point in self.inner_circle:
                if point._collides_with_point(event.pos):
                    drag = True
                    point.update(event)
                    break
               
        # break to smaller funcs
            if not drag and event.type == pygame.MOUSEBUTTONDOWN:
                self.add_inner_point(DraggablePoint(event.pos, 5))
