import pygame
from DragablePoint import DraggablePoint
from PointsToOsmConv import PointsToOsmConverter
from enum import Enum

class LoopType(Enum):
    INNER = 0
    OUTER = 1

class LoopTrack:
    def __init__(self):
        self.inner_circle = []
        self.outer_circle = []
        self.circle_type = LoopType.OUTER
    
    def add_inner_point(self, point):
        self.inner_circle.append(point)
    
    def add_outer_point(self, point):
        self.outer_circle.append(point)
        
    def add_point(self, point):
        if self.circle_type == LoopType.OUTER:
            self.outer_circle.append(point)
        else:
            self.inner_circle.append(point)
        
    def remove_point(self):
        if self.circle_type == LoopType.OUTER:
            if len(self.outer_circle) > 0:
                self.outer_circle.pop()
        else:
            if len(self.inner_circle) > 0:
                self.inner_circle.pop()
    
    def draw(self, screen):
        for i in range(len(self.inner_circle)):
            self.inner_circle[i].draw(screen)
            pygame.draw.line(screen, (0, 0, 0), self.inner_circle[i].pos, self.inner_circle[(i+1)%len(self.inner_circle)].pos, 2)
        
        for i in range(len(self.outer_circle)):
            self.outer_circle[i].draw(screen)
            pygame.draw.line(screen, (0, 0, 0), self.outer_circle[i].pos, self.outer_circle[(i+1)%len(self.outer_circle)].pos, 2)
    
    
    
    def update(self, event):
        if event.type == pygame.MOUSEBUTTONUP:
            for point in self.inner_circle + self.outer_circle:
                point.dragging = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_z:
                self.remove_point()
            if event.key == pygame.K_x:
                self.circle_type = LoopType.INNER if self.circle_type == LoopType.OUTER else LoopType.OUTER
            if event.key == pygame.K_s:
                converter = PointsToOsmConverter(self.inner_circle, self.outer_circle)
                converter.convert()
        if event.type in DraggablePoint.handledEvents:
            drag = False
            for point in self.inner_circle + self.outer_circle:
                if point._collides_with_point(event.pos):
                    drag = True
                    point.update(event)
                    break
               
        # break to smaller funcs
            if not drag and event.type == pygame.MOUSEBUTTONDOWN:
                print(event.pos)
                self.add_point(DraggablePoint(event.pos, 5))
