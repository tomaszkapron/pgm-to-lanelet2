import pygame

class DraggablePoint:
    handledEvents = [pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP, pygame.MOUSEMOTION]
    
    def __init__(self, pos, radius):
        self.pos = pos
        self.radius = radius
        self.dragging = False
        
    
    def update(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and not self.dragging:
            mouse_pos = pygame.mouse.get_pos()
            if self._collides_with_point(mouse_pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP and self.dragging:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            mouse_pos = pygame.mouse.get_pos()
            if self._is_within_bounds(mouse_pos):
                self.pos = mouse_pos
    
    def draw(self, surface):
        pygame.draw.circle(surface, (255, 0, 0), self.pos, self.radius)
    
    def _is_within_bounds(self, pos):
        dist_sq = (pos[0] - self.pos[0]) ** 2 + (pos[1] - self.pos[1]) ** 2
        return dist_sq <= self.radius ** 2
    
    def _collides_with_point(self, pos):
        return self._is_within_bounds(pos)