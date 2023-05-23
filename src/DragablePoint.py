import cv2

class DraggablePoint:
    handledEvents = [cv2.EVENT_LBUTTONDOWN, cv2.EVENT_LBUTTONUP, cv2.EVENT_MOUSEMOVE]
    
    def __init__(self, pos, radius):
        self.pos = pos
        self.radius = radius
        self.dragging = False
    
    def update(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not self.dragging:
            mouse_pos = (x, y)
            if self._collides_with_point(mouse_pos):
                self.dragging = True
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            mouse_pos = (x, y)
            if self._is_within_bounds(mouse_pos):
                self.pos = mouse_pos
    
    def draw(self, surface_name):
        cv2.circle(surface_name, self.pos, self.radius, (102, 255, 102), -1)
    
    def _is_within_bounds(self, pos):
        dist_sq = (pos[0] - self.pos[0]) ** 2 + (pos[1] - self.pos[1]) ** 2
        return dist_sq <= self.radius ** 2
    
    def _collides_with_point(self, pos):
        return self._is_within_bounds(pos)
