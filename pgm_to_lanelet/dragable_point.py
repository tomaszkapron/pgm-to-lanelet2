#!/usr/bin/env python3

# Copyright 2023 Perception for Physical Interaction Laboratory at Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2


class DraggablePoint:
    handledEvents = [cv2.EVENT_LBUTTONDOWN, cv2.EVENT_LBUTTONUP, cv2.EVENT_MOUSEMOVE]

    def __init__(self, pos: tuple, radius: int) -> None:
        self.pos = pos
        self.radius = radius
        self.dragging = False

    def update(self, event: int, x: int, y: int, flags: int, param: None) -> None:
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

    def draw(self, surface_name: str) -> None:
        cv2.circle(surface_name, self.pos, self.radius, (102, 255, 102), -1)

    def _is_within_bounds(self, pos: tuple) -> bool:
        dist_sq = (pos[0] - self.pos[0]) ** 2 + (pos[1] - self.pos[1]) ** 2
        return dist_sq <= self.radius ** 2

    def _collides_with_point(self, pos: tuple) -> bool:
        return self._is_within_bounds(pos)
