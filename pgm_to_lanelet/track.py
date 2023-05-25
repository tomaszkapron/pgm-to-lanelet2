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

from enum import Enum

import cv2

from dragable_point import DraggablePoint
from points_to_osm_conv import PointsToOsmConverter


class LoopType(Enum):
    INNER = 0
    OUTER = 1


class LoopTrack:

    def __init__(self, map_config: dict, track_img_shape: tuple) -> None:
        self.map_config = map_config
        self.track_img_shape = track_img_shape

        self.inner_circle = []
        self.outer_circle = []
        self.circle_type = LoopType.OUTER

    def add_inner_point(self, point: DraggablePoint) -> None:
        self.inner_circle.append(point)

    def add_outer_point(self, point: DraggablePoint) -> None:
        self.outer_circle.append(point)

    def add_point(self, point: DraggablePoint) -> None:
        if self.circle_type == LoopType.OUTER:
            self.outer_circle.append(point)
        else:
            self.inner_circle.append(point)

    def remove_point(self) -> None:
        if self.circle_type == LoopType.OUTER:
            if len(self.outer_circle) > 0:
                self.outer_circle.pop()
        else:
            if len(self.inner_circle) > 0:
                self.inner_circle.pop()

    def draw(self, screen_name: str) -> None:
        for i in range(len(self.inner_circle)):
            self.inner_circle[i].draw(screen_name)
            cv2.line(screen_name, self.inner_circle[i].pos,
                     self.inner_circle[(i+1) % len(self.inner_circle)].pos, (51, 153, 255), 2)
        for i in range(len(self.outer_circle)):
            self.outer_circle[i].draw(screen_name)
            cv2.line(screen_name, self.outer_circle[i].pos,
                     self.outer_circle[(i+1) % len(self.outer_circle)].pos, (51, 153, 255), 2)

    def update(self, event: int, x: int, y: int, flags: int, param: None) -> None:
        if event == cv2.EVENT_LBUTTONUP:
            for point in self.inner_circle + self.outer_circle:
                point.dragging = False
        if event in DraggablePoint.handledEvents:
            drag = False
            for point in self.inner_circle + self.outer_circle:
                if point._collides_with_point((x, y)):
                    drag = True
                    point.update(event, x, y, flags, param)
                    break

            if not drag and event == cv2.EVENT_LBUTTONDOWN:
                print((x, y))
                self.add_point(DraggablePoint((x, y), 5))

    def toggle_circle_type(self) -> None:
        self.circle_type = LoopType.INNER if self.circle_type == LoopType.OUTER else LoopType.OUTER

    def handle_keyboard_input(self, key: str) -> None:
        if key == 'z':
            self.remove_point()
        if key == 'x':
            self.toggle_circle_type()
        if key == 's':
            converter = PointsToOsmConverter(self.inner_circle,
                                             self.outer_circle,
                                             map_config=self.map_config,
                                             track_img_shape=self.track_img_shape)
            converter.convert()
