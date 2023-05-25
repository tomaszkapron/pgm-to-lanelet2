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
from track import LoopTrack


class TrackGeneratorApp:

    def __init__(self, track_img: cv2.Mat, map_config: dict) -> None:
        self.generatorRunning = True

        self.track = LoopTrack(map_config=map_config, track_img_shape=track_img.shape)
        self.track_img = track_img
        self.screen_name = 'Track Generator'
        self.screen = cv2.namedWindow(self.screen_name, cv2.WINDOW_KEEPRATIO)
        cv2.setMouseCallback(self.screen_name, self.mouseCallback)

    def run(self) -> None:
        while self.generatorRunning:
            canvas = self.track_img.copy()
            self.drawTrack(canvas)

            result = cv2.bitwise_and(self.track_img, canvas)
            cv2.imshow(self.screen_name, result)

            key = chr(cv2.waitKey(1) & 0xFF)
            self.track.handle_keyboard_input(key)

    def drawTrack(self, canvas: cv2.Mat) -> None:
        self.track.draw(canvas)

    def mouseCallback(self, event: int, x: int, y: int, flags: int, param: None) -> None:
        self.track.update(event, x, y, flags, param)
