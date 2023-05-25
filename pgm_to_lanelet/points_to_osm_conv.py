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

from pathlib import Path

from typing import TextIO

from dragable_point import DraggablePoint


class PointsToOsmConverter:

    def __init__(self, inner_points: list,
                 outer_points: list,
                 map_config: dict,
                 track_img_shape: tuple) -> None:
        self._inner_points = inner_points
        self._outer_points = outer_points
        self._inner_ids = []
        self._outer_ids = []
        self._inner_way_id = 1000
        self._outer_way_id = 1001
        self._relation_id = 1002

        self._id = 1

        self._resolution = map_config['resolution']
        origin = map_config['origin']
        self._origin = (origin[0], origin[1])
        self._track_img_shape = track_img_shape

        self._file_name = 'lanelet_map.osm'
        self._output_dir = Path(__file__).parent.parent / 'data'

    def reset_converter(self) -> None:
        pass

    def convert(self) -> None:
        with open(self._output_dir / Path(self._file_name), 'w') as f:
            self._write_header(f)
            self._write_nodes(f)
            self._write_ways(f)
            self._write_relation(f)
            self._write_footer(f)

    def _write_header(self, f: TextIO) -> None:
        f.write("<?xml version='1.0' encoding='UTF-8'?>\n")
        f.write('<osm generator="VMB">\n')
        f.write('   <MetaInfo format_version="1" map_version="2"/>')

    def _write_nodes(self, f: TextIO) -> None:
        for point in self._inner_points:
            f.write(self._write_node(point))
            self._inner_ids.append(self._id)
            self._id += 1
        for point in self._outer_points:
            f.write(self._write_node(point))
            self._outer_ids.append(self._id)
            self._id += 1

    def _write_ways(self, f: TextIO) -> None:
        f.write(self._write_way(self._inner_ids, self._inner_way_id))
        f.write(self._write_way(self._outer_ids, self._outer_way_id))

    def _write_relation(self, f: TextIO) -> None:
        relation = '''
    <relation id="{id}">
      <member type="way" role="left" ref="{inner_way_id}"/>
      <member type="way" role="right" ref="{outer_way_id}"/>
      <tag k="type" v="lanelet"/>
      <tag k="subtype" v="road"/>
      <tag k="speed_limit" v="1000"/>
      <tag k="location" v="urban"/>
      <tag k="one_way" v="yes"/>
    </relation>'''.format(id=self._relation_id, inner_way_id=self._inner_way_id, outer_way_id=self._outer_way_id)

        f.write(relation)

    def _write_footer(self, f: TextIO):
        f.write('\n</osm>')

    def _write_node(self, point: DraggablePoint) -> str:
        x, y = self._pixels_to_meters(point)

        node = '''
    <node id="{id}" lat="" lon="">
      <tag k="mgrs_code" v="99XXX0-1000"/>
      <tag k="local_x" v="{x}"/>
      <tag k="local_y" v="{y}"/>
      <tag k="ele" v="0"/>
    </node>'''.format(id=self._id, x=x, y=y)

        return node

    def _write_way(self, points_id_list: list, way_id: int) -> str:
        if len(points_id_list) < 2:
            return ''

        way_text = '\n    <way id="{id}">\n'.format(id=way_id)

        for point_id in points_id_list:
            way_text += '      <nd ref="{point_id}"/>\n'.format(point_id=point_id)

        way_text += '      <nd ref="{point_id}"/>\n'.format(point_id=points_id_list[0])
        way_text += '      <tag k="type" v="line_thin"/>\n'
        way_text += '      <tag k="subtype" v="solid"/>\n'

        way_text += '   </way>\n'

        return way_text

    def _pixels_to_meters(self, point: DraggablePoint) -> tuple:
        x = point.pos[0] * self._resolution + self._origin[0]
        y = (point.pos[1] * self._resolution * -1) + self._origin[1] + self._track_img_shape[1] * self._resolution
        return x, y
