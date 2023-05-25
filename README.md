# pgm_to_lanelet
<!-- Required -->
<!-- Package description -->
A tool for drawing a Lanelet2 map using occupancy grid map (.pgm) as a background. 
The package is useful for basic vector map creation which contains only road lanes.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to pgm_to_lanelet
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch pgm_to_lanelet pgm_to_lanelet.launch.py map_yaml_path:=path/to/map.yaml
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Parameters

| Name            | Type   | Description                    |
| --------------- | ------ | ------------------------------ |
| `map_yaml_path` | string | Path to .pgm map config.       |
| `line_width`    | int    | GUI line width between points. |
| `point_size`    | int    | GUI point size.                |


## User Interface

| key / mouse button            | Description                  |
| ---------------      | ------------------------------        |
| `z`                  | remove last drawn point               |
| `x`                  | change between inner and outer loop   |
| `s`                  | save drawn map to .osm file           |
| `left mouse button`  | draw point                            |
| `mouse scroll`       | zoom in/out the map                   |
| `ctrl + arrows`      | move the image while beeing zoomed in |

## References / External links
<!-- Optional -->
* [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
