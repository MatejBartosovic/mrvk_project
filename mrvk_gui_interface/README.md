# Mrvk_gui_interface

This node is for keep way points in robot computer and provide way points
interface for **mrvk_gui** node.

### Requirements:
* launched **move_base** node
* uploaded params on param server:
    * **/move_base/Planner/origin_latitude**
    * **/move_base/Planner/origin_longitude**
* **osm_planner** package (for coordinates_converters)

### This node provide services:
```text
~/add_waypoint_to_queue
~/add_waypoints_to_queue
~/erase_waypoints_queue
```

### Provide Action server as interface for process movement
```text
~\process_waypoints
```