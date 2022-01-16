# Tutorial

* To save the map dynamically using the preset topic:

      rosrun map_server map_saver map:=/$MAP_TOPIC -f $OUTPUT_MAP_FILENAME

### RTABMap

* Converting the `.db` file to `.pgm` map file:

      roscore
      rosrun rtabmap_ros rtabmap _database_path:=your_file_name.db
      rosrun map_server map_saver map:=/rtabmap/grid_map -f $OUTPUT_MAP_FILENAME