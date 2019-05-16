# larics\_gazebo\_worlds

larics\_gazebo\_worlds package contains all the worlds used in our Gazebo simulation environments. To use the worlds simple clone the directory in your ros_workspace, and launch the appropriate file.

## Generating files for new world
There are three types of files for each model, that are used within this repository:

* .dae - This provides 3D model that is suitable for visualization in Gazebo environment.
* .world - This sets up world in Gazebo based on previously mentioned .dae file.
* .binvox.bt - This file represents a map suitable for path planning in spawned environment. It is not necessary to have this file for your world, it is recommended for planning.


### Generating *octomap*
In folder ```scripts``` there are files that can transform 3D model of your world to .binvox.bt format to represent octomap. 

#### Converting from *.stl* file

Convet *.stl* to *.binvox* by running: ```./binvox -e res path_to_file ```. res is resolution in voxels, the larger this number better the resolution. **Note** that this greatly impacts memory usage and process might not even start. For res=4096 on a 26MB *.stl* file it used around 65GB of RAM.

Next convert *.binvox* to *.binvox.bt* by running:

```binvox2bt --bb <minx> <miny> <minz> <maxx> <maxy> <maxz> path_to_file```

#### Converting from *.dae* file

You will have to convert your .dae file to .x3d file format in order for this to work. The conversion can be done by calling:

```bash
./x3dToBinwoxbt.sh path_to_file
```

The map file will be saved to the same folder where .x3d file is.

### Map has voxels where they don't belong
The aforementioned procedure can produce some weird results, like occupied voxels in the map where they shouldn't be. This can be solved by rotating map in your editor and exporting it with different rotation. If this happens try to use different rotations until you are satisfied with the final map. The map will then be rotated and may not be suitable for planning, that's why there is a ROS node that can rotate the map and save it. You can run the conversion with:

```bash
rosrun larics_gazebo_worlds transform_octomap input_map:=path_to_input_map output_map:=path_to_output_map config_file:=path_to_config_file
```

An example of a configuration file is provided within ```config``` folder. It is a .yaml file with transformation matrix. You can edit this file or create your own file with transformation.

## Adding a new World
To add a new world to this repository simple follow this guide. You will need three files:
- world.launch : launch file that is called to start the simulation with your world. It should look something like this:
```xml
<launch>
  <arg name="args" default=""/>
  <include file="$(find larics_gazebo_worlds)/launch/start.launch">
    <arg name="world" value="$(find larics_gazebo_worlds)/worlds/world.world"/>
    <arg name="args" default="$(arg args)"/>
  </include>
</launch>
```
- model.dae: all models should be exported as collada files and referenced as visual and coolision blocks within world.sdf file. For more info on this please consult the provided files in the repository.
- world.world: world files can be saved using Gazebo in .sdf format. They provide the means to run the simulation with the provided map, models and physics. Using a workaround from [answers.gazebosim.com](http://answers.gazebosim.org/question/6416/using_a_urdf_in_gazebo-package-uris-not-described/) you can reference dae files from the repository using the following code snippet:

```xml
<visual name='new_model_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>model://larics_gazebo_worlds/models/model.dae</uri>
      </mesh>
    </geometry>
  </visual>
```
For example, imagine you are addin a new world called new_world with a model new_map spawned at the start of the simulation. You should create the following files in the folder structure:
```
../catkin_ws/src
    /larics_gazebo_worlds
        package.xml
        CMakeLists.txt
        /models
            ...
            /new_map
                model.dae
                model.config
        /worlds
            new_world.world
            ...
        /launch
            new_world.launch
            ...
        
```
