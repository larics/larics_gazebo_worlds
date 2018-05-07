# larics_gazebo_worlds

larics_gazebo_worlds contains all the worlds used in our Gazebo simulation environments. To use the worlds simple clone the directory in your ros_workspace, and launch the appropriate file.

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
