# Building a Custom RViz Display

## Background
There are many types of data that have existing visualizations in RViz. However, if there is a message type that does
not yet have a plugin to display it, there are two choices to see it in RViz.

 1. Convert the message to another type, such as `visualization_msgs/Marker`.
 2. Write a Custom RViz Display.

With the first option, there is more network traffic and limitations to how the data can be represented. It is also quick and flexible.
The latter option is explained in this tutorial. It takes a bit of work, but can lead to much richer visualizations.

All of the code for this tutorial can be found in `this repository <https://github.com/MetroRobots/rviz_plugin_tutorial>`__.
In order to see the incremental progress of the plugin written in this tutorial,
the repository has different branches (`step2`, `step3`...) that can each be compiled and run as you go.


## Point2D Message
We'll be playing with a toy message defined in the [`rviz_plugin_tutorial_msgs`](rviz_plugin_tutorial_msgs) package: `Point2D.msg`:

```
std_msgs/Header header
float64 x
float64 y
```
