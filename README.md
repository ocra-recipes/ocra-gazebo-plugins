# ocra-gazebo-plugins

A warm and cozy place for all ocra-related(ish) gazebo pluginz - yeah with a z.

## Dependencies

All plugins require yarp. For linux follow http://www.yarp.it/install_yarp_linux.html for install.


To use the camera you will need `ffmpeg` as well. On linux:
```
sudo apt-get install ffmpeg
```

You probably already have it.

## How to use
To use the models, make sure to point your `GAZEBO_MODEL_PATH` to `[path to ocra-gazebo-plugins]/models`.

Also, make sure to set your `GAZEBO_PLUGIN_PATH` to point to wherever you choose to install the plugins. Then run the following

```
gazebo [path to ocra-gazebo-plugins]/worlds/test_ocra_task_widget.world 

```
to test.


## Camera
Yarpserver must be running before you launch the model so from a terminal type: `yarpserver`. Once running, in gazebo drop a `yarp_camera` model into the scene and orient it however you like. 

Now to operate the camera, from terminal you type

```
yarp rpc /Gazebo/camera_sensor/rpc:i
```

This will open up an rpc command promt. (note: if you have multiple cameras then the `camera_sensors` will be numbered e.g. `camera_sensor2`. Use tab-completion when typing the rpc port name to get a list.)

In your rpc prompt type the following to record:
```
>>> record 1 [dir_to_save_png] [video_name]
```
Both `[dir_to_save_png]` and `[video_name]` are optional, but you must specify a folder if you specify the name.

To stop recording type:

```
>>> record 0
```
You will see where everything is when you stop recording. Also, give FFMpeg some time to compile the video before closing the terminal. 

Enjoy!

