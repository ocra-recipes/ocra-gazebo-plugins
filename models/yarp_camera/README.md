# Usage

```
cd [dir_with_jpgs]
ffmpeg -framerate 30 -start_number 1 -i default_camera_link_camera\(1\)\-%04d.jpg -s 1280x960 [video_name].mp4
```

example:
```
ffmpeg -framerate 30 -start_number 1 -i default_camera_link_camera\(1\)\-%04d.jpg -s 1280x960 test.mp4
# to run
vlc test.mp4
```
