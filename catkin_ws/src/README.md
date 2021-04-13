# To Run use the following:

```git submodule init``` # initialize the repo submodules
``` git submodule update``` # Get them



# To Stream video:
If using ssh don't forget to use the "share image buffer" flag:
```
ssh -X {name}@{host}
```

## Run the Camera Module
```rosrun cv_camera cv_camera_node                         
```
## Stream the video
```
rosrun image_view image_view image:=/cv_camera/image_raw
```
