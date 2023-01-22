# bowl_sdf_generator

## preequisites
- install requirements
```
$ pip install -r requirements.txt
```

## usage
- run
```
$ ./sample_usage.sh
```

- or run with your own custom parameters
```
./make_bowl.py --num_division 16 --radius_bottom 0.08 --radius_top 0.1 --height 0.03 --thickness 0.003 --out_dir ./viewer/catkin_ws/src/gazebo_bringup/models/bowl
```

```
<--- (A) --->   
\           /   ^
 \         /    | (C)
  \_______/     v
   <-(B)->
```

- radius_top: (A)
- radius_bottom: (B)
- height: (C)

## viewer
- sample_usage.sh generate bowl model in viewer/catkin_ws/src/gazebo_bringup/models/bowl
- to see this model, run
```
$ cd viewer/catkin_ws
$ catkin_make
$ . ./devel/setup.bash
$ roslaunch gazebo_bringup simple_sim.launch
```
