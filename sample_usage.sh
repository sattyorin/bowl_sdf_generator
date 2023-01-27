#!/bin/bash

./make_bowl.py --model_name cooking_bowl --num_division 16 --radius_bottom 0.04 --radius_top 0.0712 --height 0.054 --thickness 0.006 --out_mujoco_dir ~/lab/reinforcement_learning/envs/stir --out_gazebo_dir ~/lab/ros/stir_description/models
