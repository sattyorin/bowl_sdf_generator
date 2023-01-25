#!/bin/bash

./make_bowl.py --model_name cooking_bowl --num_division 16 --radius_bottom 0.08 --radius_top 0.12 --height 0.06 --thickness 0.001 --out_mujoco_dir ~/lab/reinforcement_learning/envs/stir --out_gazebo_dir ~/lab/ros/stir_description/models
