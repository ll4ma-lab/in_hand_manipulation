
# Table of Contents

1.  [Sub Packages](#orge548763)
    1.  [Perception](#orgba39b19)
    2.  [Motion Planning](#org555615a)
    3.  [Control](#org041082e)
    4.  [Util](#org8f49511)
2.  [External Dependencies](#org82081d9)
    1.  [Harwdare specific packages](#org52b0314)

This package contains research code that I am working on as part of my PhD program at the University of Utah.
Visit my website for more information: <https://balakumar-s.github.io>


<a id="orge548763"></a>

# Sub Packages


<a id="orgba39b19"></a>

## Perception


### in\_hand\_collision\_lib

Contains collision checking classes.


<a id="org555615a"></a>

## Motion Planning


### in\_grasp\_manipulation

Plans joint space motion for reposing object with respect to the palm without breaking/making new contact.


### finger\_relocation

Plans joint space motion to relocate fingertip on the object surface by finger gaiting and fingertip sliding.


### regrasp\_planner

Generates plans for regrasping the object using in-hand manipulation primitives.


<a id="org041082e"></a>

## Control


### in\_hand\_controllers

Contains code to run generated motion plans on the robot(open-loop/feedback/MPC). This package wraps around hand\_services to enable task specific controllers.


<a id="org8f49511"></a>

## Util


### in\_hand\_manipulation\_msgs

Contains custom messages and services used by other sub-packages.


### in\_hand\_manipulation\_pipeline

Contains scripts to evaluate various experiments.


### in\_hand\_manipulation\_data

Contains data files collected for motion planning and experiments.


### in\_hand\_manipulation

Contains launch files for various in-hand manipulation skills. This package also contain demos.


<a id="org82081d9"></a>

# External Dependencies

-   [ll4ma\_robots\_description](https://bitbucket.org/robot-learning/ll4ma_robots_description)
-   [ll4ma\_collision\_wrapper](https://bitbucket.org/robot-learning/ll4ma_collision_wrapper)
-   [ll4ma\_kdl](https://bitbucket.org/robot-learning/ll4ma_kdl)
-   [ll4ma\_opt\_wrapper](https://bitbucket.org/robot-learning/ll4ma_opt_wrapper)


<a id="org52b0314"></a>

## Harwdare specific packages

-   [allegro\_driver](https://bitbucket.org/robot-learning/allegro_driver)
-   [hand\_services](https://bitbucket.org/robot-learning/hand_services)
-   [biotac\_sensors](https://bitbucket.org/robot-learning/biotac_sensors)

