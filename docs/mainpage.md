
# Table of Contents

1.  [Sub Packages](#orgf1238ab)
    1.  [Perception](#org27c7b9a)
    2.  [Motion Planning](#org2967a4e)
    3.  [Control](#org4368fea)
    4.  [Util](#org014aa37)
2.  [External Dependencies](#orgac3b415)

This package contains research code that I am working on as part of my PhD program at the University of Utah.
Visit my website for more information: <https://balakumar-s.github.io>


<a id="orgf1238ab"></a>

# Sub Packages


<a id="org27c7b9a"></a>

## Perception


### in\_hand\_collision\_lib

Contains collision checking classes.


<a id="org2967a4e"></a>

## Motion Planning


### in\_grasp\_manipulation

Reposes object with respect to the palm without breaking/making new contact.


### finger\_relocation

Relocates fingertip on the object surface.


### regrasp\_planner

Generates plans for regrasping the object using in-hand manipulation primitives.


<a id="org4368fea"></a>

## Control


### in\_hand\_feedback\_controller

Contains code to run generated motion plans on the robot(open-loop/feedback/MPC).


<a id="org014aa37"></a>

## Util


### in\_hand\_manipulation\_msgs

Contains custom messages and services used by other sub-packages.


### in\_hand\_manipulation\_pipeline

Contains scripts to evaluate various experiments.


### in\_hand\_manipulation\_data

Contains data files collected/used in various experiments.


<a id="orgac3b415"></a>

# External Dependencies

-   pcl\_mesh\_tools
-   ll4ma\_kdl
-   allegro\_driver
-   optimization\_pkg

