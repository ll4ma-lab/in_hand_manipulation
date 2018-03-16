- [Sub Packages](#orgbcba842)
  - [Perception](#orge0ec722)
  - [Motion Planning](#org73accc3)
  - [Control](#org857c512)
  - [Util](#org4b18f17)
- [External Dependencies](#org3044e2f)

This package contains research code that I am working on as part of my PhD program at the University of Utah. Visit my website for more information: <https://balakumar-s.github.io>


<a id="orgbcba842"></a>

# Sub Packages


<a id="orge0ec722"></a>

## Perception


### in\_hand\_collision\_lib

Contains collision checking classes.


<a id="org73accc3"></a>

## Motion Planning


### in\_grasp\_manipulation

Reposes object with respect to the palm without breaking/making new contact.


### finger\_relocation

Relocates fingertip on the object surface.


### regrasp\_planner

Generates plans for regrasping the object using in-hand manipulation primitives.


<a id="org857c512"></a>

## Control


### in\_hand\_feedback\_controller

Contains code to run generated motion plans on the robot(open-loop/feedback/MPC).


<a id="org4b18f17"></a>

## Util


### in\_hand\_manipulation\_msgs

Contains custom messages and services used by other sub-packages.


### in\_hand\_manipulation\_pipeline

Contains scripts to evaluate various experiments.


### in\_hand\_manipulation\_data

Contains data files collected/used in various experiments.


<a id="org3044e2f"></a>

# External Dependencies

-   pcl\_mesh\_tools
-   ll4ma\_kdl
-   allegro\_driver
-   optimization\_pkg
