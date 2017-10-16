		  ____________________________________

		     IN-HAND MANIPULATION PIPELINE

		   Balakumar Sundaralingam, LL4MA lab
		  ____________________________________


*In development!*

This package contains scripts to automate experiments for performing
in-hand manipulation.


1 srv msg types:
================

  - SendPreshape
  - GraspVelServo


1.1 TODO 
~~~~~~~~~

  - GraspPreshape
  - EEVelocityServo


2 Services:
===========

2.1 grasp_server.py:
~~~~~~~~~~~~~~~~~~~~

  This file has services to perform grasping. Currently available
  services:
  - `grasp/send_preshape':[SendPreshape] Sends the preshape to the
    robot.
  - `grasp/grasp_vel_servo':[GraspVelServo] grasps the object by
    velocity servoing from preshape.


2.1.1 TODO 
-----------

  - gen_preshape_server.py: Computes a preshape given the object mesh.
  - grasp_ee_velocity_servo_server.py: Velocity servoing the finger-tips
    towards the object from preshape.


3 Dev notes:
============

3.1 Grasp object:
~~~~~~~~~~~~~~~~~

  1. Obtain preshape from object mesh
  2. Send preshape to hand.
  3. Grasp object using velocity servoing.
  4. Enter Impedance mode to stabilize object.


3.2 Finger removing and placement:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  As a simple test, we will break finger-i contact on the object and
  make new contact (maybe in the same position)
  1. Remove finger(i) from impedance controller.
  2. Move finger(i) away from object.
  3. Check impedance controller stability.
  4. Move finger(i) towards object (velocity servoing).


3.3 Finger gaiting:
~~~~~~~~~~~~~~~~~~~

  1. Remove finger(i) from impedance controller.
  2. Move finger(i) away from object.
  3. Plan joint trajectory on finger(i) and send joint
     trajectory(position tracking) to hand.
  4. Place finger(i) on object using impedance mode?


4 Pipeline:
===========

4.1 Services:
~~~~~~~~~~~~~

  - object_mesh: Get Object pose and mesh from kinect + segmentation
    server.
  - object_pose: Get object pose from object_tracker server.
  - generate_grasp: Get grasp from object mesh using PCA or graspit.
  - generate_manipulator_plan: Get joint trajectory plan to move to the
    desired palm pose.
  - execute_manipulator_plan: Execute manipulator trajectory.
  - execute_grasp: Execute the desired grasp until the object is in
    contact.
  - lift_object: Lift object away from other objects (collision free).
  - generate_in_hand_manipulation: Generate in-hand manipulation
    trajectory (Open-loop).
  - generate_ilqr_gains: Generate ILQR gains for the trajectory.
  - execute_trajectory: open-loop or ILQR.
  - bag_service: records rostopics.


4.2 Currently available services:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

4.2.1 From Qingkai's pipeline:
------------------------------

  - object_mesh
  - object_pose
  - generate_grasp- PCA method.
  - generate_manipulator_plan- moveit planner.
  - execute_manipulator_plan- joint trajectory server.
  - lift_object- moveit planner.


4.2.2 In development:
---------------------

  - generate_grasp: graspit planner
  - generate_in_hand_manipulation: Optimizing SNOPT optimization code
    (achieve 2 seconds planning time).
