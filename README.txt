	________________________________________________________

	     IN-HAND RE-GRASP PLANNING USING FINGER GAITING
		       AND IN-GRASP MANIPULATION

	 Balakumar Sundaralingam, LL4MA Lab. [bala@cs.utah.edu]
	________________________________________________________


Table of Contents
_________________

1 Structure:
2 Messages:
3 Services:
4 Implementation details:
.. 4.1 TODO GetIK:
.. 4.2 TODO GetHandContact:
.. 4.3 TODO GetEllipse:
.. 4.4 TODO RelocateFinger:
.. 4.5 TODO MoveObj:
.. 4.6 TODO GetGaitPlan:


1 Structure:
============

  Look at program_structure.pdf for an outline of the proposed
  algorithm.


2 Messages:
===========


3 Services:
===========

  - GetHandContact.srv
  - GetEllipse.srv
  - GetIK.srv
  - RelocateFinger.srv
  - MoveObj.srv
  - GetGaitPlan.srv


4 Implementation details:
=========================

4.1 TODO GetIK:
~~~~~~~~~~~~~~~

  Given fingertip points in the reachable workspace of the fingers, this
  service finds the joint angles to reach the points. Current
  implemetation uses snopt with a quadratic cost on FK and constraints
  on the joint angle limits.


4.2 TODO GetHandContact:
~~~~~~~~~~~~~~~~~~~~~~~~

  This service finds the vertex on the object mesh that is closest to
  the fingetip location in the palm frame. Current implementation finds
  the closest vertex to a point in 3D space using trimesh python
  library.


4.3 TODO GetEllipse:
~~~~~~~~~~~~~~~~~~~~

  Given the reachable workspace of the fingers and the object mesh, this
  service finds 2D ellipses on the surface of the object that is
  reachable by the fingers. Currently only one ellipse is fitted per
  finger. Current implementation involves getting the intersection
  between the object mesh and the reachable workspace. Followed by
  clustering to find the largest surface cluster. PCA is done on this
  cluster to find a plane, to fit a 2D ellipse.


4.4 TODO RelocateFinger:
~~~~~~~~~~~~~~~~~~~~~~~~

  finger gaiting


4.5 TODO MoveObj:
~~~~~~~~~~~~~~~~~

  in-grasp manipulation


4.6 TODO GetGaitPlan:
~~~~~~~~~~~~~~~~~~~~~

  Get full plan
