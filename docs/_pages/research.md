---
permalink: /research
layout: page
title: Research
---

Our group aims to use modern machine-learning approaches to extend the state of the art of robotics and bring human-level autonomy and dexterity to Agile Justin. 



## [In-Hand Manipluation](https://aidx-lab.github.io/tactile-manipulation/)
![in-hand-manipulation](../assets/imgs/in-hand.png){:.this style="width: 1000px"}
Dextrous in-hand manipulation is a challenging problem in robotics. 
Due to modeling uncertainties, sensor noise, and other factors, classical control methods can solve this problem until now. 
We work on this problem using reinforcement learning in simulation.
Crucially, our approach is to learn a controller without external sensors, such as cameras [[Sievers2022](https://dlr-alr.github.io/dlr-tactile-manipulation/_pages/icra22.html)].
This creates the need for an object state estimator based on purely tactile feedback [[Roestel2023](https://dlr-alr.github.io/dlr-tactile-manipulation/_pages/humanoids23.html)].
We combine a simple policy network with the state estimator in a modular architecture to keep as much insight into the system as possible [[Pitz2023](https://dlr-alr.github.io/dlr-tactile-manipulation/_pages/icra23.html)].

## [Grasping](https://aidx-lab.github.io/grasping/)
![in-hand-manipulation](../assets/imgs/grasping.png){:.this style="width: 1000px"}
[more information](https://dlr-alr.github.io/grasping)

## [Motion Planning](https://aidx-lab.github.io/motion-planning/)
![tactile-calibartion](../assets/imgs/motion-planning.jpg){:.this style="width: 1000px"}
Fast and efficient motion planning in unknown environments is the basis for combining Agile Justin's individual skills to solve challenging tasks autonomously. 
* Supervised learning of optimal motions in unknown, challenging environments to speed up optimization-based motion planning ([more information](https://dlr-alr.github.io/2022-iros-planning/))
* Unsupervised learning of solutions to the ambiguous IK problem while tackling the different modes with a twin-head architecture ([more information](https://dlr-alr.github.io/2023-humanoids-ik/))

## [Calibration](https://aidx-lab.github.io/calibration/)
![tactile-calibartion](../assets/imgs/calibration-tactile.jpg){:.this style="width: 1000px"}
An accurate model of the whole robot is crucial to perform dextrous manipulation and grasping.
We developed self-contained calibration routines from the elastic rope mechanism in the torso to the fingertips of the torque-controlled DLR-Hand II. 
* Calibration of an elastic robot model for a humanoid and its efficient compensation for motion planning ([more information](https://dlr-alr.github.io/dlr-elastic-calibration/))
* Self-contained calibration using the internal camera while maintaining high accuracy in the cartesian workspace ([more information](https://dlr-alr.github.io/2022-humanoids-calibration/))
* Calibration of a four-fingered hand using only pairwise self-contact ([more information](https://dlr-alr.github.io/2023-humanoids-contact/))


