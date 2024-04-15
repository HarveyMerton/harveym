---
title: "Reinforcement learning for autonomous race cars"
excerpt: "Using reinforcement learning for local path following of an autonomous formula student race car."
header:
  image: /assets/images/6_4132_padm/arm_on_base.png
  teaser: assets/images/6_4132_padm/arm_on_base.png
sidebar:
  - title: "Summary"
    text: "Action and motion planning using PDDL, custom implementations of classical search algorithms, and nonlinear mathematical optimization."
  - title: "Skills"
    text: "Action planning with the 'planning domain definition language' (PDDL), breadth first and depth first search, heuristic search with the fast-forward planner, sample-based motion planning with RRT (and RRT*), mathematical programming for trajectory optimization with pyDrake, Franka Panda robotic arm simulation"
  # - title: "Code"
  # - label: "GitHub"
  #   icon: "fab fa-fw fa-github"
  #   url: "https://github.com/hmer101"
  # - title: "Documentation"
  # - label: "rajectory optimization and control"
  #   icon: "fas fa-fw fa-envelope"
  #   url: "mailto:hmer101@mit.edu"
gallery:
  - url: /assets/images/6_4132_padm/arm_on_base.png
    image_path: assets/images/6_4132_padm/arm_on_base.png
    alt: "Physical system on cart"
  # - url: /assets/images/masters/phys_testing_rig.jpg
  #   image_path: assets/images/masters/phys_testing_rig.jpg
  #   alt: "Drone on testing rig"
  # - url: /assets/images/masters/phys_on_ground_tall.jpg
  #   image_path: assets/images/masters/phys_on_ground_tall.jpg
  #   alt: "Physical system on ground"
---

https://www.fsae.co.nz/

- **[Code](https://github.com/hmer101/6_4132_arm_planning/tree/main)**
- **Documentation**:
  - [Project report: visual pose estimation]({{ site.url }}{{ site.baseurl }}/assets/docs/6_4132_padm/6_4132_final_report.pdf)


# Overview
This project was completed for MIT's graduate class '6.4132 principles of autonomy and decision making' (taught by Prof. Brian Williams). It focuses on higher-level deliberative planning across three areas: activity planning, motion planning and trajectory optimization. Please see the document attached above for a discussion of how each challenge was approached.  


{% include gallery caption="The environment: a Panda arm on a mobile base in a pyBullet simulated kitchen." %}


# Action planning results
The following action plan can be generated using the fast-forward planner (and breadth-first search) from types, predicates and actions defined in the 'planning domain definition language' (PDDL). The waypoints from this plan are then passed into the sample-based motion planner or trajectory optimizer to generate a motion plan for execution. 

![action_plan]({{ site.url }}{{ site.baseurl }}/assets/images/6_4132_padm/action_plan.png)


# Motion planning results
<figure>
  <video width="800" height="600" controls>
    <source src="{{ site.url }}{{ site.baseurl }}/assets/images/6_4132_padm/traj_opt_with_smoothing.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
    <figcaption>Trajectory optimization with smoothing.</figcaption>
</figure>

<figure>
  <video width="800" height="600" controls>
    <source src="{{ site.url }}{{ site.baseurl }}/assets/images/6_4132_padm/traj_opt_no_smoothing.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
    <figcaption>Trajectory optimization without smoothing.</figcaption>
</figure>

<figure>
  <video width="800" height="600" controls>
    <source src="{{ site.url }}{{ site.baseurl }}/assets/images/6_4132_padm/motion_planning_rrt.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
    <figcaption>Sample-based motion planning using RRT.</figcaption>
</figure>



