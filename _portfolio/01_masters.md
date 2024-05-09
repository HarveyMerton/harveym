---
title: "Multi-drone slung load carrying"
excerpt: "Using a system of multiple drones to carry a single slung load. Allows the carrying of heavier loads with more precise control over load pose."
header:
  image: /assets/images/masters/phys_on_ground.jpg
  teaser: assets/images/masters/phys_on_ground.jpg
sidebar:
  - title: "Summary"
    text: "Using a system of multiple drones to carry a single slung load. Allows the carrying of heavier loads with more precise control over load pose."
  - title: "Skills"
    text: "Mutli-agent coordination, computer vision, openCV, visual pose estimation, universal differential equations, machine learning, trajectory optimization, control, networking, ROS2, gazebo, PX4 autopilot, python, C++, julia, drake."
  # - title: "Code"
  # - label: "GitHub"
  #   icon: "fab fa-fw fa-github"
  #   url: "https://github.com/hmer101"
  # - title: "Documentation"
  # - label: "rajectory optimization and control"
  #   icon: "fas fa-fw fa-envelope"
  #   url: "mailto:hmer101@mit.edu"
gallery:
  - url: /assets/images/masters/phys_on_cart.jpg
    image_path: assets/images/masters/phys_on_cart.jpg
    alt: "Physical system on cart"
  - url: /assets/images/masters/phys_testing_rig.jpg
    image_path: assets/images/masters/phys_testing_rig.jpg
    alt: "Drone on testing rig"
  - url: /assets/images/masters/phys_on_ground_tall.jpg
    image_path: assets/images/masters/phys_on_ground_tall.jpg
    alt: "Physical system on ground"
---

- **Code**: Will be open-sourced after publishing. 
- **Documentation**:
  - [Project report: visual pose estimation]({{ site.url }}{{ site.baseurl }}/assets/docs/masters/16_485_project_report.pdf)
  - [Project report: cable dynamics modelling with universal differential equations]({{ site.url }}{{ site.baseurl }}/assets/docs/masters/18_337_project_report.pdf)
  - [Project report: trajectory optimization and control with Drake]({{ site.url }}{{ site.baseurl }}/assets/docs/masters/6_8210_project_report.pdf)
  - [Workshop poster IROS 2023: agricultural methane emissions measurement with a multi-drone slung load system]({{ site.url }}{{ site.baseurl }}/assets/docs/masters/IROS2023_WRIA_HM_poster.pdf)


# Overview
This project forms the core of my Master's work, due to be complete August 2024. The motivation is to create a platform to autonomously deploy any sensor across a range of outdoor environments. Using an aerial system is ideal as they can quickly access challenging areas. Using multiple drones allows for more control over the slung load's pose. Further, heavier loads can be carried, and the air downwash from the drones' propellers can be shifted laterally away from the sensing device, thus preventing interference with sensor readings.

As the sole researcher working on the project, I independently built the testing system from the ground up. This involved *extensive* development in ROS2 (both python and c++) and PX4 autopilot to create a full simulation and real-world testing platform. This work required knowledge across the robotics stack from computer vision, pose estimation, trajectory optimization/motion planning, and controls. Further, I have scoped, driven and executed novel research on the load's visual pose estimation and cable dynamics modelling systems. At least one publication and a thesis will come out of this project on completion, but please see the documents linked at the beginning of this page for now. These reports were generated as part of various graduate classes at MIT including: '16.485 visual navigation' (taught by Prof. Luca Carlone), '18.337 parallel computing and scientific machine learning' (taught by Prof. Alan Edelman), and '6.8210 underactuated robotics' (taught by Prof. Russ Tedrake).


{% include gallery caption="The physical multi-drone slung load platform I custom designed and built for testing." %}

<figure>
  <video width="800" height="600" controls>
    <source src="{{ site.url }}{{ site.baseurl }}/assets/images/masters/16.485_compiled_videos.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
    <figcaption>Visual pose estimation functioning in a Gazebo simulation.</figcaption>
</figure>


<!-- # Visual Pose Estimation


# Cable Dynamics Modelling


# Trajectory Optimization and Control


# Application: Measuring Methane Emissions -->
