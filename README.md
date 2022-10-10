# MRS SubT planning library

The library including A* planner and path postprocessing used by UAVs of team CTU-CRAS-NORLAB at DARPA SubT challenge since the Urban Circuit.

The core of the method is described in publication:
```
[1] V. Kratky, P. Petracek, T. Baca, and M. Saska, "An Autonomous Unmanned Aerial Vehicle System for Fast 
Exploration of Large Complex Indoor Environments," Journal of Field Robotics, vol. 38, no. 8, pp. 1036–1058, 
May 2021, https://doi.org/10.1002/rob.22021.
```

Further details and description of the deployment of the planner within the system designed for the exploration of subterranean environments are provided in publications:
```
[2] P. Petracek, V. Kratky, M. Petrlik, T. Baca, R. Kratochvil, and M. Saska, "Large-Scale Exploration 
of Cave Environments by Unmanned Aerial Vehicles," in IEEE Robotics and Automation Letters, vol. 6, no. 4, 
pp. 7596-7603, Oct. 2021, https://doi.org/10.1109/LRA.2021.3098304.

[3] M. Petrlik, P. Petracek, V. Kratky, T. Musil, Y. Stasinchuk, M. Vrba, T. Baca, D. Hert, M. Pecka, 
T. Svoboda, and M. Saska, "UAVs Beneath the Surface: Cooperative Autonomy for Subterranean Search 
and Rescue in DARPA SubT," preprint arXiv:2206.08185, 2022.
```

The library is integrated in the [octomap_mapping_planning](https://github.com/ctu-mrs/octomap_mapping_planning) meta-repository on subt_planner branch.
