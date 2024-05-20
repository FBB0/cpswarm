## This is the repository of group 13 from the Multi Disciplinary Project
In this document, all instructions to build the code and explanation about the implementation can be found.

- perception:
    - use camera to detect obstacles
    - TODO: debug 2D to 3D; yolob8 weights for detection
- state estimation:
    - lidar preprocessing to remove lidar points in certain orientation
    - gmapping for offline mapping
    - cartographer for offline mapping (not included in repo)
    - amcl for localization
    - cartographer for localization (not included in repo)
    - TODO: add detection results to Bird Eye View of barn, track obstables and remove their point clouds localization