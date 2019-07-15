# Semantic-Segmantation-based-Dynamic-Robust-SLAM
Semantic-Segmantation-based-Dynamic-Robust-SLAM
1. Compiling the ORB_SLAM2 follow the README.md, and start a RGBD node.   
2. Putting the semantic_slam in ROS workspace, then use roslaunch to launch the semantic segmentation node.
Download the model in
<br />[model trained on ade20k](https://drive.google.com/file/d/1u_BEWdVIYiDnpVmAxwME1z3rnWWkjxm5/view?usp=sharing)
<br />[model trained on sunrgbd](https://drive.google.com/file/d/1t26t2VHNOzmjH-0lDTdYzXBACOV_4-eL/view?usp=sharing)
<br />and put them in models.
3. Running a .bag file in TUM database to publish rgb and depth images.
## Acknowledgement
This work cannot be done without many open source projets. Special thanks to
<br />[semantic_slam](https://github.com/floatlazer/semantic_slam)
<br />[ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
<br />[ORB_SLAM2_SSD_Semantic](https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic)
## License
This project is released under a GPLv3 license.
