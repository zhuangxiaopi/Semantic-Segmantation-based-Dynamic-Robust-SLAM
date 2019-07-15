# Semantic-Segmantation-based-Dynamic-Robust-SLAM
Semantic-Segmantation-based-Dynamic-Robust-SLAM
1. Compiling the ORB_SLAM2 follow the README.md, and start a RGBD node.   
2. Putting the semantic_slam in ROS workspace, then use roslaunch to launch the semantic segmentation node.
Download the model in 
[model trained on ade20k](https://drive.google.com/file/d/1u_BEWdVIYiDnpVmAxwME1z3rnWWkjxm5/view?usp=sharing) 
[model trained on sunrgbd](https://drive.google.com/file/d/1t26t2VHNOzmjH-0lDTdYzXBACOV_4-eL/view?usp=sharing)
and put the models in model directory.
3. Running a .bag file in TUM database to publish rgb and depth images.
