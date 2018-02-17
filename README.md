# Robo_nd Project3 3D Perception

---

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Perception-Project) into the ***src*** directory of your ROS Workspace.  
3.Set up the downsampling and RANSAC pipeline.
4.Set up the cluster pipeline.
5.Set up svm pipeline for recognition.
6. Fill in the `Project_code.py` with my own written code. 



---

## Pipeline setup
### 1. Run the pipeline for sampling
1. As point cloud stored in [voxel_downsampled.pcd], firstly through the  domnsampling to reduce the number of point clouds.
2. Then pass through the filter along the 'y' and 'z' axis to get rid of useless point clouds,and get [pass_through_filtered.pcd]
3. RANSAC to fit the cloud points into the model, with registered indices to separate the objects in [extracted_outliers.pcd] from the table [extracted_inliers.pcd]
### 2. Run the segmentation and cluster pipeline
The clustered point clouds are stored in [table_scene_lms400_outliers.pcd]
### 3. Run the recognition pipeline
Use "compute_color_histograms()" and "compute_normal_histograms()" function from the capture_features module and then the feasture vector is obtained and can be used as the training set for svm, for the 3 world's scenes with 3 different pickup lists, then the svm should load accordingly the right dataset.

### 3. Feature vector extract
From line 59 to 63 in capture_features.py, the histogram sequencefor colors and normals of surface are concatenated, to be used for comparison. Then the normalized confusion matrix is presented as following.
## Implementation in project environment		
For the training model, the confusion matrix visualizes the percentage of successful recognition.
![Confusion_matrxi_scene1](Norm_confusion_matrix_scene1.png)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Confusion_matrxi_scene1<br />
It is perfect to recognize all three objects in scene1, and all with the rate larger than 97%, for soap2 with the 100% is even achieved. The surface norms of soap2 is more remarkable, so it is prone to be recognized easily.
![Confusion_matrxi_scene2](Norm_confusion_matrix_scene2.png)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Confusion_matrxi_scene2<br />
The good performance is also presented in scene2, and all objects is recognized with the accuracy of above 90%. Except that there is a small drop to the rate of biscuits, and this may be  due the irregular surface of it, and resulting in some misclassification.
![Confusion_matrxi_scene3](Norm_confusion_matrix_scene3.png)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Confusion_matrxi_scene3
In scene3, there are 6 of 8 object(75%), still with above 90% recognition accuracy, but the problem lies in the recognition between biscuits and sticky_notes, both of them are confused together. This is because the similar color histogram and norms histogram, resulting in the difficulty for recognition.
The pipeline is same as above and an additional step is added for the filtering out the noise points using statistical method . Then the core part of the recognition is done within a nested loop, the outside loop is incremented with the object enumeration in the pickup list, in the inner loop the compare, between the recognized object in cluster cloud and the one in pickup list is run, to check if the recognized object is right there and then calculate the centroid of the target, which will be used for pick and place task.
The outcome is quite good, in scene 1 and scene 2, all objects are recognized correctly, in scene 3 only one object is not recognized sometimes due to the occlusion caused by another object, the camera pose is static in my test and the distance between them is also very small.
![recognized_marker world1](rviz_screenshot_world1.png)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;recognized_marker world1<br />
In scene1 all objects are recognized accordingly, and high accuracy in training model makes the recognition a piece of cake.
![recognized_marker world2](rviz_screenshot_world2.png)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;recognized_marker world2<br />
In scene2 all objects are recognized similarly.
![recognized_marker world3](rviz_screenshot_world3.png)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;recognized_marker world3<br />
The misclassified models of biscuits and stiky_notes are mapped into the recognition process, sometimes the sticky_motes is misclassified as biscuits, sometimes correctly recognized, the switch between them is quite often. 
And each set of parameters for the recognized object is aslo stored into the dictionary and forwarded to the yaml file,[output_1.yaml] ,[output_2.yaml], [output_3.yaml] 
The request for the pick and place information is commented out because I do want to make the infomation in my termial clean without the red error promption.
I think if the robot coud rotate a little bit, then the occluded object will be found again, so the command to the joint should be added in the futrue to solve the problem in scene3.
For the improvement of model training in  scene3, I think in the future the number of the norms of whole surface area should be counted, because the sticky_notes surface area is obviously smaller compared to that of biscuits, so this feature should be used for recognition. 



