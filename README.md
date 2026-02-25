# Perception Lab: Corner Extraction and Room Geometry using ALTBD

## 📖 Introduction
Mobile robots rely on accurate localization to navigate indoor spaces effectively. Light Detection and Ranging (LiDAR) sensors are preferred for this task because they provide fast, illumination-independent distance measurements. 

In this lab, you will implement a state-of-the-art perception algorithm: the Adaptive Line Tracking Breakpoint Detector (ALTBD). Older methods, like Line Tracking (LT), struggle with hardcoded threshold values. The Adaptive Breakpoint Detector (ABD) improves upon this by using dynamic thresholds but still requires a computationally expensive secondary step called Iterative End Point Fit (IEPF) to extract room corners. 

ALTBD combines the strengths of both methods, eliminating the need for the slow IEPF step by utilizing an adaptive, elliptical detection zone. This allows the robot to detect room corners faster and with much higher accuracy.

---

## ⚙️ Dependencies and Setup
To run this lab, you must have the standard F1TENTH ROS 2 stack installed on your machine or container.
* **ROS 2:** Humble/Foxy (Depending on your lab setup)
* **Python Packages:** `rclpy`, `numpy`, `math`
* **Simulator:** F1TENTH Gym or standard Gazebo environment

---

## 🧠 Theory: The ALTBD Approach
The core innovation of the ALTBD method is how it identifies a "breakpoint"—the physical point where a straight wall ends and a corner or obstacle begins.

1. **Prediction:** The algorithm uses linear regression on previous scan points to predict where the next LiDAR point should land if the wall continues in a straight line.
2. **The Elliptical Boundary:** Instead of a simple circular threshold, ALTBD draws an elliptical detection area around the predicted point. The minor axis limit ($hmax_{n+1}$) adapts dynamically based on the sensor's measurement error percentage.



3. **Mathematical Evaluation:** The new scan point is transformed into the origin coordinates $(k_{n+1}, j_{n+1})$. It is then evaluated against the ellipse equation:

$$e\_result = \frac{k_{n+1}^2}{qmax_{n+1}} + \frac{j_{n+1}^2}{hmax_{n+1}}$$

If $e\_result > 1$, the point falls outside the predicted elliptical boundary, confirming that a breakpoint has been detected.

---

## 💻 Lab Instructions
Your primary objective is to complete the `altbd_node.py` script. The node must subscribe to the `/scan` topic and publish the detected corners as spherical markers via a `visualization_msgs/MarkerArray` to the `/corners` topic.

Locate the `TODO` blocks in the skeleton code and implement the following steps:

* **Task 1: Sensor Noise Profiling:** Calculate the percentage error of previous points relative to their distance from the sensor ($r_n$). Determine the average error percentage to set the minor axis limit ($hmax_{n+1}$).
* **Task 2: Predictive Line Tracking:** Extract three starting points $(p_{n-2}, p_{n-1}, p_n)$ and use linear regression to generate the predictive line equation $F^{\prime}_{n}(x)$.
* **Task 3: The Elliptical Boundary Check:** Transform the incoming data point into the new coordinate system to find $k_{n+1}$ and $j_{n+1}$. Compute the $e\_result$ equation. 
* **Task 4: Corner Filtering:** If $e\_result > 1$, verify if the detected position falls within the defined corner tolerance limit ($Dth_{corner}$) to classify it as a true room corner. Convert valid corners into RViz markers.

---

## 🧪 Testing and Evaluation
Once your node is complete, launch the simulation arena. Use a square-shaped room layout, similar to the 2.4 m x 2.4 m multi-room setup used in standard robotics competitions.



Evaluate your algorithm against the following benchmarks:
1. **Computational Speed:** Measure the average execution time of your breakpoint detection logic. A properly optimized ALTBD implementation should process a scan in approximately **101 ms**, making it significantly faster than the ~175 ms average of the ABD+IEPF method.
2. **Corner Detection Accuracy:** Monitor RViz and count any false corner detections (markers placed where no physical corner exists). The ALTBD algorithm is proven to have a very low error rate (e.g., 2 errors out of 43 samples), whereas older methods struggle heavily with square corners.

---

## 📦 Deliverables
1. **Source Code:** Push your completed `altbd_node.py` to your assigned GitHub repository.
2. **Lab Report:** Submit a brief Markdown document containing your recorded computation speeds, accuracy observations, and a screenshot of your markers rendering successfully in RViz.
