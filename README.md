# 🚀 Perception Lab: Corner Extraction and Room Geometry using ALTBD

## 📖 Overview

Welcome to the Autonomous Racing Perception Lab. In this module, you will implement a state-of-the-art perception algorithm designed for high-speed, indoor mobile robots: the **Adaptive Line Tracking Breakpoint Detector (ALTBD)**.

Accurate room geometry and opponent detection rely on finding "breakpoints" (the exact coordinates where a continuous wall ends or an obstacle begins). While traditional methods like Line Tracking (LT) use rigid thresholds, and the Adaptive Breakpoint Detector (ABD) requires computationally heavy recursive algorithms like Iterative End Point Fit (IEPF), ALTBD solves this in linear $O(n)$ time using predictive linear regression and dynamically scaling elliptical boundaries.

## 🎯 Core Objectives

By completing this lab, you will:

* Process raw 2D LiDAR polar arrays into Cartesian spatial maps.
* Implement localized linear regression to continuously predict wall trajectories.
* Apply 2D rigid body affine transformations to shift coordinate frames in real-time.
* Evaluate spatial points using dynamically adapting elliptical threshold mathematics.

## ⚙️ System Dependencies

Ensure your environment meets the following requirements before starting:

* **Operating System:** Ubuntu 20.04 or 22.04
* **ROS 2:** Foxy Fitzroy or Humble Hawksbill
* **Python 3:** With `numpy` and `math` libraries installed
* **Simulator:** F1TENTH Gym environment or a standard Gazebo multi-room simulation setup.

## 🧠 Theoretical Background

The ALTBD algorithm operates on a sliding window of LiDAR data points to determine if a newly scanned point belongs to the current wall, or if it represents a breakpoint.

1. **Predictive Foundation:** The algorithm takes the three most recent validated points and calculates a predictive linear regression line $F^{\prime}_{n}(x)$.

2. **Adaptive Boundary:** It establishes an elliptical detection zone around the predicted next point. The minor axis ($hmax$) expands or contracts dynamically based on the average LiDAR measurement error percentage ($\sigma_n$) combined with a static offset ($\sigma_{offset}$).

3. **Mathematical Evaluation:** The incoming LiDAR point is transformed into a local coordinate frame $(k_{n+1},j_{n+1})$ where the predicted point acts as the origin $(0,0)$. The point is evaluated against the ellipse equation:

$$\frac{k_{n+1}^2}{qmax_{n+1}}+\frac{j_{n+1}^2}{hmax_{n+1}}$$

If the resulting scalar is $>1.0$, the point sits outside the ellipse, breaking the continuity of the line, and is flagged as a breakpoint.

## 🛠️ Lab Instructions (Your Tasks)

Your primary objective is to complete the `altbd_node.py` script located in the `detection/scripts/` directory. You will find 9 specific `TODO` blocks within the `detect_breakpoints()` function.

* **Tasks 1-3 (Initialization):** Extract the $X$ and $Y$ arrays from your point buffer. Use `np.polyfit` to establish your prediction line. *Hint: Be sure to handle near-vertical walls by swapping your axes to avoid singular matrix errors.* Calculate the orthogonal geometric error of your starting points to establish your baseline noise floor.
* **Tasks 4-5 (Drawing the Ellipse):** Calculate the minor axis ($hmax$) using your rolling error. Use trigonometric ray intersection to determine exactly where the next laser pulse should strike the wall to find your major axis ($qmax$).
* **Task 6-7 (The Interrogation):** Translate the incoming data point to your new origin and rotate it using $\sin\theta$ and $\cos\theta$ to align with the x-axis. Run the transformed point through the ellipse equation.
* **Tasks 8-9 (Classification):** If a breakpoint is triggered, use the physical distance tolerance ($Dth_{corner}$) to classify whether it is a true room corner or just a wall edge.

## 🚀 Running the Code

1. **Build your workspace:**

```bash
cd ~/ros2_ws
colcon build --packages-select detection
source install/setup.bash
```

2. **Launch your simulator** (e.g., your F1TENTH or custom multi-room arena).
3. **Run the perception node:**
```bash
ros2 run detection altbd_node.py
```

4. **Visualize in RViz:** Open RViz2, add a `MarkerArray` display, and subscribe it to the `/corners` topic. You should see red spheres populating accurately on the room corners.

## 📊 Benchmarks and Deliverables

Historically, the legacy ABD+IEPF pipeline requires roughly 175 ms to process a scan and yields high error rates in square rooms. By successfully vectorizing your math and implementing the ALTBD ellipse, your node should process a 1080-point LiDAR scan in approximately 101 ms, while drastically reducing corner detection errors.

**Submission Requirements:**

1. Push your fully resolved `altbd_node.py` file to your lab repository.
2. Submit a brief Lab Report containing:
* Average computation speeds (use ROS 2 timing or Python's `time` module).
* Observations on detection accuracy at varying speeds/distances.
* Screenshots of the detected markers effectively mapping the corners in RViz.
