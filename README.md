# 🚁 Drone Simulation for Dataset Acquisition (CARLA + ROS)

🎓 *University Project – Robotics 2 Course, Politecnico di Bari*

A **drone-based image and video acquisition system** developed in the **CARLA simulator**, integrated with **ROS** and **Docker** for managing urban simulation environments.  
This project explores autonomous UAV operations in complex city scenarios for repeatable dataset generation.

---

## 🌆 Overview

The project focuses on using the **CARLA simulator** to capture **images and videos** through a **drone equipped with a downward-facing camera** flying over urban intersections.  
By setting a fixed seed, the simulator ensures that the same scene, including vehicles and pedestrians, is **repeatable**, allowing consistent dataset generation.

<p align="center">
  <img src="images/rondabout/clear_sunrise.jpg" alt="Clear sunrise scene" width="500">
</p>

---

## 🧰 Technologies

- ![CARLA](https://img.shields.io/badge/CARLA-Simulator-lightgrey?style=flat-square&logo=unrealengine)
- ![ROS](https://img.shields.io/badge/ROS1-Integration-blue?style=flat-square&logo=ros)
- ![Python](https://img.shields.io/badge/Python-Scripting-yellow?style=flat-square&logo=python)
- ![Docker](https://img.shields.io/badge/Docker-Containerization-2496ED?style=flat-square&logo=docker)
- ![SSH](https://img.shields.io/badge/SSH-Remote%20Access-orange?style=flat-square&logo=gnu-bash)
- ![Windows Remote Desktop](https://img.shields.io/badge/Windows-Remote%20Desktop-lightblue?style=flat-square&logo=windows)

---

## ⚙️ System Description

The **simulation environment** is built using CARLA’s **Unreal Engine-based** infrastructure, which enables realistic modeling of city intersections populated by cars and pedestrians.  
The workflow consists of several stages:

1. **Environment Configuration** – Setting up urban intersections and environmental conditions.  
2. **Scenario Population** – Using Python scripts to spawn vehicles and pedestrians.  
3. **Drone Control** – Managing the drone’s flight path and altitude via ROS topics.  
4. **Data Recording** – Capturing synchronized **images and videos** through ROS and Python APIs.  

<p align="center">
  <img src="images/four_way_junction/heavy_rain_night.jpg" alt="Heavy rain night scene" width="500">
</p>

All simulation tasks were executed on a **remote server** through **SSH** and **Windows Remote Desktop**, ensuring efficient management of computation-heavy processes.

---

## 🧪 Results

The system allows the drone to fly autonomously across **different intersections** and **weather conditions**, enabling dataset acquisition under a wide variety of realistic scenarios.

<p align="center">
  <img src="images/five_way_junction/foggy_noon.jpg" alt="Foggy noon scene" width="500">
</p>

Captured data was used for further **computer vision and robotics applications**, such as obstacle detection, object tracking, and autonomous navigation research.

---

## 📁 Repository Structure

📦 uav_dataset_simulation  
- scripts/ # Python control and acquisition scripts  
- config/ # Configuration and environment setup files  
- images/ # Captured simulation scenes (clear, foggy, rainy)  
- docs/ # Project documentation  
- README.md # Project overview  

---

## 👥 Authors

Developed by [Francesco Savino](https://github.com/FrankSav80) and [Tommaso Savino](https://github.com/ItsTomSav)  
Master’s Degree in Automation and Robotics Engineering – Politecnico di Bari

---

## 🧠 Keywords

`CARLA` • `ROS` • `Drone Simulation` • `Dataset Acquisition` • `Python` • `Docker` • `Robotics 2`

