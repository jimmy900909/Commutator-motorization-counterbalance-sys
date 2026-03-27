
# Active Counterbalance System for Freely-Moving Electrophysiology  
### Real-time Tension Stabilization + 3D Printed Mechanical System + Closed-Loop Firmware
Author:Wei Ching Ling

This repository contains the code, mechanical design files, and system description for an **active counterbalance system** combined with **customized torque-free commutator** that stabilizes tether tension and prevents entangle during freely-moving electrophysiology experiments.  
This system is based on the engineering framework from my master thesis, but **this implementation uses a different commutator model** and a reorganized folder structure.

to use the system, check the manual for the guide
---

https://github.com/user-attachments/assets/3104b9ad-547f-40e5-a1b2-401eba12acd4


## 📌 Project Overview

Tethered electrophysiology often suffers from:
- Mechanical noise from cable tension/torque  
- Slack formation causing entanglement  
- Excessive pulling that may damage the headstage  
- Behavioral interference in freely moving animals  

This project implements an **active counterbalance system** that:
- Measures real-time tension via load cell  
- Runs an event-driven control algorithm (retraction/payout)  
- Uses a motorized pulley to collect and release cable to maintain tension in a safe window  
- Uses 3D printed mechanical components for compact integration  
- Supports different commutator (this version uses: **customized 52 channels slip ring commutator**)  
- Provides reproducible, tunable closed-loop control logic
---

<img src="https://github.com/user-attachments/assets/d7947978-d306-43dd-a18c-0365b9fccbbf" width="400">

## material:
motorization:
- Stepping Motor * 1 (17HS4401)
- A4988 Motor driver
- capacitor 470muF * 1 and 1000muF * 1
- arduino nano * 1 


counterbalance:
- N20 Gearmotor with encoder (100-200RPM) 144 is used in this testing
- DRV8833 motor driver
- arduino nano* 1 or ESP 32 * 1
<img width="3156" height="2125" alt="circuit" src="https://github.com/user-attachments/assets/ed0563bc-b112-487b-9aee-da2bee69ee00" />

## 🗂 Repository Structure
- project-root/
- ├── code/
- │ └── counterblance
- │    └── /nano_version/auto1/ # Main firmware and scripts for nano
- │    └──/esp_version/auto_esp/ # Main firmware and scripts for esp
- │
- │ 
- ├── parts/ #stl. and solidworks model
- │   ├── motorisation/ # model for commutator motorization
- │   └── counterbalance/ # parts and platform for counterbalance system 
- └── README.md



The counterbalance system converts cable tension into a measurable compression force using a **pulley-mediated force-redirection mechanism**.  
The controller:
1. Reads tension (80 Hz)
2. Applies dual-EMA filtering  
3. Detects events (slack, over-tension, impact)
4. Controls motor payout/retraction  
5. Maintains tension in a safe range  


