# ML-Based Adaptive PID Tuning for Process Control

## Overview
This project focuses on improving PID controller performance for process control systems using **Genetic Algorithm (GA)-based tuning**.  
The main aim is to overcome the limitations of conventional PID tuning methods such as **high overshoot, oscillations, and poor robustness** in systems with **dead-time and varying conditions**.

The work compares the following controllers:

- Conventional PID
- Adaptive PID
- GA-Optimized Conventional PID (GA-CPID)
- GA-Based Adaptive PID (GA-APID)

The project is implemented and tested in **MATLAB/Simulink**.

---

## Problem Statement
PID controllers are widely used in industrial control because of their simple structure and effectiveness. However, their performance depends heavily on proper tuning of the controller gains:

- **Kp** → Proportional gain
- **Ki** → Integral gain
- **Kd** → Derivative gain

Traditional tuning methods such as **Ziegler-Nichols (ZN)** are simple, but they often produce:

- High overshoot
- Large oscillations
- Long settling time
- Poor disturbance rejection

To address these issues, this project uses **Genetic Algorithm** to optimize PID parameters and improve the overall system response.

---

## Objectives
The main objectives of this project are:

- To design a PID controller for a dead-time process
- To optimize PID gains using Genetic Algorithm
- To reduce overshoot, oscillations, and settling time
- To minimize error indices such as **IAE** and **ITAE**
- To compare the performance of different PID tuning approaches
- To study the effectiveness of GA for adaptive PID tuning

---


This type of process is challenging to control due to the presence of **time delay**, which reduces stability and makes tuning difficult.

---

## Methodology
The project follows the steps below:

1. Model the process plant using its transfer function
2. Design a conventional PID controller using standard tuning rules
3. Evaluate system response for baseline performance
4. Define performance indices such as:
   - IAE (Integral Absolute Error)
   - ITAE (Integral Time Absolute Error)
   - Overshoot
   - Settling Time
5. Apply **Genetic Algorithm** to optimize PID gains
6. Implement optimized gains in the control loop
7. Compare the results with Adaptive PID and Conventional PID
8. Analyze performance in terms of stability and robustness

---

## Why Genetic Algorithm?
Genetic Algorithm is used in this project because:

- It performs **global optimization**
- It avoids getting trapped in local minima
- It does not require gradient information
- It works well for **nonlinear and dead-time systems**
- It can optimize multiple performance indices together
- It provides better tuning than manual trial-and-error methods

---

## Controllers Compared
### 1. Conventional PID
A fixed-gain PID controller tuned using standard methods.

### 2. Adaptive PID
A PID controller whose gains vary depending on system error or operating condition.

### 3. GA-Optimized Conventional PID (GA-CPID)
A conventional PID controller whose fixed gains are optimized using Genetic Algorithm.

### 4. GA-Based Adaptive PID (GA-APID)
An adaptive PID controller whose parameters are further improved using Genetic Algorithm.

---

## Performance Parameters
The following performance measures are used to evaluate the controllers:

- **Overshoot (%)**
- **Rise Time (s)**
- **Settling Time (s)**
- **Steady-State Error**
- **IAE**
- **ITAE**
- **Disturbance Rejection**
- **Robustness to Delay Variation**

---

## Expected Results
From the simulation study, the expected trend is:

- Conventional PID shows high overshoot and oscillations
- Adaptive PID improves settling and reduces overshoot
- GA-CPID gives better optimized response than conventional PID
- GA-APID provides the best overall performance with:
  - minimum overshoot
  - faster settling time
  - reduced oscillations
  - improved robustness
  - lower IAE and ITAE

---

## Project Structure
Example project folder structure:

```bash
ML-Based-Adaptive-PID-Tuning/
│
├── README.md
├── report/
│   └── project_report.pdf
├── presentation/
│   └── review_ppt.pptx
├── matlab_code/
│   ├── conventional_pid.m
│   ├── adaptive_pid.m
│   ├── ga_cpid.m
│   ├── ga_apid.m
│   ├── objective_function.m
│   └── comparison_plot.m
├── simulink_models/
│   ├── conventional_pid.slx
│   ├── adaptive_pid.slx
│   └── ga_pid_model.slx
└── results/
    ├── conventional_response.png
    ├── adaptive_response.png
    ├── ga_cpid_response.png
    └── ga_apid_response.png
