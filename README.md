\# Automotive Cruise Control — Model-Based Control



\## Professional V2



A model-based automotive longitudinal control project developed and validated in \*\*MATLAB/Simulink\*\* using classical and modern control strategies.



> \*\*Engineering workflow:\*\* Problem → Modeling → Controller Design → Sensitivity Analysis → Validation → Engineering Decision



\---



\## 1. Project Overview



This project develops a closed-loop automotive cruise control system for regulating vehicle speed to a desired reference.



The project progresses from a simple proportional controller to PI, filtered PID and LQR control while introducing:



\* Mathematical plant modeling

\* Classical controller design

\* State-space modeling

\* Actuator dynamics

\* Controller sensitivity analysis

\* Control-effort analysis

\* Actuator saturation

\* MATLAB/Simulink cross-validation

\* Engineering decision documentation



The objective is not only to design controllers, but to demonstrate a \*\*complete model-based control engineering workflow\*\*.



\---



\## 2. System Architecture



```text

Speed Reference

&#x20;     │

&#x20;     ▼

┌───────────────┐

│   Controller  │

│ P / PI / PID  │

│      / LQR    │

└───────┬───────┘

&#x20;       │

&#x20;       ▼

┌──────────────────┐

│ Actuator Dynamics│

│    τₐ = 0.5 s    │

└────────┬─────────┘

&#x20;        │

&#x20;        ▼

┌──────────────────┐

│ Vehicle Dynamics │

│ m dv/dt + bv = u │

└────────┬─────────┘

&#x20;        │

&#x20;        ▼

&#x20;   Vehicle Speed

&#x20;        │

&#x20;        └────────────── Feedback

```



\---



\## 3. Vehicle Model



The baseline longitudinal vehicle model is:



$$

m\\frac{dv}{dt}+bv=u

$$



with transfer function:



$$

G(s)=\\frac{1}{ms+b}

$$



\### Parameters



| Parameter              |     Value |

| ---------------------- | --------: |

| Vehicle mass           |   1000 kg |

| Drag coefficient       |   50 Ns/m |

| Actuator time constant |     0.5 s |

| Maximum actuator force |     700 N |

| Gravity                | 9.81 m/s² |



The LQR model additionally includes actuator dynamics to make the control design more representative of a physical system.



\---



\# 4. Controller Development



\## Phase 01 — P Controller



The P controller establishes the baseline closed-loop response.



\### Selected Controller



```text

Kp = 500

```



\### Performance



| Metric             |   Result |

| ------------------ | -------: |

| Rise Time          | 3.9946 s |

| Settling Time      | 7.1129 s |

| Overshoot          |      0 % |

| Steady-State Error |   0.0909 |

| Closed-loop Pole   |    -0.55 |



\### Gain Sensitivity



The proportional gain was evaluated over:



```text

Kp = \[50 100 200 500 1000 2000]

```



This established the relationship between controller gain, transient response and control effort.



\### Evidence



!\[P Controller Response](Results/Figures/P\_Controller/ClosedLoop\_Response.png)



!\[P Gain Comparison](Results/Figures/P\_Controller/Kp\_Comparison.png)



!\[P Sensitivity](Results/Figures/P\_Controller/SettlingTime\_vs\_Kp.png)



\---



\# 5. Phase 02 — PI Controller



Integral action was introduced to reduce the steady-state tracking error of the P controller.



\### Selected Controller



```text

Kp = 500

Ki = 20

```



\### Performance



| Metric             |           Result |

| ------------------ | ---------------: |

| Rise Time          |         4.6538 s |

| Settling Time      |        10.2954 s |

| Steady-State Error |         0.000459 |

| Closed-loop Poles  | -0.5108, -0.0392 |



The integral gain was evaluated over:



```text

Ki = \[5 10 20 40 80]

```



Higher integral gain improved tracking but increased transient aggressiveness and, at higher values, introduced overshoot.



\### Evidence



!\[PI Response](Results/Figures/PI\_Controller/PI\_Final\_Response.png)



!\[P vs PI](Results/Figures/PI\_Controller/P\_vs\_PI\_Response\_Comparison.png)



!\[PI Pole Migration](Results/Figures/PI\_Controller/PI\_PoleMigration\_Analysis.png)



!\[PI Sensitivity](Results/Figures/PI\_Controller/SettlingTime\_vs\_Ki.png)



\---



\# 6. Phase 03 — Filtered PID Controller



Derivative action was introduced to improve transient performance.



A filtered derivative was used rather than an ideal derivative:



```text

Kp = 500

Ki = 30

Kd = 20

N  = 10

```



The derivative contribution is filtered to reduce sensitivity to high-frequency measurement noise.



\### Final Performance



| Metric             |     Result |

| ------------------ | ---------: |

| Rise Time          |   ≈ 4.26 s |

| Settling Time      |   ≈ 6.87 s |

| Overshoot          |   ≈ 1.03 % |

| Steady-State Error |        ≈ 0 |

| Control RMS        | ≈ 169.94 N |

| Control Peak       |      700 N |



A systematic gain search was also performed over candidate combinations of:



```text

Kp = \[400 500 600]

Ki = \[10 20 30]

Kd = \[0 10 20]

```



The final PID configuration satisfies the project-level stability, settling-time, overshoot and tracking requirements.



\### Evidence



!\[PID MATLAB vs Simulink](Results/Figures/PID\_Controller/PID\_MATLAB\_vs\_Simulink.png)



!\[PID Validation Error](Results/Figures/PID\_Controller/PID\_Validation\_Error.png)



\---



\# 7. Phase 04 — LQR Controller



LQR was introduced as a modern state-space control benchmark.



The augmented state vector is:



$$

x=

\\begin{bmatrix}

v\\\\

F\_a

\\end{bmatrix}

$$



where:



\* \\(v\\) = vehicle speed

\* \\(F\_a\\) = actuator force



The LQR controller minimizes:



$$

J=\\int\_0^\\infty(x^TQx+u^TRu)\\,dt

$$



\### Baseline Weighting



```text

Q = \[1 0;

&#x20;    0 0]



R = 2.0408e-6

```



\### Controller Gain



```text

K = \[637.7980   0.2798]

```



\### Closed-Loop Poles



```text

\-0.7581

\-1.8514

```



\### Controllability



```text

Rank = 2 / 2

```



The augmented model is therefore fully controllable.



\### Baseline Performance



| Metric        |     Result |

| ------------- | ---------: |

| Rise Time     |   3.2642 s |

| Settling Time |   5.8520 s |

| Overshoot     |        0 % |

| Final Speed   | 1.0000 m/s |

| SSE           |          0 |



\### Evidence



!\[LQR Response](Results/Figures/LQR\_Controller/LQR\_Baseline\_Response.png)



!\[LQR Pole Map](Results/Figures/LQR\_Controller/LQR\_Baseline\_PoleMap.png)



!\[LQR Control Effort](Results/Figures/LQR\_Controller/LQR\_Baseline\_ControlEffort.png)



\---



\# 8. LQR Sensitivity Analysis



The effect of relative state and control weighting was evaluated systematically.



| Case                   |   Q/R | Rise Time | Settling Time | Control Peak | Feasible |

| ---------------------- | ----: | --------: | ------------: | -----------: | :------: |

| Low State Priority     | 0.500 |  4.5196 s |      8.2026 s |     497.50 N |     ✓    |

| Baseline               | 1.000 |  3.2658 s |      5.8542 s |     701.79 N |     ✗    |

| Higher State Priority  | 2.000 |  2.3933 s |      4.1625 s |     991.22 N |     ✗    |

| Higher Control Penalty | 0.500 |  4.5196 s |      8.2026 s |     497.50 N |     ✓    |

| Strong Control Penalty | 0.200 |  7.0000 s |     12.7366 s |     317.02 N |     ✓    |



\### Engineering Interpretation



Increasing state priority improves speed of response but demands greater actuator effort.



Increasing control penalty reduces actuator demand but sacrifices transient performance.



Therefore, \*\*Q/R selection cannot be based on performance alone\*\*. Actuator capability must be treated as a hard engineering constraint.



!\[LQR Sensitivity — Settling Time](Results/Figures/LQR\_Controller/LQR\_Sensitivity\_SettlingTime.png)



!\[LQR Sensitivity — Control Peak](Results/Figures/LQR\_Controller/LQR\_Sensitivity\_ControlPeak.png)



!\[LQR Sensitivity — Control RMS](Results/Figures/LQR\_Controller/LQR\_Sensitivity\_ControlRMS.png)



\---



\# 9. Actuator Constraint Validation



The unconstrained LQR controller demands:



```text

Raw control peak = 701.786235 N

```



The actuator is limited to:



```text

Maximum actuator force = 700 N

```



Therefore the actual actuator output is saturated at:



```text

Actual peak = 700 N

```



\### Saturation Results



| Metric                |       Result |

| --------------------- | -----------: |

| Raw control peak      | 701.786235 N |

| Actual control peak   | 700.000000 N |

| Actuator limit        | 700.000000 N |

| Saturated samples     |   5 / 120001 |

| Saturation percentage |     0.0042 % |

| Closed-loop stability |         PASS |

| Tracking              |         PASS |

| Actuator output       |         PASS |



The saturation event is small and localized, but it is explicitly retained rather than hidden.



\### Engineering Decision



The LQR controller is accepted because the actuator constraint is respected in the implemented system and the saturation duration is negligible.



!\[LQR Control Validation](Results/Figures/LQR\_Controller/LQR\_Control\_Validation.png)



\---



\# 10. MATLAB ↔ Simulink Cross-Validation



An important project requirement was independent validation between MATLAB analytical simulation and Simulink implementation.



\### P Controller



```text

Maximum error = 2.306e-06

RMSE          = 4.928e-07

Status        = PASS

```



\### PI Controller



```text

Maximum error = 2.096e-06

RMSE          = 2.100e-07

Status        = PASS

```



The close agreement demonstrates that the MATLAB implementation and Simulink model represent the same intended control architecture.



!\[MATLAB vs Simulink — P](Results/Figures/P\_Controller/MATLAB\_vs\_Simulink.png)



!\[MATLAB vs Simulink — PID](Results/Figures/PID\_Controller/PID\_MATLAB\_vs\_Simulink.png)



\---



\# 11. Controller Comparison



| Controller | Main Purpose                    | Key Engineering Characteristic                        |

| ---------- | ------------------------------- | ----------------------------------------------------- |

| P          | Baseline                        | Simple, stable, but non-zero SSE                      |

| PI         | Tracking improvement            | Eliminates steady-state error                         |

| PID        | Classical performance benchmark | Improved transient response with filtered derivative  |

| LQR        | Modern control benchmark        | State feedback with explicit control-effort trade-off |



The project does \*\*not\*\* claim that one controller is universally optimal.



Instead, each controller demonstrates a different engineering design principle.



\---



\# 12. Engineering Workflow



```text

Physical Problem

&#x20;     ↓

Mathematical Model

&#x20;     ↓

P Controller Baseline

&#x20;     ↓

Sensitivity Analysis

&#x20;     ↓

PI Controller

&#x20;     ↓

PID + Derivative Filtering

&#x20;     ↓

State-Space Model

&#x20;     ↓

LQR Design

&#x20;     ↓

Q/R Sensitivity

&#x20;     ↓

Actuator Constraint

&#x20;     ↓

MATLAB Validation

&#x20;     ↓

Simulink Validation

&#x20;     ↓

Engineering Decision

```



\---



\# 13. Repository Structure



```text

Automotive-Cruise-Control-Model-Based-Control

│

├── Documentation

│   ├── Design\_Decisions.md.txt

│   └── Engineering\_Notes.md.txt

│

├── MATLAB

│   ├── Analysis

│   ├── Controllers

│   ├── Models

│   ├── Reports

│   ├── Utilities

│   └── Validation

│

├── Results

│   ├── Data

│   └── Figures

│

├── Simulink

│   ├── CruiseControl\_P\_Controller.slx

│   ├── CruiseControl\_PI\_Controller.slx

│   ├── CruiseControl\_PID\_Controller.slx

│   └── CruiseControl\_LQR\_Controller.slx

│

├── .gitignore

├── LICENSE

└── README.md

```



\---



\# 14. Engineering Skills Demonstrated



\### Control Engineering



\* Classical feedback control

\* P / PI / PID controller design

\* Derivative filtering

\* State-space modeling

\* LQR optimal control

\* Pole analysis

\* Controllability

\* Sensitivity analysis

\* Actuator saturation



\### Model-Based Development



\* MATLAB modeling

\* Simulink implementation

\* Parameterized models

\* Modular controller architecture

\* Automated validation

\* MATLAB/Simulink cross-validation



\### Engineering Validation



\* Performance metrics

\* Gain sensitivity

\* Control-effort evaluation

\* Constraint validation

\* Numerical error analysis

\* Evidence-based controller selection



\### Software / Engineering Practice



\* Modular MATLAB architecture

\* Git version control

\* Structured project repository

\* Engineering documentation

\* Reproducible analysis workflow



\---



\# 15. Engineering Lessons



\### 1. Performance alone is not enough



A controller with faster response may demand actuator effort that the physical system cannot provide.



\### 2. Integral action solves tracking error but changes transient behavior



Increasing integral gain improves steady-state accuracy but can increase overshoot and settling time.



\### 3. Derivative action must be implemented carefully



Filtering is required for a practical derivative implementation.



\### 4. LQR is a trade-off, not a magic optimum



The Q/R weighting determines the balance between tracking performance and control effort.



\### 5. Constraints must be validated at the actuator level



The raw controller command and actual actuator output must be distinguished.



\### 6. Simulation models require independent validation



Agreement between MATLAB and Simulink provides confidence that the implemented model represents the intended design.



\---



\# 16. Final Engineering Decision



The final project retains four controller stages:



```text

P → PI → Filtered PID → LQR

```



P, PI and PID provide the classical-control development path.



LQR provides the modern state-space benchmark and demonstrates explicit performance-versus-control-effort optimization.



The final LQR implementation is validated with actuator saturation and passes the defined project-level validation criteria.



\---



\# 17. Project Status



\## Completed — PROFESSIONAL V2



The objective is to demonstrate a complete and defensible \*\*model-based automotive control engineering workflow\*\*

\---



\## Author



\*\*Yash Khiste\*\*



M.Sc. Electrical Engineering \& Information Technology



Focus:



\*\*Control Systems | Model-Based Development | Simulation | Control Algorithms\*\*



Target Engineering Areas:



\* Automotive Control Systems

\* Model-Based Development

\* Control Algorithm Development

\* Simulation \& Validation

\* State Estimation



