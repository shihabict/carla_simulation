## IDM Car-Following Simulation in CARLA

![CARLA Simulation Demo](demo.gif) *(Example GIF of the simulation)*

## Overview
This project implements an **Intelligent Driver Model (IDM)** car-following simulation using the CARLA simulator. The simulation features:
- Leader vehicle (Tesla Model 3) with CARLA's autopilot
- Follower vehicle (Audi TT) with custom IDM controller
- Physics tuning for smooth following behavior
- Data logging for performance analysis

## Prerequisites
- CARLA 0.9.15+ ([Download](https://github.com/carla-simulator/carla/blob/master/Docs/download.md)
- Python 3.7 or 3.9
- Follow this document to install CARLA ([CARLA Installation Document](https://drive.google.com/file/d/10hNyc_oCqnkp902S2kJs19bu8Eurxxes/view?usp=sharing)

## File Structure
```bash 
carla_simulation/
├──carla_idm_simulation/
    ├── main.py
    ├── follower_controller.py
    ├── idm_controller.py
    ├── data_logger.py
    └── camera_setup.py
├── .gitignore
└── logs/
```
## How to Run
1. Launch CARLA server:
```bash 
./CarlaUE4.sh
 ```
2. Create Virtual Environment and install requirements:
```bash 
    python3.9 -m vevn carlaenv
    source carlaenv/bin/activate
    pip install -r requirements.txt
```
3. Run the simulation:
```bash 
python carla_idm_simulation/main.py
```

## Key Features
Parameters in `idm_controller.py`
```bash 
desired_gap = 10.0    # Target following distance (m)
max_accel = 1.0       # Maximum acceleration (m/s²)
desired_speed = 15.0  # Cruise speed (m/s)
```
## Physic Tuning
Optimized vehicle dynamics in `main.py`
```bash 
physics_control.linear_damping = 0.3    # Reduce forward/backward bounce
physics_control.angular_damping = 0.7   # Reduce steering wobble
wheel.damping_rate = 2.0                # Smoother suspension
```
## Data Logging
Outputs to `logs/car_following_data.csv` with:
- Timestamps
- Positions (x,y,z)
- Speeds
- Control inputs (throttle/brake/steer)
- Inter-vehicle distance
- Relative speed