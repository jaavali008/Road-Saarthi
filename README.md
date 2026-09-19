# Road Saarthi

## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

Road Saarthi is a MATLAB-based prototype for adaptive path planning and collision avoidance for autonomous vehicles operating on unstructured Indian roads.

## Problem Statement

Indian roads often have mixed traffic, unclear road boundaries, sudden obstacles and unpredictable movements. These conditions make it difficult for an autonomous vehicle to follow a fixed path safely.

## Proposed Solution

Road Saarthi continuously monitors the simulated road environment, detects obstacles and adapts the vehicle's path when an obstacle creates a potential collision risk.

The prototype demonstrates a closed-loop process:

**Detect → Assess → Replan → Move**

## Current Prototype

The current MATLAB prototype demonstrates:

- Indian road scenario generation
- Ego vehicle and obstacle simulation
- Simulated sensor-based obstacle detection
- Adaptive lateral path replanning
- Collision avoidance
- Vehicle movement
- Closed-loop simulation

### Simulated Scenario

The scenario includes:

- Ego vehicle
- Pedestrian
- Auto-rickshaw
- Cow
- Pothole

## Technologies Used

- MATLAB
- MATLAB scripting
- Simulated sensor detection
- Reactive path planning
- Closed-loop vehicle simulation

## MATLAB Files

### `scenario_setup.m`

Creates and initializes the road scenario, ego vehicle and obstacles.

### `sensor_config.m`

Configures the simulated sensor detection parameters.

### `freenet_planner.m`

Handles the path planning logic used for obstacle avoidance.

### `closed_loop_sim.m`

Runs the vehicle movement and closed-loop adaptive replanning simulation.

### `.mat` Files

The `.mat` files contain saved scenario, sensor and planner data used by the MATLAB prototype.

## Execution

The current simulation uses:

- **0.1 second simulation timestep**
- **10 Hz simulation decision cycle**
- **101 simulation steps**
- **Approximately 25 microseconds average measured replanning computation time**


## Future Scope

The simulation can be further enhanced with:

- Advanced camera, LiDAR and radar sensor simulation
- Improved sensor fusion
- AI-based object detection
- More accurate motion prediction
- Advanced risk assessment
- Dynamic speed adaptation
- More complex Indian road scenarios
- Testing with different traffic and obstacle conditions


## Results

The prototype successfully demonstrates adaptive path replanning when obstacles are detected in the vehicle's path.

Simulation screenshots and results are available in:

`results- screenshots`

## Demo Video

The complete prototype demonstration is available on YouTube:

**Road Saarthi MATLAB Prototype Demo**

https://youtu.be/Wq_vhhcmRFw

## Project Structure

```text
Road-Saarthi/
│
├── MATLAB/
│   ├── scenario_setup.m
│   ├── sensor_config.m
│   ├── freenet_planner.m
│   ├── closed_loop_sim.m
│   ├── scenario_data.mat
│   ├── sensor_data.mat
│   └── planner_data.mat
README.md
demo_video_link.txt
results
    └── screenshots
