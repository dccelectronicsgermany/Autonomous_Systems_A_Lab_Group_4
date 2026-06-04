# Emergency Braking Scenario

## Scenario Information

| Field             | Value                                       |
| ----------------- | ------------------------------------------- |
| **Scenario Name** | Emergency Braking                           |
| **Project**       | Truck Platooning — Autonomous Systems A Lab |
| **Creator**       | Riwaj Ghimire                               |
| **Version**       | 1.0                                         |
| **Date**          | June 4, 2026                                |
| **Scenario ID**   | SC-EB-02                                    |

---

## Scenario Description

An emergency braking event occurs when the lead truck detects an obstacle or hazard that requires an immediate full stop. The lead truck applies maximum braking and simultaneously broadcasts an emergency brake signal to all following trucks via V2V (Vehicle-to-Vehicle) communication.

Each follower truck receives the signal and applies emergency braking without waiting for a human reaction, allowing the entire platoon to stop in a coordinated cascade. If V2V communication is unavailable, follower trucks fall back to onboard sensor-based collision avoidance.

---

## Objective

Ensure that all trucks in the platoon perform a safe, coordinated emergency stop by propagating brake signals via V2V faster than the physical reaction chain alone, preventing rear-end collisions.

---

## System Components

### Lead Truck

Responsible for:

* Detecting obstacles and hazards using onboard sensors
* Applying maximum braking force
* Broadcasting the emergency brake signal via V2V

### Follower Trucks

Responsible for:

* Receiving and processing emergency brake signals
* Applying brakes immediately upon signal reception
* Sending braking acknowledgement to the Platoon Manager

### V2V Communication System

Responsible for:

* Broadcasting the emergency brake signal to all platoon members
* Forwarding braking status and acknowledgements
* Ensuring low-latency delivery of safety-critical messages

### Platoon Manager

Responsible for:

* Monitoring overall platoon braking status
* Confirming all trucks have stopped
* Broadcasting standby mode and logging the event

---

## Preconditions

* At least two trucks are operating in an active platoon.
* V2V communication links between all trucks are active.
* All trucks are travelling at normal platoon cruising speed.
* Onboard sensors on the lead truck are operational.

---

## Trigger Events

One of the following events triggers this scenario:

* Lead truck sensor detects an obstacle within critical stopping distance
* A follower truck detects an imminent collision with the truck ahead
* Manual emergency brake activation by a driver or operator

---

## Main Flow

1. Lead truck sensors detect an obstacle or hazard.
2. Lead truck applies full emergency braking.
3. Lead truck broadcasts `EMERGENCY_BRAKE_SIGNAL` via V2V.
4. All follower trucks receive the signal and apply emergency braking immediately.
5. Each follower truck sends a braking acknowledgement to the Platoon Manager.
6. All trucks decelerate and come to a complete stop.
7. Platoon Manager confirms all trucks stopped and broadcasts `STANDBY`.
8. System enters post-emergency standby mode and logs the event.

---

## Alternative Flows

### AF1 — V2V Communication Unavailable

1. Lead truck initiates emergency braking but V2V signal cannot be delivered.
2. Follower trucks detect deceleration of the truck ahead via onboard sensors.
3. Each follower truck activates sensor-based collision avoidance braking independently.
4. Platoon stops safely with increased stopping distances.

### AF2 — False Positive Detection

1. Lead truck sensors trigger an emergency brake signal.
2. All trucks apply emergency braking.
3. No actual obstacle is present.
4. Trucks come to a stop and the system performs a diagnostic check.
5. Platoon resumes normal operation after verification.

---

## Exception Flows

### EF1 — Braking System Failure on a Follower Truck

1. Emergency brake signal is received but one follower truck fails to respond.
2. The affected truck broadcasts a braking failure alert.
3. Surrounding trucks increase deceleration to compensate.
4. A maintenance alert is issued and the incident is logged.

---

## Postconditions

### Success Condition

* All trucks have stopped safely without inter-vehicle collision.
* Emergency event is logged by the Platoon Manager.
* System is in standby mode awaiting operator decision.

### Failure Condition

* One or more trucks failed to brake in time.
* Maintenance alert is issued and the platoon is dissolved.

---

## Functional Requirements

* Detect obstacles via onboard sensors and trigger emergency braking
* Broadcast emergency brake signal to all platoon members via V2V
* Apply emergency braking on all follower trucks upon signal reception
* Acknowledge signal receipt and report braking status to Platoon Manager
* Fall back to sensor-based collision avoidance if V2V is unavailable
* Log all emergency braking events with timestamps and truck states
* Transition to standby mode after full platoon stop

---

## Safety Requirements

| ID           | Requirement                                                                           |
| ------------ | ------------------------------------------------------------------------------------- |
| **SR-EB-01** | The system shall detect a hazard and initiate braking within 50 ms of sensor detection. |
| **SR-EB-02** | The V2V emergency brake signal shall reach all follower trucks within 100 ms.         |
| **SR-EB-03** | All follower trucks shall begin braking within 150 ms of receiving the signal.        |
| **SR-EB-04** | Sensor-based fallback braking shall activate within 200 ms if V2V is unavailable.    |
| **SR-EB-05** | No truck shall exceed the maximum deceleration rate that could cause jackknifing.     |
| **SR-EB-06** | A braking failure on one truck shall not cascade into a chain-reaction collision.     |

---

## Timing Constraints

| Requirement                            | Constraint   |
| -------------------------------------- | ------------ |
| Hazard detection to brake activation   | <= 50 ms     |
| Emergency signal broadcast latency     | <= 100 ms    |
| Follower brake activation after signal | <= 150 ms    |
| Sensor-based fallback activation       | <= 200 ms    |
| Full platoon stop from signal          | <= 5 seconds |

---

## Sequence Flow

1. Lead truck detects obstacle and applies emergency brake.
2. Lead truck broadcasts `EMERGENCY_BRAKE_SIGNAL` via V2V.
3. Follower trucks receive signal and apply emergency braking.
4. Follower trucks send braking acknowledgement.
5. All trucks report `STOPPED` status.
6. Platoon Manager confirms stop and broadcasts `STANDBY`.
7. All trucks enter standby mode.

---

## SysML Models Included

* Sequence Diagram — `emergency_braking_sequence.puml`
* State Machine Diagram — `emergency_braking_state_machine.puml`

---

## Notes

* V2V communication eliminates human reaction delay, significantly reducing stopping distance compared to conventional convoy driving.
* The sensor-based fallback is a degraded mode — safe but with larger stopping distances.
* Safety requirements take precedence over platoon efficiency during any emergency event.
