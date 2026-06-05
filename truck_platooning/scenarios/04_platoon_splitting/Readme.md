
# Autonomous Systems Lab 1 – Truck Platooning Scenario: Platoon Splitting

## Student Information

**Name:** Stephen Uzochi Obuh
**Matriculation Number:** 2210044
**Course:** Autonomous Systems Lab 1
**Group:** 4

---

## Project Title

### Truck Platooning Scenario: Platoon Splitting

---

## Scenario Overview

This project focuses on a **Truck Platooning Platoon Splitting** scenario involving one lead truck and three follower trucks operating as a coordinated platoon.

As the platoon approaches a route divergence point, the convoy must split into two independent sub-platoons while maintaining safety, stability, and communication efficiency. The lead truck driver initiates a manual override of the Vehicle-to-Vehicle (V2V) communication system to begin the platoon split operation.

After separation:

* The original lead truck continues leading one sub-platoon.
* One of the follower trucks is promoted to lead the second sub-platoon.
* V2V communication is re-established within each newly formed platoon.
* Inter-vehicle gaps are reconfigured and synchronized to ensure safe and efficient operation.

---

## Objectives

The platoon splitting system must be capable of:

1. Detecting an upcoming route divergence.
2. Coordinating the separation process.
3. Creating safe inter-vehicle gaps.
4. Splitting control groups into independent platoons.
5. Stabilizing the resulting sub-platoons.

---

## Platoon Configuration

### Initial Platoon

```
Lead Truck → Follower 1 → Follower 2 → Follower 3
```

### After Split

```
Sub-Platoon A:
Lead Truck → Follower 1

Sub-Platoon B:
New Lead Truck (Follower 2 or Follower 3) → Remaining Follower
```

---

## Operational Phases

The platoon splitting process is divided into six phases:

| Phase | Description                        | Typical Timing Characteristics      |
| ----- | ---------------------------------- | ----------------------------------- |
| 1     | Route Divergence Detection         | Seconds to minutes before split     |
| 2     | Split Negotiation and Coordination | 100 ms – several seconds            |
| 3     | Gap Creation                       | 2 – 10 seconds                      |
| 4     | Physical Separation                | 1 – 5 seconds                       |
| 5     | Topology Reconfiguration           | Less than 1 second to a few seconds |
| 6     | Stabilization of Sub-Platoons      | 5 – 20 seconds                      |

---

## Phase Descriptions

### Phase 1: Route Divergence Detection

The platoon identifies an upcoming road divergence using route planning and navigation data. This phase provides sufficient time to prepare for the split maneuver.

### Phase 2: Split Negotiation and Coordination

Vehicles exchange V2V messages to determine:

* Which truck will become the new platoon leader.
* The target split point.
* Timing and synchronization requirements.

### Phase 3: Gap Creation

The vehicles adjust their longitudinal spacing to create a safe separation gap between the two future sub-platoons.

### Phase 4: Physical Separation

The trucks execute the planned maneuver and physically separate onto different routes.

### Phase 5: Topology Reconfiguration

The communication network is restructured:

* Leadership roles are reassigned.
* New platoon memberships are established.
* Internal V2V communication links are restored.

### Phase 6: Stabilization of Sub-Platoons

Both platoons independently stabilize:

* Desired spacing is achieved.
* Vehicle speeds are synchronized.
* Platoon control algorithms return to steady-state operation.

---

## Communication Requirements

### Before Split

* Single V2V communication network.
* One lead truck controls three follower trucks.

### During Split

* Temporary communication restructuring.
* Coordination messages exchanged between all trucks.

### After Split

* Two independent V2V communication groups.
* Each sub-platoon operates with its own leader and followers.

---

## Expected Outcome

The platoon splitting maneuver should result in:

* Safe separation of vehicles.
* Successful leader reassignment.
* Reliable V2V communication within each sub-platoon.
* Stable platoon formation after route divergence.
* Minimal disruption to traffic flow and vehicle safety.

---


