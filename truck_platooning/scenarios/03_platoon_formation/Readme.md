# Truck Platoon Formation Scenario

This models a **Truck Platoon Formation Scenario** for connected and autonomous vehicles using Vehicle-to-Vehicle (V2V) communication and centralized platoon management. The scenario demonstrates how multiple trucks dynamically form a platoon, coordinate their movements, and transition into normal platoon operation while maintaining safety and communication constraints.

The model is developed using SysML and includes requirements analysis, sequence diagram, timing constraints, and system behaviour specifications.**

## Scenario Description

A lead truck initiates platoon formation by broadcasting a platoon advertisement containing its speed, position, lane, and formation intent.

Follower trucks receive the advertisement and submit join requests containing:

* Truck ID
* Vehicle capabilities
* Current position

The requests are processed by a Platoon Manager, which evaluates available resources and safety policies before approving or rejecting membership.
Once approved, joining vehicles receive:

* Assigned platoon position
* Vehicle role
* Target following gap

The lead truck then initiates a coordinated formation maneuver, allowing follower vehicles to reach their assigned positions. After successful positioning and verification, the system confirms platoon formation and enters normal platoon operation.

## System Components

### Lead Truck (Initiator)

Responsible for:

* Broadcasting platoon advertisements
* Coordinating formation maneuvers
* Providing target speed and lane information

### Follower Trucks (Joiners)

Responsible for:

* Receiving platoon advertisements
* Sending join requests
* Following formation instructions
* Reporting status updates

### V2V Communication System

Responsible for:

* Advertisement distribution
* Join request forwarding
* Status updates
* Formation coordination messages

### Platoon Manager

Responsible for:

* Evaluating join requests
* Checking policies and resources
* Managing platoon membership
* Confirming formation completion

## Functional Requirements

* Detect available platoons
* Process join requests
* Assign platoon positions
* Coordinate formation maneuvers
* Monitor vehicle status
* Confirm successful platoon formation
* Maintain communication between platoon members

## Safety Requirements

* Maintain safe inter-vehicle gaps
* Monitor road conditions continuously
* Validate vehicle capabilities before approval
* Detect communication failures
* Support safe platoon operation

## Communication Requirements

* Reliable V2V communication
* Low-latency message exchange
* Continuous status monitoring
* Formation coordination messaging

## Timing Constraints

| Requirement             | Constraint |
| ----------------------- | ---------- |
| Advertisement Reception | ≤ 200 ms   |
| Join Request Forwarding | ≤ 100 ms   |
| Join Approval Decision  | ≤ 500 ms   |
| Formation Completion    | ≤ 1 second | 

## Sequence Flow

1. Lead truck broadcasts platoon advertisement.
2. Follower trucks receive advertisement.
3. Follower truck submits join request.
4. Platoon Manager evaluates request.
5. Approval or rejection is issued.
6. Slot assignment is provided.
7. Formation maneuver begins.
8. Vehicles move to assigned positions.
9. Status updates are exchanged.
10. Formation completion is confirmed.
11. Normal platoon operation begins.

## SysML Models Included

* Requirement Diagram
* Sequence Diagram

## Expected Outcome

The system demonstrates how connected autonomous trucks can safely and efficiently form a platoon while satisfying communication, timing, and safety requirements. The model provides a foundation for future implementation and simulation of cooperative driving systems.

