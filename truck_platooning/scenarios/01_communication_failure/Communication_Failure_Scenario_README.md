# Communication Failure Scenario

## Scenario Information

| Field             | Value                                            |
| ----------------- | ------------------------------------------------ |
| **Scenario Name** | Communication Failure                            |
| **Project**       | Traffic Signal System for n-Street Intersections |
| **Creator**       | Daniel Chidi Chimezie                            |
| **Version**       | 1.0                                              |
| **Date**          | 2026                                             |
| **Scenario ID**   | SC-COM-01                                        |

---

## Scenario Description

A communication failure occurs when one or more traffic signal controllers lose the ability to exchange information with the central traffic management system or with neighboring intersection controllers.

The failure may result from:

* Network outages
* Hardware faults
* Cable disconnections
* Wireless interference
* Software communication errors

To maintain road safety, the affected intersection shall switch to a predefined fail-safe operating mode until communication is restored.

---

## Objective

Ensure that the traffic signal system continues operating safely during communication failures and prevents hazardous traffic situations.

---

## Preconditions

The following conditions must be satisfied before this scenario can occur:

* Traffic signal system is operating normally.
* Communication links between controllers and the central management system are active.
* All traffic lights are functioning correctly.

---

## Trigger Events

One of the following events triggers this scenario:

* Loss of network connection
* Communication timeout exceeded
* Failure of communication hardware
* Corrupted communication messages
* Missing communication messages

---

## Main Flow

1. System continuously monitors communication channels.
2. Communication watchdog detects missing messages.
3. Timeout threshold is exceeded.
4. Communication failure is confirmed.
5. Controller enters fail-safe mode.
6. Warning or alarm is generated.
7. Traffic signals switch to predefined safe behavior.
8. System periodically attempts reconnection.
9. Communication link is restored.
10. Normal operation resumes.

---

## Alternative Flows

### AF1 – Temporary Communication Loss

1. Communication interruption lasts less than the configured timeout period.
2. Communication is restored automatically.
3. No fail-safe mode is activated.
4. Normal operation continues.

### AF2 – Extended Communication Failure

1. Communication remains unavailable beyond the timeout period.
2. Fail-safe mode remains active.
3. Maintenance personnel are notified.
4. System continues operating with local control logic.

---

## Exception Flows

### EF1 – Communication Failure with Hardware Fault

1. Communication failure is detected.
2. Diagnostic system identifies a hardware malfunction.
3. Maintenance alert is issued immediately.
4. Intersection remains in fail-safe mode until repair is completed.

---

## Postconditions

### Success Condition

* Communication is restored.
* Traffic signal controllers synchronize correctly.
* Normal traffic operation resumes.

### Failure Condition

* Communication remains unavailable.
* System continues operating in safe local mode.
* Maintenance intervention is required.

---

## Safety Requirements

| ID            | Requirement                                                                         |
| ------------- | ----------------------------------------------------------------------------------- |
| **SR-COM-01** | The system shall detect communication failures within the specified timeout period. |
| **SR-COM-02** | The system shall enter fail-safe mode when communication is lost.                   |
| **SR-COM-03** | The system shall prevent conflicting green signals during communication failure.    |
| **SR-COM-04** | The system shall generate an alarm upon communication failure detection.            |
| **SR-COM-05** | The system shall attempt automatic reconnection at regular intervals.               |

---

## Actors

* Traffic Signal Controller
* Central Traffic Management System
* Communication Network
* Maintenance Personnel
* Road Users

  * Drivers
  * Cyclists
  * Pedestrians

---

## Inputs

* Communication status messages
* Heartbeat signals
* Network diagnostics
* Controller status reports

---

## Outputs

* Fail-safe mode activation
* Warning alarms
* Maintenance notifications
* Reconnection attempts
* System status reports

---

## Fail-Safe Behaviour

During communication failure:

* Safe signal timing plans are enforced.
* Conflicting green phases are prevented.
* Local intersection control remains active.
* Warning alarms are generated.
* Communication recovery attempts continue.
* Road safety is prioritized over traffic efficiency.

---

## Related Artifacts

* Activity Diagram: `communication_failure_activity.puml`
* State Machine Diagram: `communication_failure_state_machine.puml`
* Requirements Specification
* System Architecture Documentation

---

## Notes

* Timeout thresholds shall be configurable.
* Reconnection attempts shall be performed periodically.
* The system shall remain operational even when isolated from the central controller.
* Safety requirements take precedence over traffic optimization during communication failures.
