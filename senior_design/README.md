# CQI 2026 Robot Software Functional Requirements

## Overview
This document outlines the functional requirements for the embedded software running on the ESP8266 NodeMCU. The robot must operate in two phases—autonomous and remote-controlled—using a single Arduino sketch and the RemoteXY app for user interaction.

---

## 1. System Initialization

### Function: `initializeSystem()`
- Initialize RemoteXY interface
- Configure GPIOs for motors, sensors, and actuators
- Initialize SPI and NFC reader (MFRC522)
- Set initial robot state to `AUTONOMOUS`
- Calibrate sensors if needed

---

## 2. Main Control Loop

### Function: `mainLoop()`
- Continuously handle RemoteXY input
- Dispatch control to the appropriate phase handler:
  - `runAutonomousPhase()`
  - `runRemoteControlPhase()`
- Monitor timers and transitions between phases
- Handle emergency stop or restart commands

---

## 3. Autonomous Navigation

### Function: `runAutonomousPhase()`
- Follow black tape line using IR sensors
- Navigate to the central switch
- Detect and confirm contact with the switch
- Transition to `REMOTE_CONTROL` phase upon success or timeout

---

## 4. Remote Control Operation

### Function: `runRemoteControlPhase()`
- Interpret RemoteXY joystick/button inputs
- Drive motors accordingly for manual navigation
- Allow user to trigger NFC scans for:
  - Resource identification (top-mounted NFC)
  - Container identification (side-mounted NFC)
- Track and log identified hazardous containers
- Validate resource placement in correct zones

---

## 5. NFC Resource Identification

### Function: `scanResourceNFC()`
- Read UID from top-mounted NFC reader
- Match UID to resource type using lookup table
- Return resource classification (wood, metal, etc.)
- Flag hazardous waste for penalty handling

---

## 6. NFC Container Identification

### Function: `scanContainerNFC()`
- Read UID from side-mounted NFC reader
- Match UID to container type
- Log hazardous container positions for end-of-game reporting

---

## 7. RemoteXY Interface Handling

### Function: `handleRemoteXYInput()`
- Parse joystick and button states
- Map inputs to motor control signals
- Trigger NFC scans or other actions based on button presses
- Display status feedback (phase, time, errors)

---

## 8. Timer and Phase Management

### Function: `startPhaseTimers()`
- Start countdown for autonomous phase (1 minute max)
- Start total challenge timer (5 minutes max)
- Enforce phase transitions and timeouts

---

## 9. Safety and Compliance

### Function: `enforceSafetyConstraints()`
- Prevent robot from exceeding arena boundaries
- Stop motors if unsafe behavior is detected
- Enforce restart penalties if robot is removed from field
- Ensure robot returns to start zone before timeout

---

## 10. Scoring and Logging

### Function: `logGameEvents()`
- Track number and type of resources collected
- Track correct vs incorrect placements
- Record hazardous container identifications
- Output summary at end of game for scoring

---

## 11. Emergency Handling

### Function: `handleRestartRequest()`
- Stop all motion
- Reset robot state and timers
- Wait for judge approval before re-entry
- Apply 5% penalty per restart

---

## 12. Presentation Support (Optional)

### Function: `exportDebugInfo()`
- Output logs via Serial for post-run analysis
- Provide UID-to-resource mapping for judges
- Support presentation with technical justification

---