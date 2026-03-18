# Hybrid Integration of Avionics Components using X-Plane and ED-247

## Overview
This project implements a **Hardware-in-the-Loop (HiL) avionics test bench** that integrates the **X-Plane flight simulator** with a distributed embedded system using the **[Airbus ED-247 standard](https://github.com/airbus/ED247_LIBRARY)**.

Real-time flight data is captured from X-Plane, processed into an **AFDX-like packet format**, transmitted between two Raspberry Pi nodes using **ED-247 streams**, and visualised through a web-based dashboard. A reverse communication path enables **closed-loop control commands**.

---

## System Architecture

### Data Flow

1. **X-Plane → Raspberry Pi A**
   - UDP DATA packets (Port 49000)
   - Flight parameters: pitch, roll, heading, speed, altitude

2. **Raspberry Pi A (Sender Node)**
   - UDP receiver (non-blocking)
   - X-Plane DATA decoder
   - FlightState struct creation
   - AFDX-like packet builder (VL ID, sequence, payload)
   - ED-247 sender

3. **Network Layer**
   - Ethernet LAN using UDP transport

4. **Raspberry Pi B (Receiver Node)**
   - ED-247 receiver
   - Packet decoding and FlightState extraction
   - Latency, packet loss, and rate analysis

5. **Web Interface**
   - FastAPI backend
   - WebSocket streaming
   - Real-time instrument dashboard

6. **Control Loop (Reverse Path)**
   - Node-RED sends UDP commands to Pi B
   - Pi B transmits via ED-247 (VL2001_CTRL)
   - Pi A forwards commands to X-Plane

---

## Features

- Real-time flight data acquisition from X-Plane  
- Custom AFDX-like packetisation  
- ED-247-based communication between nodes  
- Latency and packet loss monitoring  
- Web-based flight instrument visualisation  
- Closed-loop control using Node-RED  
- Modular HiL system architecture  

---


**Note:**  
The main working scripts are located in:
- `/src/pi-a` → ED-247 sender (Pi A)  
- `/src/pi-b` → ED-247 receiver + control sender (Pi B)  

---

## Technologies Used

- C++  
- CMake  
- ED-247 library  
- UDP sockets  
- Raspberry Pi  
- X-Plane Flight Simulator  
- FastAPI & WebSockets  
- Node-RED  

---

## Build Instructions

### Prerequisites
- CMake ≥ 3.10  
- g++ compiler  
- ED-247 library installed  
- Linux / Raspberry Pi  

### Build

```bash
cd xplane_bridge
mkdir build
cd build
cmake ..
make
```
## Running the System

### 1. Configure X-Plane
- Enable UDP output on port **49000**
- Select required flight data parameters

### 2. Run Pi A (Sender)
```bash
./pi-a
```
### 3. Run Pi B (Receiver)
```bash
./pi_b_receiver
```
### 4. Run Web server (Pi B)
```bash
./pi_b_receiver
```
### 5. (Optional) Run Control System
- Start Node-RED flow
- Send UDP commands to Pi B (Port 6001)

---

## Performance Metrics

The system measures:
- Stream rate (Hz)
- Packet loss (%)
- End-to-end latency (ms)
- Sequence integrity

**Example:**
```bash
ATT: 20 Hz | missed=0 (0%) | mean latency ≈ 48 ms
SPD: 10 Hz | missed=0 (0%) | mean latency ≈ 48 ms
ALT: 10 Hz | missed=0 (0%) | mean latency ≈ 48 ms
```
## Limitations

- No security or encryption implemented  
- Dependent on network reliability  
- Uses simulated avionics data (X-Plane)

---

## Future Work

- Add secure communication (encryption / authentication)  
- Integrate real avionics interfaces (e.g. ARINC 429, AFDX)  
- Improve timing determinism and scheduling  
- Implement fault injection and redundancy testing  
- Extend closed-loop control capabilities  

---

## Author

Taha Al-Salihi  
MEng Electronic & Computer Engineering  
University of Limerick  

---

## License

This project is for academic and research purposes.
