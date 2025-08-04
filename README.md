# Velocity-Based Training Device

This repository contains the Arduino firmware and design logic for a compact, velocity-based training device designed and built at Yale Center for Engineering Innovation and Design (CEID). The device measures peak/average velocity and power output in real-time during compound lifts, enabling data-driven athletic performance tracking.

## Features
- Real-time measurement of barbell velocity and power
- TOF sensor + gyroscope + accelerometer fusion algorithm
- TFT touchscreen interface with leaderboard, calibration, and workout modes
- Fully 3D-printed, friction-fit housing
- <5% error compared to industry benchmark devices (e.g., Push)

## Technologies Used
- Arduino IDE + C++
- TOF (Time-of-Flight) sensor
- MPU6050 gyroscope + accelerometer
- TFT LCD Touchscreen
- 3D CAD via OnShape, 3D printed housing

## Project Highlights
- Reduced setup time by 70% and manufacturing cost by 85%
- Deployed to athletic coaches for usability testing
- Achieved 86% satisfaction score in MVP feedback
- Co-filed for a patent with Yale University

## File Structure
- `VelocirepperDevice1Updated.ino`: Main Arduino firmware
- `pitches.h`: set of pre-defined constants representing the frequencies of various musical notes
- `cad/`: 3D CAD files (OnShape exports - to be uploaded)
- `docs/`: Supporting documentation (future updates)

## 🎥 Demo & Documentation
[👉 Click here to access design videos and demo walkthrough](https://drive.google.com/drive/folders/192VMI9i2ERZRvw-EJ-ldwrsGDiov8WNI)

## Collaborators
- Jin Wuk Lee
- Michelle Luh
- Ben Phifer
- Yale CEID Faculty & Engineers

## License
MIT License (or your preferred open-source license)
