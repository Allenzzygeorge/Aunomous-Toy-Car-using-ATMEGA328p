# Aunomous Toy Car using ATMEGA328p

> Autonomous toy car project built around the ATMEGA328p — obstacle avoidance, motor control and remote programming.

**Last updated:** 2025-12-08 02:03 UTC

---

<!-- STATS -->
![GitHub stats](https://github-readme-stats.vercel.app/api?username=Allenzzygeorge&show_icons=true&count_private=true&theme=default)

![Top Langs](https://github-readme-stats.vercel.app/api/top-langs/?username=Allenzzygeorge&layout=compact&theme=default)

![GitHub Streak](https://streak-stats.demolab.com?user=Allenzzygeorge&theme=dark)

---

## Badges
[![Repo size](https://img.shields.io/github/repo-size/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p)](https://github.com/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p) [![License](https://img.shields.io/github/license/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p)](https://github.com/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p/blob/main/LICENSE) [![Stars](https://img.shields.io/github/stars/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p?style=social)](https://github.com/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p/stargazers) [![Issues](https://img.shields.io/github/issues/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p)](https://github.com/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p/issues)

---

## Tech Stack
- **MCU:** ATMEGA328p (AVR)
- **Motor driver:** L298N (or recommended alternative: TB6612 or DRV8833)
- **Sensors:** Ultrasonic HC-SR04 / IR line sensors / MPU (optional)
- **Tools & Software:** avr-gcc / avrdude / PlatformIO / Arduino IDE / C / Makefile
- **Others:** Breadboard, LiPo / battery management

---

## Skills & Topics Covered
- Embedded C and AVR microcontroller programming
- PWM motor control, H-bridge drivers
- Ultrasonic distance sensing & obstacle avoidance
- Serial bootloading & flashing with avrdude
- PCB prototyping and wiring
- Basic electronics (voltage regulation, motor power management)

---

## Repo Links & Quick Access
- 📁 Repository: `Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p`
- 📄 Hardware schematic: `docs/schematic.png` <!-- update path -->
- 🔧 Build instructions: `docs/BUILD.md` <!-- update path -->
- 🧾 License: `LICENSE` <!-- update as needed -->

---

## How to build & flash (quick)
1. Clone repo  
   `git clone https://github.com/Allenzzygeorge/Aunomous-Toy-Car-using-ATMEGA328p.git`
2. Build (example with avr-gcc/make):  
   `make`
3. Flash (example using avrdude):  
   `avrdude -p m328p -c usbasp -U flash:w:build/firmware.hex`

---

## Screenshots / Demo
![demo-placeholder](docs/demo.png) <!-- replace with real screenshot path -->

---

## License
This project is released under the MIT License. See `LICENSE` for details.
