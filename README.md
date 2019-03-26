[![license-badge][]][license] ![version] [![stars][]][stargazers]  [![GitHub issues][]][issues]

# HTU21D

This is an Arduino library for SHT21, HTU21D & Si70xx Digital Humidity and Temperature Sensor

Supports all sensors features:

- read humidity for SHT21, HTU21D or compensated humidity for Si70xx**
- calculate compensated humidity for SHT21, HTU21D**
- read temperature for SHT21, HTU21D, Si70xx or retrive temperature value after RH measurement for Si70xx**
- soft reset
- check battery status
- turn ON/OFF build-in Heater
- read FW version**
- read sensor ID**

Tested on:

- Arduino AVR
- Arduino ESP8266
- Arduino STM32

[license]:       https://img.shields.io/badge/License-GPLv3-blue.svg
[license-badge]: https://img.shields.io/aur/license/yaourt.svg
[version]:       https://img.shields.io/badge/Version-1.2.1-green.svg
[stars]:         https://img.shields.io/github/stars/enjoyneering/HTU21D.svg
[stargazers]:    https://github.com/enjoyneering/HTU21D/stargazers
[GitHub issues]: https://img.shields.io/github/issues/enjoyneering/HTU21D.svg
[issues]:        https://GitHub.com/enjoyneering/HTU21D/issues/

**Sensor returns 255, if there is a CRC8 mismatch or a communication error has occurred
