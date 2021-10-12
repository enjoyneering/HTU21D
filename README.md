[![license-badge][]][license] ![version] [![stars][]][stargazers] ![hit-count] [![github-issues][]][issues]

<h1>This library is no longer supported. New library here - https://github.com/enjoyneering/HTU2xD_SHT2x_Si70xx<h1>

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

[license-badge]: https://img.shields.io/badge/License-GPLv3-blue.svg
[license]:       https://choosealicense.com/licenses/gpl-3.0/
[version]:       https://img.shields.io/badge/Version-1.2.1-green.svg
[stars]:         https://img.shields.io/github/stars/enjoyneering/HTU21D.svg
[stargazers]:    https://github.com/enjoyneering/HTU21D/stargazers
[hit-count]:     https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fenjoyneering%2FHTU21D&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false
[github-issues]: https://img.shields.io/github/issues/enjoyneering/HTU21D.svg
[issues]:        https://github.com/enjoyneering/HTU21D/issues/

**Library returns 255, if there is a CRC8 mismatch or a communication error has occurred
