# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial release of the ADS1256 driver
- Support for single-ended and differential measurements
- Programmable gain amplifier (PGA) control
- Multiple data rates from 2.5 SPS to 30,000 SPS
- Self and system calibration functionality
- Buffer enable/disable control
- Non-blocking operation support
- Continuous reading mode
- Comprehensive error handling
- Support for defmt logging in embedded environments
- Mock testing support

### Changed

- Updated to embedded-hal 1.0.0

### Fixed

- None

## [0.1.0] - 2024-03-03

- Initial release

[Unreleased]: https://github.com/yourusername/ads125x/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/yourusername/ads125x/releases/tag/v0.1.0
