# CHANGELOG

## [1.10.2] - 2019-12-17
### Fixed
- all template configs updated with DTVEC_BASE config

## [1.10.1] - 2019-12-17
### Changed
- made DTVEC reset value parameterizable from soc_config.ini

## [1.5.0] - 2019-07-29

### Changed
- resetpc is no longer a parameter for stage1.bsv, riscv.bsv and eclass.bsv. It is now a dynamic
   argument to the all the modules. The reset-pc will be assigned after initialization of the
   registerfile.
- Updated devices version in manager.sh from 3.4.0 to 3.4.1
