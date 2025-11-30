cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ring-fw)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/i2c_scanner.c)
target_sources(app PRIVATE src/bma400.c)
target_sources(app PRIVATE src/maxm86161.c)
target_sources(app PRIVATE src/max30208.c)
target_sources(app PRIVATE src/ble_app.c)