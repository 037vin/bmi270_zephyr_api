#
# Copyright (c) 2021 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(NONE)

add_subdirectory(Users/vin/git/bmi270_zephyr_api/bmi270)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
