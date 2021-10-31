#!/bin/bash
ghdl -a project.vhd
ghdl -e Test
ghdl -r Test --stop-time=1000fs --vcd=$1
