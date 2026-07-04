#!/usr/bin/env python3
"""
Simple script to push FMS/game data into NetworkTables so the simulator sees it.
Usage: python tools/push_game_data.py [GAME_DATA]
GAME_DATA: string like 'R' or 'B' or 'R...' per WPILib format
This script sets /FMSInfo/gameSpecificMessage and updates /FMSInfo/matchTime every second
for a short demo.
"""
import sys
import time
from networktables import NetworkTables

GAME_DATA = sys.argv[1] if len(sys.argv) > 1 else "R"
# Connect to local NetworkTables server (simulator)
NetworkTables.initialize(server='127.0.0.1')
# Wait for connection
time.sleep(0.5)
ft = NetworkTables.getTable("FMSInfo")
# Set some common FMSInfo keys
ft.putString('gameSpecificMessage', GAME_DATA)
ft.putString('eventName', 'SimEvent')
ft.putString('matchType', 'F')
ft.putNumber('matchNumber', 1)
ft.putBoolean('isFMSAttached', True)
# Simulate match time counting down from 135 to 0
match_time = 135
print(f"Pushing gameData='{GAME_DATA}' and starting simulated matchTime={match_time} s")
for i in range(30):
    # update a few times and sleep
    ft.putNumber('matchTime', match_time)
    print(f"-> matchTime={match_time}")
    match_time = max(0, match_time - 1)
    time.sleep(1.0)

print('Done.')
