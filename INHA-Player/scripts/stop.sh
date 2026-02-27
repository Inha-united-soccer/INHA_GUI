#!/bin/bash
echo ["STOP VISION"]
pkill -9 vision_node
echo ["STOP BRAIN"]
pkill -9 brain_node
echo ["STOP SOUND"]
pkill -9 sound_play_node
echo ["STOP GAMECONTROLLER"]
pkill -9 game_controller
