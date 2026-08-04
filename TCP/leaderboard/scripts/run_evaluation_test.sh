#!/bin/bash
#export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
#export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
#export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
#export PYTHONPATH=$PYTHONPATH:leaderboard
#export PYTHONPATH=$PYTHONPATH:leaderboard/team_code
#export PYTHONPATH=$PYTHONPATH:scenario_runner

export LEADERBOARD_ROOT=leaderboard
export CHALLENGE_TRACK_CODENAME=SENSORS
export PORT=2000
export TM_PORT=8000
export DEBUG_CHALLENGE=0
export REPETITIONS=3 # multiple evaluation runs
export RESUME=True


# TCP evaluation
export ROUTES=/home/hp/STF/SafeBenchHK/SafeBenchHK/safebench/scenario/scenario_data/1201-ShaTin12D/scenario_01_routes/scenario_01_route_00_weather_00.xml
export TEAM_AGENT=team_code/tcp_agent.py
export TEAM_CONFIG= /home/hp/STF/SafeBenchHK/SafeBenchHK/safebench/agent/model_ckpt/tcp/best_model.ckpt
export CHECKPOINT_ENDPOINT=results_TCP.json
export SCENARIOS=/home/hp/STF/SafeBenchHK/SafeBenchHK/safebench/scenario/scenario_data/1201-ShaTin12D/scenarios/scenario_01.json
export SAVE_PATH=data/results_TCP/


python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
--scenarios=${SCENARIOS}  \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=${DEBUG_CHALLENGE} \
--record=${RECORD_PATH} \
--resume=${RESUME} \
--port=${PORT} \
--trafficManagerPort=${TM_PORT}


