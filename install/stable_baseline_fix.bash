# !/bin/bash

rm -f /usr/local/lib/python3.6/dist-packages/stable_baselines/ddpg/ddpg.py && \
cp ${AMBF_WS}/training_scripts/stable_baseline_fix/ddpg.py \
   /usr/local/lib/python3.6/dist-packages/stable_baselines/ddpg/