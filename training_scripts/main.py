#!/usr/local/bin/python

import os, sys, shutil                                             
from multiprocessing import Pool

processes = ( 
              'roscore', 
              'launch_ambf', 
              'psmIK_service', 
              'training', 
              # 'tensorboard_launch'
            )

def run_process(script_name):
  print('Running process: '.format(script_name))
  os.system('./{}.bash >> .logs/{}.txt 2>&1'.format(script_name, script_name))

if __name__ == '__main__':
  
  if os.path.isdir('.logs'):
    print('Previous .logs dir exists, deleting it...')
    shutil.rmtree('.logs')

  if os.path.isdir('ddpg_dvrk_tensorboard'):
    print('Previous tensorboard dir exists, deleting it...')
    shutil.rmtree('ddpg_dvrk_tensorboard')
  
  print('Creating new .logs dir...')
  os.mkdir('.logs')

  print('Running {} processes: [{}]'.format(len(processes), processes))
  pool = Pool(processes=len(processes))
  pool.map(run_process, processes)
