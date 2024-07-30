#!/usr/bin/env python3

import argparse
import pathlib
import subprocess
import time
import sys

indices = [2, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15]
remote_dir = 'arcis'
data_dir = 'data/all'

def deploy(remote_dir, minipcs):
    for pc in minipcs:
        subprocess.run(['scp', '-r', 'bin', 'benchmarks.py', 'config.toml', f'{pc}:{remote_dir}'])
        subprocess.run(['ssh', pc, 'mkdir', f'{remote_dir}/data'])

def collect(remote_dir, data_dir, minipcs):
    results_dir = f'{data_dir}_{time.strftime("%Y%m%d")}'
    subprocess.run(['mkdir', '-p', results_dir])
    for pc in minipcs:
        subprocess.run(['scp', '-r', f'{pc}:{remote_dir}/data/*', results_dir])
    subprocess.run(['zip', '-r', f'{results_dir}.zip', results_dir])

def clean(remote_dir, minipcs):
    for pc in minipcs:
        subprocess.run(['ssh', pc, 'rm', '-rf', f'{remote_dir}/*'])

def clean_data(remote_dir, minipcs):
    for pc in minipcs:
        subprocess.run(['ssh', pc, 'rm', '-rf', f'{remote_dir}/data/*'])

def ls(remote_dir, minipcs):
    for pc in minipcs:
        print(f'{pc}')
        subprocess.run(['ssh', pc, 'ls', f'{remote_dir}/*'])
        print('---------------------------')

def data(remote_dir, data_dir, minipcs):
    data_path = pathlib.Path(data_dir)
    files = data_path.glob('**/*.json')

    i = 0;
    for file in files:
        subprocess.run(['scp', str(file), f'{minipcs[i]}:{remote_dir}/data'])
        i = (i + 1) % len(minipcs)

def ps(minipcs):
    for pc in minipcs:
        print(f'{pc}:')
        subprocess.run(['ssh', pc, 'ps aux --sort=-pcpu | head -n 2'])

def run(remote_dir, minipcs):
    for pc in minipcs:
        subprocess.Popen(['ssh', pc, f'cd {remote_dir} && nohup ./benchmarks.py data --config config.toml &>/dev/null </dev/null'])
    time.sleep(1)

if __name__ == '__main__':

    if len(indices) == 0:
        print('no minipc specified')
        sys.exit(1)

    if remote_dir == '':
        print('no remote dir specified')
        sys.exit(1)

    if data_dir == '':
        print('no data dir specified')
        sys.exit(1)

    parser = argparse.ArgumentParser()
    parser.add_argument('command', help='command to run')

    args = parser.parse_args()

    minipcs = ['minipc'+str(i) for i in indices]

    if args.command == 'deploy':
        deploy(remote_dir, minipcs)
    
    elif args.command == 'collect':
        collect(remote_dir, data_dir, minipcs)
    
    elif args.command == 'clean':
        clean(remote_dir, minipcs)

    elif args.command == 'clean_data':
        clean_data(remote_dir, minipcs)

    elif args.command == 'ls':
        ls(remote_dir, minipcs)
    
    elif args.command == 'data':
        data(remote_dir, data_dir, minipcs)

    elif args.command == 'ps':
        ps(minipcs)
    
    elif args.command == 'run':
        run(remote_dir, minipcs)
    
    elif args.command == 'all':
        clean(remote_dir, minipcs)
        deploy(remote_dir, minipcs)
        data(remote_dir, data_dir, minipcs)
        run(remote_dir, minipcs)
