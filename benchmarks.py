#! /usr/bin/env python3.12

import pathlib
import argparse
import subprocess
import requests
import socket
import datetime
import tomllib
import time
import sys
import os

def send_message_to_telegram(msg):
    token = "6276168703:AAHuAEsAKTu08oEHbEPBF2PUHm4_IhQ7z2c"
    chat_id = "60446563"

    if not token or not chat_id:
        return

    url = f'https://api.telegram.org/bot{token}/sendMessage?chat_id={chat_id}&text={msg}'
    try:
        requests.get(url)
    except ConnectionError:
        print('no connection')


# must be launch from project root dir
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('input_dir', type=pathlib.Path, help='instances directory')
    parser.add_argument('--config', type=pathlib.Path, help='configuration file')
    parser.add_argument('--exec', help='name of the executable')
    parser.add_argument('--timelimit', type=float, default=float('inf'))
    parser.add_argument('--iterlimit', type=float, default=float('inf'))
    parser.add_argument('--memlimit', type=float, default=float('inf'))
    parser.add_argument('--threads', type=int, default=0, help='number of threads to use')
    parser.add_argument('--seed', type=int, default=0, help='random seed')

    args = parser.parse_args()
    
    if not args.input_dir.exists():
        print('input_dir does not exist')
        sys.exit()

    if not args.input_dir.glob('**/*.json'):
        print('input_dir does not contains any instance')
        sys.exit()

    if args.config:
        with open(args.config, 'rb') as f:
            config = tomllib.load(f);
    else:
        config = {}

    if args.exec:
        execs = args.exec.split(',')
    elif args.config:
        execs = config['models'] + config['algos']
    else:
        print('no executable specified (use --exec or a config file)')
        sys.exit()

    cmdl_args = ['--timelimit']
    if config:
        timelimit = config.get('timelimit', float('inf'))
    else:
        timelimit = args.timelimit
    cmdl_args.append(str(timelimit))

    cmdl_args.append('--iterlimit')
    if config:
        iterlimit = config.get('iterlimit', float('inf'))
    else:
        iterlimit = args.iterlimit
    cmdl_args.append(str(iterlimit))

    cmdl_args.append('--memlimit')
    if config:
        memlimit = config.get('memlimit', float('inf'))
    else:
        memlimit = args.memlimit
    cmdl_args.append(str(memlimit))

    cmdl_args.append('--threads')
    if config:
        threads = config.get('threads', 0)
    else:
        threads = args.threads
    cmdl_args.append(str(threads))

    cmdl_args.append('--seed')
    if config:
        seed = config.get('seed', 0)
    else:
        seed = args.seed
    cmdl_args.append(str(seed))

    for cmd in execs:
        cmd = './' + cmd
        start = time.time();
        for input in args.input_dir.glob('**/*.json'):
            subprocess.run([cmd, input] + cmdl_args)

        end = time.time();
        elapsed_time = datetime.timedelta(seconds=(end - start))

        hostname = os.environ.get('HOSTNAME', socket.gethostname())
        msg = f'run of {cmd} on {hostname} completed in {elapsed_time}'
        send_message_to_telegram(msg)

    postopt = pathlib.Path.cwd() / './postopt.py'
    if postopt.exists():
        print('run post optimization...')
        subprocess.run([postopt, args.input_dir, '--config', args.config])
