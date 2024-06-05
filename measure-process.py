#!/usr/bin/env python3

# Usage example:
#
# ./measure-process.py --process mvsim
#
# ./measure-process.py --process gzserver --process gzclient   # sums both

import psutil
import time

import argparse

parser = argparse.ArgumentParser()
# parser.add_argument("--process", type=str,
#                     help="name of the process to measure", required=True, action='append')
parser.add_argument("--period", type=float,
                    help="period in seconds to wait for the measure (<1 second is ok)", required=False, default=0.5)

args = parser.parse_args()


def do_measure():
    # pi = psutil.process_iter()
    # val = 0.0
    # for proc in pi:
    #     if not proc.name() in args.process:
    #         continue

    #     # We need to call this twice to get a first meaningful value
    #     val += proc.cpu_percent()
    #     print(proc.name()+'=' + str(proc.cpu_percent()) + ' val=' + str(val) + ' ' + str(proc.cmdline()))

    val = 0.0
    val += psutil.cpu_percent()
    return val



do_measure()
import csv

log = {}
log["times"] = []
log["cpu"] = []
log["mem_virtual"] = []

with open('log_5.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    

    start_time = time.time()
    for i in range(180):
                    # Find current time
        current_time = time.time()
        elapsed_time = current_time - start_time
        current_mem_virtual = psutil.virtual_memory().used/ 1024.0**2
        current_cpu = do_measure()
        log["times"].append(elapsed_time)
        log["cpu"].append(current_cpu)
        log["mem_virtual"].append(current_mem_virtual)

        time.sleep(args.period)
        print(current_cpu)
        writer.writerow(
                        [f"{elapsed_time},{current_cpu},{current_mem_virtual}"]
                    )
        i += 1

import matplotlib.pyplot as plt

with plt.rc_context({"backend": "Agg"}):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    ax.plot(log["times"], log["cpu"], "-", lw=1, color="r")

    ax.set_ylabel("CPU (%)", color="r")
    ax.set_xlabel("time (s)")
    ax.set_ylim(0.0, max(log["cpu"]) * 1.2)

    ax2 = ax.twinx()

    ax2.plot(log["times"], log["mem_virtual"], "-", lw=1, color="b")
    ax2.set_ylim(0.0, max(log["mem_virtual"]) * 1.2)

    ax2.set_ylabel("Real Memory (MB)", color="b")

    ax.grid()

    fig.savefig("plot")