#!/usr/bin/env python3

import numpy as np
import math
import click
import json
import sys
from bitarray import bitarray

from utils import add_options

plot_env_options = [
    click.option('--show_distances', default=False, type=bool),
    click.option('--draw_start_goal', default=True, type=bool),
    click.option('--draw_start_goal_thetas', default=False, type=bool),
    click.option('--set_title', default=True, type=bool)
]


@add_options(plot_env_options)
def plot_env(env, run_id: int = -1, colors=('b', 'r'),
             draw_start_goal=True, draw_start_goal_thetas=False,
             show_start_goal_labels=True,
             set_title=True, show_distances=False,
             custom_min_x=None, custom_min_y=None,
             custom_max_x=None, custom_max_y=None,
             polygon_margin_ratio=0.1,
             **_):
    """
    Plots the json branch for an environment.
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.collections import PatchCollection

    title = ''
    if run_id >= 0:
        title += 'Run %i ' % run_id
    if colors is None:
        colors = ['b', 'r']
    plt.gca().set_axisbelow(True)
    if env["type"] == "grid":
        w = env["width"]
        h = env["height"]
        ax = plt.gca()

        if w * h > 200 * 200:
            major_ticks = np.arange(0, max(w+1, h+1), 50)
            minor_ticks = np.arange(0, max(w+1, h+1), 10)
        elif w * h > 100 * 100:
            major_ticks = np.arange(0, max(w+1, h+1), 25)
            minor_ticks = np.arange(0, max(w+1, h+1), 5)
        else:
            major_ticks = np.arange(0, max(w+1, h+1), 10)
            minor_ticks = np.arange(0, max(w+1, h+1), 1)

        ax.set_xticks(major_ticks)
        ax.set_xticks(minor_ticks, minor=True)
        ax.set_yticks(major_ticks)
        ax.set_yticks(minor_ticks, minor=True)

        ax.grid(which='both')
        ax.grid(which='minor', alpha=0.2)
        ax.grid(which='major', alpha=0.5)

        if show_distances:
            if "distances" not in env:
                click.echo('Environment contains no distance information.', err=True)
            else:
                map_data = np.array(env["distances"]).reshape((w, h))
                click.echo(map_data)
                click.echo("Maximum distance:", map_data.max())
                plt.imshow(np.flip(map_data, axis=0), cmap='jet', vmin=0, vmax=map_data.max(), extent=[0, w, 0, h])
        map_data = np.array(list(bitarray(env["map"]))).reshape((h, w))
        map_data = 1. - np.flip(map_data, axis=0)
        plt.imshow(map_data, cmap='gray', vmin=-1, vmax=1, extent=[0, w, 0, h], alpha=0.5)
        ax.set_xlim([0, w])
        ax.set_ylim([0, h])

        title += '($%i\\times%i$ %s' % (w, h, env["generator"])
        if env["generator"] in ("corridor", "random"):
            title += " %i" % env["seed"]
        title += ')'
    elif env["type"] == "polygon":
        polygons = []
        for points in env["obstacles"]:
            points = np.array(points)
            polygons.append(Polygon(points, True))
        plt.gca().add_collection(PatchCollection(polygons, color='lightgray', edgecolor='gray', alpha=0.8))
        plt.grid()

        title += env["name"]

    elif env["type"] == "yaml":
        import occmap as oc
        occmap = oc.OccMap()
        occmap.load(env["file"])
        occmap.plot()
    #plt.axis('equal')

    if draw_start_goal:
        start = env["start"]
        goal = env["goal"]
        plt.scatter([start[0]], [start[1]], label=("Start" if show_start_goal_labels else None), color=colors[0])
        plt.scatter([goal[0]], [goal[1]], label=("Goal" if show_start_goal_labels else None), color=colors[1])
        if draw_start_goal_thetas:
            radius = max(env["width"], env["height"]) / 20
            head_width = max(env["width"], env["height"]) / 100
            plt.arrow(start[0], start[1], math.cos(start[2]) * radius, math.sin(start[2]) * radius, width=0.01,
                      head_width=head_width,
                      fc=colors[0], color=colors[0], edgecolor=None)
            plt.arrow(goal[0], goal[1], math.cos(goal[2]) * radius, math.sin(goal[2]) * radius, width=0.01,
                      head_width=head_width,
                      fc=colors[1], color=colors[1], edgecolor=None)
    if set_title:
        plt.title(title)

    plt.gca().autoscale(False)
    plt.gca().set_aspect('equal', 'box')
    if env["type"] == "polygon":
        margin_x = env["width"] * polygon_margin_ratio
        margin_y = env["height"] * polygon_margin_ratio
    else:
        margin_x = 0.
        margin_y = 0.
    if custom_min_x is not None and custom_max_y is not None:
        plt.gca().set_xlim([custom_min_x - margin_x, custom_max_x + margin_x])
        plt.gca().set_ylim([custom_min_y - margin_y, custom_max_y + margin_y])
    elif "min_x" in env and "max_y" in env:
        plt.gca().set_xlim([env["min_x"] - margin_x, env["max_x"] + margin_x])
        plt.gca().set_ylim([env["min_y"] - margin_y, env["max_y"] + margin_y])
    elif "width" in env and "height" in env:
        plt.gca().set_xlim([0, env["width"]])
        plt.gca().set_ylim([0, env["height"]])


@click.command()
@click.option('--json_file', type=str, help='Name of the JSON file of a benchmarking run.')
@click.option('--run_id', default='0', type=str,
              help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--show_distances', default=False, type=bool)
@click.option('--draw_start_goal', default=True, type=bool)
@click.option('--draw_start_goal_thetas', default=True, type=bool)
@click.option('--set_title', default=True, type=bool)
@click.option('--headless', default=False, type=bool)
@click.option('--save_file', default=None, type=str)
@click.option('--dpi', default=200, type=int)
def main(json_file: str, run_id: str, draw_start_goal=True, draw_start_goal_thetas=True,
         set_title=True, show_distances=True, headless=False, save_file: str = None, dpi: int = 200):
    if headless and 'matplotlib' not in sys.modules:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    data = json.load(open(json_file, 'r'))
    if run_id.lower() == "all":
        run_ids = list(range(len(data["runs"])))
    else:
        run_ids = [int(s.strip()) for s in run_id.split(',')]

    for i in run_ids:
        print("Plotting run %i" % i)
        env = data["runs"][i]["environment"]
        aspect_ratio = env["width"] / env["height"]
        plt.figure("Run %i" % i, figsize=(5, 5 * aspect_ratio))
        plot_env(env, i, draw_start_goal=draw_start_goal,
                 draw_start_goal_thetas=draw_start_goal_thetas,
                 set_title=set_title, show_distances=show_distances)
        if save_file is not None:
            plt.tight_layout()
            ext = save_file.rindex('.')
            filename = save_file[:ext] + '_%i' % i + save_file[ext:]
            plt.savefig(filename, dpi=dpi)
    if not headless:
        plt.show()

