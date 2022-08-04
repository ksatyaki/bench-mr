import yaml
from mpb import MPB, MultipleMPB
import matplotlib as mpl
mpl.rcParams['mathtext.fontset'] = 'cm'
mpl.rcParams['pdf.fonttype'] = 42

from copy import deepcopy
import math

from argparse import ArgumentParser
from yaml.loader import SafeLoader
from tqdm import tqdm
import os


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("setup_yaml_file",
                        help="Yaml file for the ",
                        metavar="DATAFILE"
                        )
    parser.add_argument("-s", "--sampling-fns", nargs="*", type=str, help="A list of sampling functions (space separated).")
    args = parser.parse_args()

    sampling_functions = args.s if len(args.s) > 0 else ["uniform", "ellipse", "intensity", "dijkstra"]
    print("Running these sampling functions: " + sampling_functions)
    with open(args.setup_yaml_file) as f:
        setup = yaml.load(f, SafeLoader)

    print("***************************************************************")
    print("Successfully read the yaml file containing {} start-goal pairs.", len(setup['sg']))
    print("***************************************************************")

    # Basic Setup
    mpb = MPB()
    mpb["ompl.seed"] = -1  # set the seed of the OMPL planners
    mpb.set_planners(['informed_rrt_star'])
    mpb.set_steer_functions(['car'])
    mpb["steer.car_turning_radius"] = 1.0
    mpb["steer.sampling_resolution"] = 0.01
    mpb["max_planning_time"] = 256
    mpb["ompl.geometric_planner_settings.RRTstar.delay_collision_checking"] = "0"
    mpb["ompl.geometric_planner_settings.RRTstar.goal_bias"] = "0.01"

    mpb["env.collision.robot_shape_source"] = os.path.abspath(os.getcwd() + "/../maps/simple_robot.yaml")
    mpb.set_image_yaml_env(os.path.abspath(os.getcwd() + "/../" + setup["occmap_file"]))
    cliff_map_file = os.path.abspath(os.getcwd() + "/../" + setup["cliff_map_file"])
    intensity_map_file = os.path.abspath(os.getcwd() + "/../" + setup["intensity_map_file"])
    gmmt_map_file = os.path.abspath(os.getcwd() + "/../" + setup["gmmt_map_file"])
    mpb["max_planning_time"] = 64

    cost_fns = setup["cost_fns"]
    cost_fn_map = {"dtc": cliff_map_file, "cliff": cliff_map_file, "intensity": intensity_map_file,
                   "gmmt": gmmt_map_file}

    mpb["mod.weight_dtc"] = 0.2
    mpb["mod.weight_cliff"] = 1.0
    mpb["mod.weight_gmmt"] = 1.0
    mpb["mod.weight_intensity"] = 2.0

    for sgs in setup['sg']:
        print("****************************************")
        print("********* Running: {} *********".format(sgs["name"]))
        print("****************************************")

        mpbs = dict()
        result_file_names = []

        mpb.set_start(sgs["start"][0],sgs["start"][1],sgs["start"][2])
        mpb.set_goal(sgs["goal"][0],sgs["goal"][1],sgs["goal"][2])
        results_folder_prefix = sgs["name"]

        try:
            os.mkdir(results_folder_prefix)
        except FileExistsError as fxe:
            print("Folder {} exists, not creating...".format(results_folder_prefix))

        for cost_fn in cost_fns:

            if "uniform" in sampling_functions:
                uniform_mpb = deepcopy(mpb)
                uniform_mpb.set_planners(['rrt_star'])
                uniform_mpb["ompl.sampler"] = ""
                uniform_mpb.set_id('{}-{}'.format(cost_fn, 'uniform'))
                uniform_mpb["ompl.intensity_map_file_name"] = intensity_map_file
                uniform_mpb["ompl.optimization_objective"] = cost_fn
                uniform_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
                mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'uniform')] = uniform_mpb
                result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'uniform'))

            if "ellipse" in sampling_functions:
                ellipse_mpb = deepcopy(mpb)
                ellipse_mpb["ompl.sampler"] = "ellipse"
                ellipse_mpb.set_id('{}-{}'.format(cost_fn, 'ellipse'))
                ellipse_mpb["ompl.intensity_map_file_name"] = intensity_map_file
                ellipse_mpb["ompl.optimization_objective"] = cost_fn
                ellipse_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
                mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'ellipse')] = ellipse_mpb
                result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'ellipse'))

            if "intensity" in sampling_functions:
                intensity_mpb = deepcopy(mpb)
                intensity_mpb["ompl.sampler"] = "intensity"
                intensity_mpb.set_id('{}-{}'.format(cost_fn, 'intensity'))
                intensity_mpb["mod.sampling_bias"] = 0.5
                intensity_mpb["ompl.intensity_map_file_name"] = intensity_map_file
                intensity_mpb["ompl.optimization_objective"] = cost_fn
                intensity_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
                mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'intensity')] = intensity_mpb
                result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'intensity'))

            if "dijkstra" in sampling_functions:
                dijkstra_mpb = deepcopy(mpb)
                dijkstra_mpb["ompl.sampler"] = "dijkstra"
                dijkstra_mpb["mod.dijkstra_cell_size"] = 0.25
                dijkstra_mpb["mod.sampling_bias"] = 0.05
                dijkstra_mpb.set_id('{}-{}'.format(cost_fn, 'dijkstra'))
                dijkstra_mpb["ompl.intensity_map_file_name"] = intensity_map_file
                dijkstra_mpb["ompl.optimization_objective"] = cost_fn
                dijkstra_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
                mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'dijkstra')] = dijkstra_mpb
                result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'dijkstra'))

        for key in mpbs:
            print("Running {}".format(key))
            mpbs[key].run(id=key, runs=int(setup['repeats']), subfolder=os.getcwd() + "/" + results_folder_prefix)



