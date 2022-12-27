from copy import deepcopy
from mpb import MPB
from tqdm.gui import tqdm_gui
import argparse
import json
import glob


parser = argparse.ArgumentParser(
    description="Merge results files from the MPBs by specifying cost function and sampling function")
parser.add_argument("-c", "--cost-fn", nargs="*", type=str, help="A list of cost functions (space separated).")
parser.add_argument("-s", "--sampling-fn", nargs="*", type=str, help="A list of sampling functions (space separated).")
parser.add_argument("-f", "--folder", nargs="*", type=str,
                    help="The folder(s) where the source and destination result files are stored.")
args = parser.parse_args()

cost_fns = args.cost_fn if args.cost_fn else ["cliff", "dtc", "gmmt", "intensity"]
sampling_fns = args.sampling_fn if args.sampling_fn else ["uniform", "ellipse", "dijkstra", "intensity", "hybrid"]

if not args.folder:
    parser.print_help()
    print("Folder is a required option!")
    exit(-1)

files = []

for folder in tqdm_gui(args.folder, unit=" folders done"):
    MPB.rename_planner_using_filename(sampfns=sampling_fns, costfns=cost_fns, folder=folder)

MPB.merge_separate_planners(sampfns=sampling_fns, costfns=cost_fns, folders=args.folder, pattern="_renamed.json")

print("DONE!")
exit(0)
