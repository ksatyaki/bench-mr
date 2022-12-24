from mpb import MPB

import argparse
import glob

parser = argparse.ArgumentParser(
    description="Merge results files from the MPBs by specifying cost function and sampling function")
parser.add_argument("-c", "--cost-fn", nargs="*", type=str, help="A list of cost functions (space separated).")
parser.add_argument("-s", "--sampling-fn", nargs="*", type=str, help="A list of sampling functions (space separated).")
parser.add_argument("-f", "--folder", nargs="*", type=str,
                    help="The folder(s) where the source and destination result files are stored.")
args = parser.parse_args()

cost_fns = args.cost_fn if args.cost_fn else ["cliff", "dtc", "gmmt", "intensity"]
sampling_fns = args.sampling_fns if args.sampling_fns else ["uniform", "ellipse", "dijkstra", "intensity", "hybrid"]

if not args.folder:
    parser.print_help()
    print("Folder is a required option!")
    exit(-1)

files = []

for folder in args.folder:
    for cost_fn in cost_fns:
        for sampling_fn in sampling_fns:
            filename = glob.glob("{}/*{}-{}_results.json".format(folder, cost_fn, sampling_fn))[0]
            files.append(filename)
            print()

print("Running merge with files={}, target_filename=combined.json".format(files))
MPB.merge(files, target_filename="combined.json".format(folder), make_separate_runs=True)

print("DONE!")
exit(0)
