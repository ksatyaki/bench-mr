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

if not (args.cost_fn and args.sampling_fn and args.folder):
        parser.print_help()
        print("All three arguments are needed!")
        exit(-1)

MPB.merge_separate_planners(sampfns=args.sampling_fn, costfns=args.cost_fn, folders=args.folder)