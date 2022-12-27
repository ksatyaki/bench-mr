from copy import deepcopy
from mpb import MPB
from tqdm.gui import tqdm_gui
import argparse
import json
import glob


parser = argparse.ArgumentParser(
    description="Merge the combined results files from different runs")
parser.add_argument("-f", "--folder", nargs="*", type=str,
                    help="The folder(s) where the source files are stored.")
parser.add_argument("-p", "--prefix", help="prefix to add to final results file.")
args = parser.parse_args()

prefix = "" if not args.prefix else args.prefix

if not args.folder:
    parser.print_help()
    print("Folder is a required option!")
    exit(-1)

files = []
for folder in args.folder:
    files.append("{}/combined.json".format(folder))
    print("Running merge with files={}".format(files))

MPB.merge(files, target_filename="{}-combined.json".format(prefix), make_separate_runs=True)
print("DONE!")
exit(0)
