#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

import sys
import argparse
from merge_bag import MergeBag

def main():
    parser = argparse.ArgumentParser(description='Merge Bags')
    parser.add_argument('output_dir',
                        help='output bag dir')
    parser.add_argument('input_dir',
                        help='input bag dir')
    parser.add_argument('-v', '--verbose', action="store_true", default=False,
                        help='verbose output')
    args = parser.parse_args()

    timeerror=1
    merge_bag_util=MergeBag()
    # merge_bag_util.merge_bag(args.outputbag,args.inputbag,args.verbose)
    files=merge_bag_util.findfiles(args.input_dir)
    files112=[]
    # get all 112 files
    for file in files:
        filenames=file.split("_")
        if len(filenames)==4 and '112ex' in filenames:
            files112.append(file)
    if len(files112)==0:
        print(">>> not find bag files in {}".format(args.input_dir))
        return
    
    for file112 in files112:
        filenames=file112.split("_")
        inputbag=[]
        outputbag=args.output_dir+filenames[0]+"_"+filenames[2]+"_"+filenames[3]
        inputbag.append(args.input_dir+file112)

        file113=merge_bag_util.matchfile(files, filenames[0], "_113ex_", filenames[2], filenames[3], timeerror)
        if file113!="":
            inputbag.append(args.input_dir+file113)
        file114=merge_bag_util.matchfile(files, filenames[0], "_114ex_", filenames[2], filenames[3], timeerror)
        if file114!="":
            inputbag.append(args.input_dir+file114)
        file115=merge_bag_util.matchfile(files, filenames[0], "_115ex_", filenames[2], filenames[3], timeerror)
        if file115!="":
            inputbag.append(args.input_dir+file115)
        merge_bag_util.merge_bag(outputbag, inputbag, args.verbose)


if __name__ == "__main__":
    main()
