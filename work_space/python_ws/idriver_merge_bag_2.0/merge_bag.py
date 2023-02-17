#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

from fnmatch import fnmatchcase
from rosbag import Bag
import os

class MergeBag:

    def swapbag(self,filename):
        f=open(filename,"rb+")
        data=f.read(128)
        f.seek(0,0)
        f.write(data[::-1])
        f.close()
    
    # def findfiles(self,path):
    #     result = []
    #     list_dirs=os.walk(path)
    #     for root ,dirs,files,in list_dirs:
    #         # print(root)
    #         # for d in dirs:
    #         #     print(os.path.join(root,d))
    #         for f in files:
    #             result.append(f)
    #     return result

    def findfiles(self,path):#zhi bian li yi ji mu lu
        result = []
        file_list = os.listdir(path)
        for file in file_list:
            real_url = os.path.join(path , file)#jue dui lujing
            if os.path.isfile(real_url):
                result.append(file)
        return result

    
    def matchfile(self,files,filehead,filesign,filetime,filetail,timeerror):
        filematch=''
        time=int(filetime)-timeerror
        while time<=int(filetime)+timeerror:
            file=filehead+filesign+str(time)+"_"+filetail
            if file in files:
                filematch=file
                break
            time+=1
        return filematch




    def merge_bag(self,outputbag,inputbag,verbose):
        # str_topics='/driver/lidar/rs_80_packets/top_center /driver/lidar/rs_bp_packets/front /driver/lidar/rs_bp_packets/left /driver/lidar/rs_bp_packets/rear /driver/lidar/rs_bp_packets/right /miivii_gmsl_ros_node_A/camera/compressed /tppcican /tpimu /tpars0 /tpars1'
        # topics = str_topics.split(' ')
        total_included_count = 0
        total_skipped_count = 0
    
        if (verbose):
            print(">> Writing bag file: " + outputbag)
            # print("Matching topics against patters: '%s'" % ' '.join(topics))
    
        with Bag(outputbag, 'w') as o:
            writedtopics=[]
            for ifile in inputbag:
                matchedtopics = [] 
                # included_count = 0
                # skipped_count = 0
                if (verbose):
                    print("< Reading bag file: " + ifile)
                with Bag(ifile, 'r') as ib:
                    for topic, msg, t in ib:
                        # if any(fnmatchcase(topic, pattern) for pattern in topics):
                        if not topic in writedtopics:
                            if not topic in matchedtopics:
                                matchedtopics.append(topic)
                                if (verbose):
                                    print("write topic '%s'" % topic)
                            o.write(topic, msg, t)
                writedtopics=writedtopics+matchedtopics
                    # print(writedtopics)
                        #     included_count += 1
                        # else:
                        #     skipped_count += 1
                # total_included_count += included_count
                # total_skipped_count += skipped_count
                # if (verbose):
                #     print("< Included %d messages and skipped %d" % (included_count, skipped_count))
    
        # if (verbose):
        #     print("Total: Included %d messages and skipped %d" % (total_included_count, total_skipped_count))
 

