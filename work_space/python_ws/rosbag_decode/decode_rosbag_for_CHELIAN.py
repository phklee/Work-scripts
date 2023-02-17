#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

import sys
import os
import csv
import rosbag
import rospy
import numpy as np
import math


bagpath = sys.argv[1]
directory, filename = os.path.split(bagpath)
# extension = ""
if not bagpath.endswith(".bag"):
    extension = ".bag"
# filename = filename + extension
# bagPath = os.path.join()
bag = rosbag.Bag(bagpath)
print("Reading the rosbag file: " + filename)


###############################################
def make_keyinfo():
    with open(directory + "/" + filename[:-4]+'_keyinfo.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        line1 = ['systemTime', 'lidarrev_time', 'frame_cnt', 'tf', 'lidar_thread_stime', 'lidar_thread_etime', 'lidardet_time_used']  # lidar
        line2 = ['tf', 'dogm_cloudtime', 'dogm_thread_stime', 'dogm_thread_etime', 'dogmdet_time_used'] # dogm
        line3 = ['tf', 'visiondet_imgframetime'] # vision
        line4 = ['tf', 'lidar2cam_thread_stime', 'lidar2cam_thread_etime', 'lidar2cam_time_used',] # lidar2cam
        line5 = ['tf', 'radar2cam_thread_stime', 'radar2cam_thread_etime', 'radar2cam_time_used',] # radar2cam
        line6 = ['tf', 'fusion_thread_stime', 'fusion_thread_etime', 'fusion_time_used',] # fusion
        line7 = ['l2c_findSyncVisDet', 'l2c_findSyncVisSem', 'l2c_findSyncLandet', 'r2c_findSyncVisDet', 'r2c_findSyncVisSem', 
                 'dogm_findSyncDogm', 'dogm_findSyncVisSem']

        data_writer.writerow(line1+line2+line3+line4+line5+line6+line7)

        # Get all message
        for topic, msg, t in bag.read_messages(topics=['/fusion_debug']):
            system_time = msg.sysTime
            obs_time_stamp = msg.obsTime
            frameId = msg.frame
            k = msg.keyinfo

            data1 = [system_time, k.lidarrev_time, k.frame_cnt, 'lidar', k.lidar_thread_stime, k.lidar_thread_etime, k.lidardet_time_used]
            data2 = ['dogm', k.dogm_cloudtime, k.dogm_thread_stime, k.dogm_thread_etime, k.dogmdet_time_used]
            data3 = ['vision', k.visiondet_imgframetime]
            data4 = ['lidar2cam', k.lidar2cam_thread_stime, k.lidar2cam_thread_etime, k.lidar2cam_time_used]
            data5 = ['radar2cam', k.radar2cam_thread_stime, k.radar2cam_thread_etime, k.radar2cam_time_used]
            data6 = ['fusion', k.fusion_thread_stime, k.fusion_thread_etime, k.fusion_time_used]
            data7 = [k.l2c_findSyncVisDet, k.l2c_findSyncVisSem, k.l2c_findSyncLandet, k.r2c_findSyncVisDet, k.r2c_findSyncVisSem, 
                     k.dogm_findSyncDogm, k.dogm_findSyncVisSem]

            data_writer.writerow(data1+data2+data3+data4+data5+data6+data7)

###############################################
def make_profile():
    with open(directory + "/" + filename[:-4]+'_profile.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        line1 = ['systemTime', 'lidarTime', 'p_cnt', 'tf', 'timestamp_thread_start', 'time_used_lidar']
        line2 = ['tf', 'dogm_timestamp_lidar', 'dogm_time_bak', 'findSyncDogm', 'dogm_timestamp', 'dogm_timestamp_callback',
                 'dogm_findSyncSemantic', 'dogm_timestamp_semantic']
        line3 = ['tf', 'time_bak_image', 'time_bak_semantic', 'time_bak_landet', 'time_used_lidar2cam']
        line4 = ['findSyncImage', 'dt_ms(no_sure)', 'timestamp_image', 'timestamp_image_callback']
        line5 = ['vision_findSyncSemantic', 'dt_ms', 'vision_timestamp_semantic', 'timestamp_semantic_callback']
        line6 = ['findSyncLandet', 'dt_ms(no_sure)', 'timestamp_landet', 'timestamp_landet_callback']

        data_writer.writerow(line1+line2+line3+line4+line5+line6)

        # Get all message
        for topic, msg, t in bag.read_messages(topics=['/fusion_debug']):
            system_time = msg.sysTime
            obs_time_stamp = msg.obsTime
            frameId = msg.frame
            p = msg.profile

            data1 = [system_time, p.lidar_frame_time, p.frame, 'lidar', p.lidar_thread_start, p.lidar_time_used]
            data2 = ['dogm', p.dogm_timestamp_lidar, p.dogm_time_bak, p.dogm_findSyncDogm,
                     p.dogm_timestamp, p.dogm_timestamp_callback, p.dogm_findSyncSemantic, p.dogm_timestamp_semantic]
            data3 = ['vision', p.l2c_time_bak_image, p.l2c_time_bak_semantic, p.l2c_time_bak_landet, p.l2c_time_used_lidar2cam]
            data4 = [p.l2c_findSyncImage, (p.dogm_timestamp_lidar-p.l2c_timestamp_image), p.l2c_timestamp_image,
                     p.l2c_timestamp_image_callback]
            data5 = [p.l2c_findSyncSemantic, (p.dogm_timestamp_lidar-p.l2c_timestamp_semantic), p.l2c_timestamp_semantic,
                     p.l2c_timestamp_semantic_callback]
            data6 = [p.l2c_findSyncLandet, (p.dogm_timestamp_lidar-p.l2c_timestamp_landet), p.l2c_timestamp_landet,
                     p.l2c_timestamp_landet_callback]

            data_writer.writerow(data1+data2+data3+data4+data5+data6)

###############################################
def make_fusion_debug():
    with open(directory + "/" + filename[:-4]+'_fusion.csv', mode='w') as data_file:

        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        line1 = ['systime', 'p_cnt', 'tf', 'trk_id', 'type', 'age', 'x', 'y', 'x_center_vcs', 'y_center_vcs', 'vx', 'vy', 'v_heading', 'speed',
                'is_trk_static', 'velo_status', 'velo_static_cnt', 'dor_static_cnt']

        data_writer.writerow(line1)

        # Get all message
        for topic, msg, t in bag.read_messages(topics=['/fusion_debug']):
            system_time = msg.sysTime
            frameId = msg.frame
            p = msg.profile
            loc = msg.locpos

            data16 = [loc.toa, loc.xg, loc.yg, loc.zg, loc.yawrate, loc.speed, loc.yaw, loc.roll, loc.pitch]
            data17 = [loc.dr_toa, loc.dr_x, loc.dr_y, loc.dr_z, loc.dr_roll, loc.dr_pitch, loc.dr_yaw]

            for trk in msg.tracks:
                data1 = [system_time, frameId, trk.tf, trk.id, trk.type, trk.age,
                              trk.x, trk.y, trk.x_center_vcs, trk.y_center_vcs, trk.vx, trk.vy, math.atan2(trk.vy,trk.vx)/3.14*180, math.sqrt(math.pow(trk.vx, 2)+math.pow(trk.vy, 2)),
                              trk.is_trk_static, trk.velocitystatus, trk.velo_static_cnt, trk.dor_static_cnt]
                data_writer.writerow(data1)

###############################################
def make_raw_inputs():
    with open(directory + "/" + filename[:-4]+'_raw_inputs.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        line1 = ['systime', 'p_cnt', 'source', 'id', 'is_dogm_unique']
        line2 = ['x_vcs', 'y_vcs', 'vx_vcs', 'vy_vcs', 'heading_vcs', 'x_dr', 'y_dr', 'vx_dr', 'vy_dr',
                 'length', 'width', 'height', 'type', 'confi']
        line3 = ['dogm_velo', 'dogm_vxabs', 'dogm_vyabs', 'dogm_v_heading', 'dogm_dynamic_cnt', 'dogm_static_cnt', 'is_dogm_static']
        line4 = ['totalCamObjs', 'isTrueObj', 'norm_type', 'geo_isvalid', 'geo_distance']
        line5 = ['semantic_noise','semantic_valid', 'sem_total_pts', 'sem_background_pts', 'sem_road_curb_pts', 'sem_fense_pts',
                 'sem_roadblock_pts', 'sem_roaduser_car_pts', 'sem_roaduser_other_pts']
        line6 = ['top_x_raw', 'top_y_raw', 'top_x_dr', 'top_y_dr', 'bot_x_raw', 'bot_y_raw', 'bot_x_dr', 'bot_y_dr']
        line7 = ['loc_time', 'loc_xg', 'loc_yg', 'loc_zg', 'loc_yawrate', 'loc_speed', 'loc_yaw', 'loc_roll', 'loc_pitch']
        line8 = ['dr_time', 'dr_x', 'dr_y', 'dr_z', 'dr_roll', 'dr_pitch', 'dr_yaw']

        data_writer.writerow(line1+line2+line3+line4+line5+line6+line7+line8)

        # Get all message
        for topic, msg, t in bag.read_messages(topics=['/fusion_debug']):
            system_time = msg.sysTime
            obs_time_stamp = msg.obsTime
            frameId = msg.frame
            loc = msg.locpos
            inputs = msg.inputs

            data7 = [loc.toa, loc.xg, loc.yg, loc.zg, loc.yawrate, loc.speed, loc.yaw, loc.roll, loc.pitch]
            data8 = [loc.dr_toa, loc.dr_x, loc.dr_y, loc.dr_z, loc.dr_roll, loc.dr_pitch, loc.dr_yaw]

            for ipt in inputs:
                data1 = [system_time, frameId, ipt.source, ipt.id, ipt.is_dogm_unique]
                data2 = [ipt.x_vcs, ipt.y_vcs, ipt.vx_vcs, ipt.vy_vcs, ipt.heading_vcs, ipt.x_dr, ipt.y_dr, ipt.vx_dr, ipt.vy_dr,
                         ipt.length, ipt.width, ipt.height, ipt.type, ipt.lidarConfi]

                noise = ''
                if ipt.sem_isTrueObj == False:
                    noise = 'semantic'

                data3 = [ipt.dogm_info.velocity, ipt.dogm_info.vxabs, ipt.dogm_info.vyabs, ipt.dogm_info.v_heading,
                         ipt.dogm_info.dynamic_cnt, ipt.dogm_info.static_cnt, ipt.dogm_info.is_dogm_static]
                data4 = [ipt.l2c_totalCamObjs, ipt.isTrueObj, noise, ipt.geo_isvalid, ipt.geo_distance]
                data5 = [noise, ipt.sem_is_valid, ipt.sem_total_cloud_pts, ipt.sem_background_pts, ipt.sem_road_curb_pts,
                         ipt.sem_fense_pts, ipt.sem_roadblock_pts, ipt.sem_roaduser_car_pts, ipt.sem_roaduser_other_pts]
                data6 = [ipt.bbox_vcs_tx, ipt.bbox_vcs_ty, ipt.bbox_dr_tx, ipt.bbox_dr_ty, ipt.bbox_vcs_bx, ipt.bbox_vcs_by,
                         ipt.bbox_dr_bx, ipt.bbox_dr_by]

                data_writer.writerow(data1+data2+data3+data4+data5+data6+data7+data8)

###############################################
def make_mapengine():
    with open(directory +"/"+ filename[:-4]+'_mapengine.csv', mode='w') as data_file:
      data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
      line = ['time_header','lineIdx','pointSize','idx','x','y','angle','xg','yg','angleglobal']
      data_writer.writerow(line)

      # Get all message
      for topic, msg, t in bag.read_messages(topics=['/mapengine/tpnavigation']):
          time_header = math.ceil(msg.header.time_stamp * 100)/100
          lineIdx = msg.map_info.current_line_index
          points = msg.map_info.alllinelists[lineIdx].frontline.map_points
          cnt = 0
          for pt in points:
              x = pt.x
              y = pt.y
              angle = pt.angle
              xg = pt.xg
              yg = pt.yg
              angleglobal = pt.angleglobal
              if x < 70:
                  data = [time_header,lineIdx,len(points),cnt,x,y,angle,xg,yg,angleglobal]
                  data_writer.writerow(data)
              cnt = cnt + 1

###############################################
def make_tpperception():
    target_cell_list = []
    with open(directory + "/" + filename[:-4]+'_perception_output.csv', mode='w') as data_file:

        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        line1 = ['systime', 'obj_time', 'id', 'type', 'age', 'x', 'y', 'z', 'vxrel', 'vyrel', 'width', 'length',
                 'height', 'confi', 'cell_size', 'fuse_source']
        line2 = ['xabs','yabs', 'vxabs', 'vyabs', 'speed', 'is_ultra_static', 'moving_status', 'heading', 'v_heading']

        data_writer.writerow(line1+line2)

        for topic, msg, t in bag.read_messages(topics=['/tpperception']):
            systime = msg.header.time_stamp
            obs_time_stamp = msg.obstacle_info.header.time_stamp

            objects = msg.obstacle_info.objs
            for obj in objects:
                obj_data = [obs_time_stamp, obj.id, obj.age, obj.type, obj.x, obj.y, obj.z, obj.vxrel, obj.vyrel, obj.xabs,
                            obj.yabs, obj.vxabs, obj.vyabs, obj.width, obj.length, obj.height, obj.speed, 
                            obj.is_ultra_static, obj.moving_status, obj.heading]

                cells = obj.cells
                xg_max = obj.xabs
                xg_min = obj.xabs
                yg_max = obj.yabs
                yg_min = obj.yabs
                for c in cells:
                    if c.xg > xg_max:
                        xg_max = c.xg
                    if c.xg < xg_min:
                        xg_min = c.xg
                    if c.yg > yg_max:
                        yg_max = c.yg
                    if c.yg < yg_min:
                        yg_min = c.yg

                # get maxi yrel in cells
                x_max = obj.x
                y_max = obj.y
                x_min = obj.x
                y_min = obj.y
                for c in cells:
                    if c.y > y_max:
                        y_max = c.y
                        x_max = c.x
                    if c.y < y_min:
                        y_min = c.y
                        x_min = c.x
                    if c.x > x_max:
                        x_max = c.x
                    if c.x < x_min:
                        x_min = c.x
                cells_data = [len(cells), xg_max, xg_min, yg_max,
                              yg_min, x_max, x_min, y_max, y_min]

                if obj.source == 0:
                    fuse_source = "LIDAR"
                elif obj.source == 1:
                    fuse_source = "RADAR"
                elif obj.source == 2:
                    fuse_source = "LIDAR_VISION"
                elif obj.source == 3:
                    fuse_source = "LIDAR_RADAR"
                elif obj.source == 4:
                    fuse_source = "LIDAR_VISION_RADAR"
                elif obj.source == 5:
                    fuse_source = "DOGM"
                elif obj.source == 6:
                    fuse_source = "LIDAR_DOGM"
                else:
                    fuse_source = "NONE"

                data1 = [systime, obs_time_stamp, obj.id, obj.type, obj.age, obj.x, obj.y, obj.z, obj.vxrel, obj.vyrel,
                         obj.width, obj.length, obj.height, obj.confidence, len(cells), fuse_source]

                data2 = [obj.xabs, obj.yabs, obj.vxabs, obj.vyabs, obj.speed, obj.is_ultra_static, obj.moving_status,
                         obj.heading, math.atan2(obj.vyabs, obj.vxabs)/3.14*180]

                data_writer.writerow(data1+data2)

                # get specific track cells data
                if obj.id == 25009:
                    point = []
                    counter = 0
                    for c in cells:
                        point.append([obs_time_stamp, obj.id, obj.type, obj.age, len(cells), counter, fuse_source, obj.x, obj.y, obj.xabs, 
                                      obj.yabs, obj.heading, obj.length, obj.width, c.x, c.y, c.xg, c.yg, x_max, x_min, y_max, y_min])
                        counter = counter + 1
                    target_cell_list.append(point)

    with open(directory + "/" + filename[:-4]+'_cells.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow(['obj_time', 'id', 'type', 'age', 'cell_size', 'cnt', 'fuse_source', 'x', 'y', 'xabs', 'yabs', 'obj_heading', 
                              'obj_lenght', 'obj_width', 'cells_x', 'cells_y', 'cells_xg', 'cells_yg', 'xmax', 'xmin', 'ymax', 'ymin'])

        for intr in target_cell_list:
            for point in intr:
                data_writer.writerow(point)

def make_vehicle_info():
    with open(directory + "/" + filename[:-4]+'_vehicle_info.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        line1 = ['time_stamp', 'speed']
        data_writer.writerow(line1)

        # Get all message
        for topic, msg, t in bag.read_messages(topics=['/tppcican']):
            time_stamp = msg.header.time_stamp
            data1 = [time_stamp, msg.vehicle_info.speed]
            data_writer.writerow(data1)

###############################################
def main():
    # make_profile()
    make_fusion_debug()
    # make_raw_inputs()
    # make_tpperception()
    # make_keyinfo()
    # make_vehicle_info()
    # make_mapengine()

    print("Finished creating csv file!")
    bag.close()


if __name__ == "__main__":
    main()
