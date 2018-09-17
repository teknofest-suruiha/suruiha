#!/usr/bin/env python
import sys, random, math
from xml.dom import minidom

if __name__ == "__main__":
    xOffset = float(sys.argv[1])
    yOffset = float(sys.argv[2])
    offsetStr = sys.argv[3]
#    print('xOffset:' + str(xOffset) + ' yOffset:' + str(yOffset))
    mydoc = minidom.parse('/tmp/models.xml')
    models = mydoc.getElementsByTagName('model')
    
    for model in models:
        model.attributes["name"].value = model.attributes["name"].value + offsetStr
        poses = model.getElementsByTagName('pose')
        pose_str = poses[0].childNodes[0].data
        vector_string = pose_str.split(' ')
        vec_values = []
        vec_idx = 0
        for vec_str in vector_string:
            print('vec['+str(vec_idx)+']:' + vec_str)
            vec_values.append(float(vec_str))
            vec_idx += 1
        vec_values[0] += xOffset
        vec_values[1] += yOffset
        vec_str = str(vec_values[0]) + ' ' + str(vec_values[1]) + ' ' + str(vec_values[2])
        poses[0].childNodes[0].data = vec_str
    
    print(mydoc.toprettyxml(indent='  '))
        
#==============================================================================
#     actor_num = int(sys.argv[4])
#     max_sim_time = int(sys.argv[5])
#     max_human_speed = 1.0
#     min_human_speed = 0.5
#     random_waypoint_dist = 3
#     
#     for i in range(actor_num):
#         start_time = random.randint(0, max_sim_time)
#         num_waypoint = random.randint(0, 100)
#         start_building_index = random.randint(0, num_model-1)
#         actor_waypoints = []
#         waypoint = {}
#         waypoint['time'] = start_time
#         waypoint['position'] = model_poses[start_building_index]['position']
#         actor_waypoints.append(waypoint)
#         while len(actor_waypoints) < num_waypoint:
#             randx = random.randint(-random_waypoint_dist, random_waypoint_dist) + actor_waypoints[len(actor_waypoints)-1]['position'][0]
#             randy = random.randint(-random_waypoint_dist, random_waypoint_dist) + actor_waypoints[len(actor_waypoints)-1]['position'][1]
#             p = [randx, randy]
#             dist_to_prev_waypoint = calc_dist(p, actor_waypoints[len(actor_waypoints)-1]['position'])
#             randspeed = random.uniform(min_human_speed, max_human_speed)
#             next_time = actor_waypoints[len(actor_waypoints)-1]['time'] + (dist_to_prev_waypoint/randspeed)
#             if next_time > max_sim_time:
#                 break
#             waypoint = {}
#             waypoint['time'] = next_time
#             waypoint['position'] = [randx, randy]
#             actor_waypoints.append(waypoint)
# 
#         print('<actor name="actor_'+str(i)+'">')
#         print('<pose>' + str(actor_waypoints[0]['position'][0]) + ' ' + str(actor_waypoints[0]['position'][1]) + ' 1.25 0 0 0</pose>')
#         print('<skin><filename>model://actor/meshes/SKIN_man_red_shirt.dae</filename></skin>')
#         print('<animation name="animation">')
#         print('<filename>model://actor/meshes/ANIMATION_walking.dae</filename>')
#         print('<interpolate_x>true</interpolate_x>')
#         print('</animation>')
#         print('<script><trajectory id="'+str(i)+'" type="walking">')
#         
#         for j in range(len(actor_waypoints)):
#             waypoint = actor_waypoints[j]
#             print('<waypoint>')
#             print('<time>' + str(waypoint['time']) + '</time>')
#             print('<pose>' + str(waypoint['position'][0]) + ' ' + str(waypoint['position'][1]) + ' 1.25 0 0 0</pose>')
#             print('</waypoint>')
#             #print(str(j) + ' time:' + str(waypoint['time']) + ' position:' + str(waypoint['position']))
#         print('</trajectory></script></actor>')
#         
#             
#==============================================================================
