#!/usr/bin/env python
import sys, random, math


def calc_dist(pose1, pose2):
    xdiff = pose1[0]-pose2[0]
    xdiff = math.pow(xdiff, 2)
    ydiff = pose1[1]-pose2[1]
    ydiff = math.pow(ydiff, 2)
    return math.sqrt(xdiff+ydiff)


def is_valid(new_pos, models, threshold=50):
    for mymodel in models:
        dist = calc_dist(mymodel['position'], new_pos)
        if dist < threshold:
            return False
    
    return True
        

if __name__ == "__main__":
    building_model_names = ['apartment',
                            'grocery_store',
                            'house_1',
                            'house_2',
                            'house_3',
                            'law_office',
                            'office_building',
                            'osrf_first_office',
                            'post_office',
                            'school',
                            'fire_station'
                            ]
                            
    other_model_names = ['water_tower', 
    'radio_tower', 'playground', 'gas_station', 
    'parking_garage','police_station','fire_station']
    
    num_building = int(sys.argv[1])
    num_model = int(sys.argv[2])
    width = int(sys.argv[3])-50
    height = int(sys.argv[4])-50
    
    buildings = []
    while len(buildings) < num_building:
        index = random.randint(0, len(building_model_names)-1)
        new_pos = [random.randint(-width/2, width/2), random.randint(-height/2, height/2)]
        if is_valid(new_pos, buildings):
            building_model = {}
            building_model['name'] = building_model_names[index]
            building_model['position'] = new_pos
            building_model['ori'] = random.randint(0, 360)
            buildings.append(building_model)
    
    others = []
    while len(others) < num_model:
        index = random.randint(0, len(other_model_names)-1)
        new_pos = [random.randint(-width/2, width/2), random.randint(-height/2, height/2)]
        if is_valid(new_pos, buildings) and is_valid(new_pos, others):
            other_model = {}
            other_model['name'] = other_model_names[index]
            other_model['position'] = new_pos
            building_model['ori'] = random.randint(0, 360)
            others.append(other_model)
    
    counter = 0
    for model in buildings:
        name = model['name']
        pos = model['position']
        ori = model['ori']
        print('<model name="building_' + str(counter) + '"
        print('  <static>true</static>')
        print('  <pose>'+str(pos[0])+' '+str(pos[1])+' 0 0 0 '+str(ori*2*math.pi/360.0)+'</pose>')
        print('  <include>')
        print('    <uri>model://' + name + '</uri>')
        print('  </include>')
        print('</model>')
        counter += 1
    
    for model in others:
        name = model['name']
        pos = model['position']
        print('<model name="'+name+'_' + str(counter) + '">')
        print('  <static>true</static>')
        print('  <pose>'+str(pos[0])+' '+str(pos[1])+' 0 0 0 0</pose>')
        print('  <include>')
        print('    <uri>model://' + name + '</uri>')
        print('  </include>')
        print('</model>')
        counter += 1
    
        
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
