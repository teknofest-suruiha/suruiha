#!/usr/bin/env python
import sys, random, math


def calc_dist(pose1, pose2):
    xdiff = pose1[0]-pose2[0]
    xdiff = math.pow(xdiff, 2)
    ydiff = pose1[1]-pose2[1]
    ydiff = math.pow(ydiff, 2)
    return math.sqrt(xdiff+ydiff)


def is_valid(new_pos, model_poses, threshold=100):
    for mymodel in model_poses:
        dist = calc_dist(mymodel['position'], new_pos)
        if dist < threshold:
            return False
    
    return True
        

if __name__ == "__main__":
    building_model_names = ['apartment',
                            'cafe',
                            'fire_station',
                            'gas_station',
                            'grocery_store',
                            'house_1',
                            'house_2',
                            'house_3',
                            'law_office',
                            'office_building',
                            'osrf_first_office',
                            'parking_garage',
                            'playground',
                            'police_station',
                            'post_office',
                            'powerplant',
                            'radio_tower',
                            'reactor',
                            'school',
                            'water_tower']
    
    num_model = int(sys.argv[1])
    width = int(sys.argv[2])
    height = int(sys.argv[3])
    
    model_poses = []
    while len(model_poses) < num_model:
        # choose a random model
        model_idx = random.randint(0, len(building_model_names)-1)
        new_pos = [random.randint(-width/2, width/2), random.randint(-height/2, height/2)]
        if is_valid(new_pos, model_poses):
            mymodel = {}
            mymodel['name'] = building_model_names[model_idx]
            mymodel['position'] = new_pos
            model_poses.append(mymodel)
    
    # print all
    counter = 0
    for mymodel in model_poses:
        name = mymodel['name']
        pos = mymodel['position']        
        print('<model name="'+name+'_'+str(counter)+'">')
        print('  <include>')
        print('    <uri>model://'+name+'</uri>')        
        print('    <pose>'+str(pos[0])+' '+str(pos[1])+' 0 0 0 0</pose>')
        print('  </include>')
        print('</model>')
        counter += 1
        
#    for model_name in building_model_names:
#        print('<uri>file://'+model_name+'</uri>')
        
    actor_num = int(sys.argv[4])
    max_sim_time = int(sys.argv[5])
    max_human_speed = 1.0
    min_human_speed = 0.5
    random_waypoint_dist = 3
    
    for i in range(actor_num):
        start_time = random.randint(0, max_sim_time)
        num_waypoint = random.randint(0, 100)
        start_building_index = random.randint(0, num_model-1)
        actor_waypoints = []
        waypoint = {}
        waypoint['time'] = start_time
        waypoint['position'] = model_poses[start_building_index]['position']
        actor_waypoints.append(waypoint)
        while len(actor_waypoints) < num_waypoint:
            randx = random.randint(-random_waypoint_dist, random_waypoint_dist) + actor_waypoints[len(actor_waypoints)-1]['position'][0]
            randy = random.randint(-random_waypoint_dist, random_waypoint_dist) + actor_waypoints[len(actor_waypoints)-1]['position'][1]
            p = [randx, randy]
            dist_to_prev_waypoint = calc_dist(p, actor_waypoints[len(actor_waypoints)-1]['position'])
            randspeed = random.uniform(min_human_speed, max_human_speed)
            next_time = actor_waypoints[len(actor_waypoints)-1]['time'] + (dist_to_prev_waypoint/randspeed)
            if next_time > max_sim_time:
                break
            waypoint = {}
            waypoint['time'] = next_time
            waypoint['position'] = [randx, randy]
            actor_waypoints.append(waypoint)

        print('<actor name="actor_'+str(i)+'">')
        print('<pose>' + str(actor_waypoints[0]['position'][0]) + ' ' + str(actor_waypoints[0]['position'][1]) + ' 1.25 0 0 0</pose>')
        print('<skin><filename>model://actor/meshes/SKIN_man_red_shirt.dae</filename></skin>')
        print('<animation name="animation">')
        print('<filename>model://actor/meshes/ANIMATION_walking.dae</filename>')
        print('<interpolate_x>true</interpolate_x>')
        print('</animation>')
        print('<script><trajectory id="'+str(i)+'" type="walking">')
        
        for j in range(len(actor_waypoints)):
            waypoint = actor_waypoints[j]
            print('<waypoint>')
            print('<time>' + str(waypoint['time']) + '</time>')
            print('<pose>' + str(waypoint['position'][0]) + ' ' + str(waypoint['position'][1]) + ' 1.25 0 0 0</pose>')
            print('</waypoint>')
            #print(str(j) + ' time:' + str(waypoint['time']) + ' position:' + str(waypoint['position']))
        print('</trajectory></script></actor>')
        
            
