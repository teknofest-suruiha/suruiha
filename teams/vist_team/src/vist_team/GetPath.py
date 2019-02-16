#!/usr/bin/env python
# -*- coding: utf-8 -*-


#       UNUTMA --->     0. indisli noktalar -,- den başlıyor sonrasında +,+ 'ya doğru gidiyor
def GetPath(area_size, height, runway, num_uavs,index):
    
    def divide(length, n, gap):
        result = []
        for i in range(n+1):                            # iki uç noktayı da alabilmek için
            result.append(-length/2 + i*gap)   
        
        return result

    cam_size = height/2
    gap = cam_size/2
    

       
    x = area_size.get("width")
    y = area_size.get("height")
    m = int(x/gap)
    n = int(y/gap)
    
    right_top_x = runway.get("right_top").get("x")
    lef_bottom_x = runway.get("left_bottom").get("x")
    right_top_y = runway.get("right_top").get("y")
    lef_bottom_y = runway.get("left_bottom").get("y")
    
#    nearest_start = [(right_top_x + lef_bottom_x)/2, (right_top_y + lef_bottom_y)/2]
    mid_runway = [(right_top_x + lef_bottom_x)/2, (right_top_y + lef_bottom_y)/2]
    
 
    #   x ve y yönünde bölme
    x_points = divide(x, m, gap)
    y_points = divide(y, n, gap)
    z = height
    
    if x%gap != 0:
        x_points.append(x_points[-1] + x%gap)
            
    if y%gap != 0:
        y_points.append(y_points[-1] + y%gap)
     
    
    
    if mid_runway[0] < 0 and mid_runway[1] < 0:
        x_points = list(reversed(x_points))
        y_points = y_points
        
    elif mid_runway[0] > 0 and mid_runway[1] < 0:
        x_points = list(reversed(x_points))
        y_points = list(reversed(y_points))
        
    elif mid_runway[0] > 0 and mid_runway[1] < 0:
        x_points = x_points
        y_points = list(reversed(y_points))
   
    
      
    #                                       PATH   
    path = []
    corner = True
    


    for i in range(num_uavs):
        way_points = []

        for j in range(n-1-3*i, -1, -2*num_uavs):  # indis 0'dan başladığı için i+1
            if corner:
                x_ind = 0
                x_ind_comp = m
            elif not corner:
                x_ind = m
                x_ind_comp = 0
            
            point1 = [x_points[x_ind], y_points[j], z]        
            point2 = [x_points[x_ind_comp], y_points[j], z]       
            way_points.append(point1); way_points.append(point2) 
                             
            corner = not corner
        
        
            #if index != 0 or index != 1:
                #del way_points[0]
        
        
        path.append(way_points)
        
        
        
    return path
        
        
    
