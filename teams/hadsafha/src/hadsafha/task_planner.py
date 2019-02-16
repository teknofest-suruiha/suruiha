# task status
TAKEOFF = 'TAKEOFF'
WAYPOINTS = 'WAYPOINTS'
LAND = 'LAND'
IDLE = 'IDLE'
WAIT = 'WAIT'

class TaskPlanner:

    def __init__(self, uav_controller, uav_name):
        self.controller = uav_controller
        self.uav_name = uav_name
        self.status = TAKEOFF
        self.way_points = []
        self.completed_way_points = []


    def add_waypoint(self, point):
        self.way_points.append(point)

    def delete_waypoint(self, index):
        self.way_points.delete(index)

    def reset_waypoints(self):
        del self.way_points[:]

    def get_waypoints(self):
        return self.way_points

    def step(self):


        if self.status == TAKEOFF:
            if len(self.way_points) is not 0:
                # print('task planner takeoff')
                completed = self.controller.takeoff(30)
                if completed:
                    self.status = WAYPOINTS
            else:
                print("Kalkis icin uygun ama bir rota belirlenmemis")

        elif self.status == WAYPOINTS:
            print('task planner waypoints')
            print(self.way_points)
            way_point = self.way_points[0]
            completed = False
            if self.uav_name.find('zephyr') >= 0:
                completed = self.controller.goto_position(way_point[0], way_point[1], way_point[2], 400)
            elif self.uav_name.find('iris') >= 0:
                completed = self.controller.goto_position(way_point[0], way_point[1], way_point[2], 440)
            if completed:

                lastest = self.way_points[0]
                self.completed_way_points.append(lastest)
                self.way_points.remove(lastest)

                if len(self.way_points) is 0:
                    #self.status = LAND
                    self.status = WAIT

        elif self.status == WAIT:
            print('end of way points.refreshing')

            if len(self.way_points) is 0:
                wait_point = self.completed_way_points[-1]

                if self.uav_name.find('zephyr') >= 0:
                    self.controller.goto_position(wait_point[0],wait_point[1], wait_point[2], 400)
                    #self.controller.loiter(20,wait_point[2])
                elif self.uav_name.find('iris') >= 0:
                    self.controller.goto_position(wait_point[0], wait_point[1], wait_point[2], 440)

                self.way_points = self.completed_way_points
                self.completed_way_points = []

            else:
                self.status = WAYPOINTS

        elif self.status == LAND:
            completed = self.controller.land()
            if completed:
                self.status = IDLE
        elif self.status == IDLE:
            print('idle state')
            self.controller.stop_motors()
            # pass
            # do nothing





