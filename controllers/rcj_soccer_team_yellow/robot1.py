# rcj_soccer_player controller - ROBOT Y1

# Feel free to import built-in libraries
import math

# You can also import scripts that you put into the folder with controller
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils


class MyRobot1(RCJSoccerRobot):
    def readSensor(self):
        ############################# Data read
        self.data = self.get_new_data()
        self.robot_pos = self.data[self.name]
        robot1_pos = self.data[self.name[0] + '1']
        robot2_pos = self.data[self.name[0] + '2']
        robot3_pos = self.data[self.name[0] + '3']
        

        self.x_ball = self.data['ball']['x']
        self.y_ball = self.data['ball']['y']
        self.x_robot = self.data[self.name]['x']
        self.y_robot = self.data[self.name]['y']
        self.orientation = (self.data[self.name]['orientation'] * 180)/math.pi
        self.ball_distance = math.sqrt((self.x_ball - self.x_robot) ** 2 + (self.y_ball - self.y_robot) ** 2)
        
        if(self.orientation > 180):
            self.orientation -= 360
        self.lookUp = (self.orientation * 20)/180
        
        distance1 = math.sqrt((self.x_ball - robot1_pos['x']) ** 2 + (self.y_ball - robot1_pos['y']) ** 2)
        distance2 = math.sqrt((self.x_ball - robot2_pos['x']) ** 2 + (self.y_ball - robot2_pos['y']) ** 2)
        distance3 = math.sqrt((self.x_ball - robot3_pos['x']) ** 2 + (self.y_ball - robot3_pos['y']) ** 2)
        
        ############################# Define Gaol Keeper
        max_dist = max(distance1, distance2, distance3)
        if(max_dist == distance1 and self.name[1] == '1'):
            self.role = 'goal keeper'
            self.goalKeeperCnt = 0
        elif(max_dist != distance1 and self.name[1] == '1'):
            if(self.goalKeeperCnt > self.turnTimeout):
                self.role = 'forward'
            else:
                self.goalKeeperCnt += 1
        if(max_dist == distance2 and self.name[1] == '2'):
            self.role = 'goal keeper'
            self.goalKeeperCnt = 0
        elif(max_dist != distance2 and self.name[1] == '2'):
            if(self.goalKeeperCnt > self.turnTimeout):
                self.role = 'forward'
            else:
                self.goalKeeperCnt += 1
        if(max_dist == distance3 and self.name[1] == '3'):
            self.role = 'goal keeper'
            self.goalKeeperCnt = 0
        elif(max_dist != distance3 and self.name[1] == '3'):
            if(self.goalKeeperCnt > self.turnTimeout):
                self.role = 'forward'
            else:
                self.goalKeeperCnt += 1
        
        if(max_dist == distance1):   self.goalKeeperNum = 1
        elif(max_dist == distance1): self.goalKeeperNum = 2
        elif(max_dist == distance1): self.goalKeeperNum = 3

        ############################# Define Passor
        if(self.name[1] == '1'):
            if((distance1 > distance2 and distance1 < distance3) or (distance1 < distance2 and distance1 > distance3)):
                self.passor = True
        if(self.name[1] == '2'):
            if((distance2 > distance3 and distance2 < distance1) or(distance2 < distance3 and distance2 > distance1)):
                self.passor = True
        if(self.name[1] == '3'):
            if((distance3 > distance1 or distance3 < distance2) or(distance3 < distance1 or distance3 > distance2)):
                self.passor = True
        
        min_dist = min(distance1, distance2, distance3)
        if(min_dist == distance1 and self.name[1] == '1'):
            self.passor = False
        if(min_dist == distance2 and self.name[1] == '2'):
            self.passor = False
        if(min_dist == distance3 and self.name[1] == '3'):
            self.passor = False
        
        ############################# Blue and Yellow Goal look calculation
        self.blue_angle, self.robot_angle = self.get_angles(self.blue_goal_pos, self.robot_pos)
        self.yellow_angle, self.robot_angle = self.get_angles(self.yellow_goal_pos, self.robot_pos)
        self.ball_angle, self.robot_angle = self.get_angles(self.data['ball'], self.robot_pos)
        self.direction = utils.get_direction(self.ball_angle)
        if(self.blue_angle > 180):
            self.blue_angle -= 360
        self.look_at_blue = (self.blue_angle * 6) / 180

        if(self.yellow_angle > 180):
            self.yellow_angle -= 360
        self.look_at_yellow = (self.yellow_angle * 6) / 180
        
        ############################# Lack off progress calculation
        if(self.cnt == 0):
            self.last_ball_pos = {'x': self.x_ball, 'y': self.y_ball}
        if(self.cnt > 20): 
            if(abs(self.last_ball_pos['x'] - self.x_ball) < 0.08 and abs(self.last_ball_pos['y'] - self.y_ball) < 0.08):
                self.kickoff = True
            else:
                self.kickoff = False
            self.cnt = 0
        else:
            self.cnt += 1
        
        ############################# Corner Position Check for Pass
        if(self.name[0] == 'Y'):
            if(self.x_ball > 0.2 and (self.y_ball > 0.2 or self.y_ball < -0.2)):
                self.corner = True
            else:
                self.corner = False
        else:
            if(self.x_ball < -0.2 and (self.y_ball > 0.2 or self.y_ball < -0.2)):
                self.corner = True
            else:
                self.corner = False
        
        ############################# Gaol Keeper Y calculation
        m = 0
        if(self.x_ball > self.x2_ball and self.name[0] == 'Y'):
            self.y_goal_keeper = self.y_ball
        elif(self.x_ball < self.x2_ball and self.name[0] == 'B'):
            self.y_goal_keeper = self.y_ball
        elif(self.x_ball - self.x2_ball != 0):
            m = (self.y_ball - self.y2_ball)/(self.x_ball - self.x2_ball)
            if(self.name[0] == 'Y'):
                self.y_goal_keeper = m*(self.yellow_goal_pos['x'] - self.x_ball) + self.y_ball
            else:
                self.y_goal_keeper = m*(self.blue_goal_pos['x'] - self.x_ball) + self.y_ball
        else:
            self.y_goal_keeper = self.y_ball
        
        if(self.y_goal_keeper > 0.5): 
            self.y_goal_keeper = 0.5
        if(self.y_goal_keeper < -0.5): 
            self.y_goal_keeper = -0.5
        
        ############################# Predict Ball future position
        m = 0
        if(self.x_ball - self.x2_ball != 0):
            m = (self.y_ball - self.y2_ball)/(self.x_ball - self.x2_ball)
        else: 
            m = 99999
        
        if(self.name[0] == 'Y'):
            if(self.x_ball > self.x2_ball):
                self.y_future_ball = m*((self.x_ball - 0.1) - self.x_ball) + self.y_ball
            else:
                self.y_future_ball = m*((self.x_ball - 0.2) - self.x_ball) + self.y_ball
        else:
            if(self.x_ball < self.x2_ball):
                self.y_future_ball = m*((self.x_ball + 0.1) - self.x_ball) + self.y_ball
            else:
                self.y_future_ball = m*((self.x_ball + 0.2) - self.x_ball) + self.y_ball
        
        if(self.y_future_ball > self.y_ball + 0.13): 
            self.y_future_ball = self.y_ball + 0.13
        if(self.y_future_ball < self.y_ball - 0.13): 
            self.y_future_ball = self.y_ball - 0.13
        
        self.ball_speed = math.sqrt((self.x_ball - self.x2_ball)**2 + (self.y_ball - self.y2_ball)**2)

        ############################# Ignore Kick Off and Passor in dangerus positions
        if(self.name[0] == 'Y'):
            if(self.x_ball < -0.65 and self.y_ball > -0.4 and self.y_ball < 0.4):
                self.kickoff = False
                self.passor = False
        else:
            if(self.x_ball > 0.65 and self.y_ball > -0.4 and self.y_ball < 0.4):
                self.kickoff = False
                self.passor = False

        ############################# Avoid multi Passor for one robot inside pass position
        if(self.name[1] == '1'):
            if(robot2_pos['y'] > -0.2 and robot2_pos['y'] < 0.2):
                self.extraPassor = True
            elif(robot3_pos['y'] > -0.2 and robot3_pos['y'] < 0.2):
                self.extraPassor = True
            else:
                self.extraPassor = False
        elif(self.name[1] == '2'):
            if(robot1_pos['y'] > -0.2 and robot1_pos['y'] < 0.2):
                self.extraPassor = True
            elif(robot3_pos['y'] > -0.2 and robot3_pos['y'] < 0.2):
                self.extraPassor = True
            else:
                self.extraPassor = False
        elif(self.name[1] == '3'):
            if(robot2_pos['y'] > -0.2 and robot2_pos['y'] < 0.2):
                self.extraPassor = True
            elif(robot1_pos['y'] > -0.2 and robot1_pos['y'] < 0.2):
                self.extraPassor = True
            else:
                self.extraPassor = False
        
        ############################# Last position of the ball
        self.x2_ball = self.x_ball
        self.y2_ball = self.y_ball 
    def move(self, x, y):
        target = {'x': x, 'y': y}
        angle, robot_angle = self.get_angles(target, self.robot_pos)
        
        # direction 1
        direction = utils.get_direction(angle)
        if(angle > 180):
            angle -= 360
        look_at_target = (angle * 20) / 180

        # direction 2
        angle2 = angle + 180
        if angle2 > 360: angle2 -= 360
        direction2 = utils.get_direction(angle2)
        if(angle2 > 180):
            angle2 -= 360
        look_at_target2 = (angle2 * 20) / 180
        
        # direction 1
        if(angle < 90 and angle > -90):
            if direction == 0:
                left_speed = -10 - look_at_target
                right_speed = -10 + look_at_target
            else:
                left_speed = direction * 7
                right_speed = direction * -7
        # direction 2
        else:
            if direction2 == 0:
                left_speed = 10 - look_at_target2
                right_speed = 10 + look_at_target2
            else:
                left_speed = direction2 * 7
                right_speed = direction2 * -7

        if(left_speed > 10): left_speed = 10
        if(left_speed <-10): left_speed =-10
        if(right_speed > 10): right_speed = 10
        if(right_speed <-10): right_speed =-10
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    def motor(self, ML, MR):
        if(ML > 10): ML = 10
        if(ML <-10): ML =-10
        if(MR > 10): MR = 10
        if(MR <-10): MR =-10
        
        self.left_motor.setVelocity(ML)
        self.right_motor.setVelocity(MR)
    def getMinDist(self, arr):
        minDist = 999999
        index = -1
        for i in range(len(arr)):
            d = math.sqrt((self.x_ball - arr[i]['x'])**2 + (self.y_ball - arr[i]['y'])**2)
            if d < minDist:
                minDist = d
                index = i
        return index
    def guessNeutralPoint(self):
        NEUTRAL_SPOTS = [
            {'x': 0,'y': 0},
            {'x': -0.3, 'y': -0.34},
            {'x': -0.2, 'y': 0},
            {'x': -0.3, 'y': 0.34},
            {'x': 0.3, 'y':  0.34},
            {'x': 0.2, 'y':  0},
            {'x': 0.3, 'y':  -0.34}
        ]
        if(self.name[0] == 'Y'):
            NEUTRAL_SPOTS = [
                {'x': 0,'y': 0},
                {'x': -0.3, 'y': -0.34},
                {'x': -0.2, 'y': 0},
                {'x': -0.3, 'y': 0.34},
                {'x': 0.3, 'y':  0.38},
                {'x': 0.2, 'y':  0},
                {'x': 0.3, 'y':  -0.38}
            ]
        else:
            NEUTRAL_SPOTS = [
                {'x': 0,'y': 0},
                {'x': -0.3, 'y': -0.38},
                {'x': -0.2, 'y': 0},
                {'x': -0.3, 'y': 0.38},
                {'x': 0.3, 'y':  0.34},
                {'x': 0.2, 'y':  0},
                {'x': 0.3, 'y':  -0.34}
            ]
        nearest = self.getMinDist(NEUTRAL_SPOTS)
        return NEUTRAL_SPOTS[nearest]
    
    def run(self):
        ################################ Declaring variables
        self.blue_goal_pos = {'x': 0.7, 'y':0}
        self.yellow_goal_pos = {'x': -0.7, 'y':0}
        self.role = 'forward'
        self.cnt = 0
        self.kickoff = False
        self.corner = False
        self.passor = False
        self.extraPassor = False
        self.x2_ball = 0
        self.y2_ball = 0
        self.y_goal_keeper = 0
        self.data = []
        self.orientation = 0
        self.lookUp = 0
        self.goalKeeperCnt = 0
        self.turnTimeout = 0
        self.goalKeeperNum = 0
        self.y_future_ball = 0
        self.ball_speed = 0
        cnt2 = 0
        insideGoal = False
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.readSensor()
                neutralPos = self.guessNeutralPoint()
                
                # self.move(self.x_ball, self.y_ball)

                if(self.name[0] == 'Y'):
                    if(self.role == 'forward'):
                        ############################# Dangerus Position
                        if(self.x_ball < -0.55 and self.x_robot < -0.55):
                            if(self.x_robot > self.x_ball + 0.08):
                                if(self.y_ball > 0):
                                    self.move(self.x_ball - 0.12, self.y_ball - 0.2)    
                                else:
                                    self.move(self.x_ball - 0.12, self.y_ball + 0.2)
                            elif(self.y_ball > 0):
                                if(self.y_robot < self.y_ball):
                                    self.move(self.x_ball - 0.05, self.y_ball)
                                else:
                                    self.move(self.x_ball + 0.2, self.y_ball)
                            else: 
                                if(self.y_robot > self.y_ball):
                                    self.move(self.x_ball - 0.05, self.y_ball)
                                else:
                                    self.move(self.x_ball + 0.2, self.y_ball)
                        ############################# Lack off progress forward robot
                        elif(self.passor and neutralPos['x'] != 0 and neutralPos['y'] != 0 and self.kickoff):
                            if(abs(neutralPos['x'] - 0.12 - self.x_robot) < 0.05 and abs(neutralPos['y'] - self.y_robot) < 0.05):
                                self.motor(-self.look_at_blue * 10, self.look_at_blue * 10)
                            else:
                                self.move(neutralPos['x'] - 0.12, neutralPos['y'])
                        ############################# Go to Pass position
                        elif self.passor and self.corner and not self.extraPassor:
                            if(abs((self.x_ball - 0.1) - self.x_robot) < 0.05 and abs(self.blue_goal_pos['y'] - self.y_robot) < 0.05):
                                self.motor(-self.look_at_blue * 10, self.look_at_blue * 10)
                            else:
                                self.move(self.x_ball - 0.1, self.blue_goal_pos['y'])
                        ############################# kick the ball in Pass position
                        elif (not self.extraPassor) and self.x_robot > 0.2 and self.y_robot > -0.1 and self.y_robot < 0.1 and self.x_ball > self.x_robot - 0.1:
                            if(self.ball_distance < 0.1 and abs(self.y_robot - self.y_ball) > 0.05):
                                if(self.y_ball > 0):
                                    self.motor(10,-10)
                                else:
                                    self.motor(-10,10)
                            else:
                                self.move(self.x_ball, 0)
                        ############################# Run away from goal keeper lack off progress position
                        elif(self.x_robot > self.x_ball and self.x_robot > 0.2 and self.y_robot > -0.2 and self.y_robot < 0.2):
                            if(self.y_robot > self.y_ball):
                                self.move((self.x_robot + self.x_ball)/2, self.y_ball + 0.5)    
                            else:
                                self.move((self.x_robot + self.x_ball)/2, self.y_ball - 0.5)
                        ############################# go behind the ball
                        elif(self.x_robot > self.x_ball):
                            if(self.x_ball < -0.3):
                                if(self.y_ball > 0):
                                    self.move(self.x_ball - 0.12, self.y_ball - 0.2)    
                                else:
                                    self.move(self.x_ball - 0.12, self.y_ball + 0.2)
                            else:
                                if(self.y_robot > self.y_ball):
                                    self.move(self.x_ball - 0.12, self.y_ball + 0.1)    
                                else:
                                    self.move(self.x_ball - 0.12, self.y_ball - 0.1)
                        ############################# Move towards the ball
                        elif(abs(self.y_robot - self.y_ball) > 0.1 and abs(self.x_robot - self.x_ball) > 0.1 ):#or (self.ball_speed > 0.02 and self.ball_speed < 1.1)):
                            self.move(self.x_ball - 0.2, self.y_ball)
                        ############################# Push the ball
                        else:
                            self.move(self.x_ball, self.y_ball)
                    else:
                        ############################# Goal Keeper kick off position in lack off progress
                        if(self.kickoff):
                            if(self.x_ball > 0):
                                if(abs(-0.1 - self.x_robot) < 0.05 and abs(self.y_robot) < 0.05):
                                    self.motor(-self.look_at_blue * 10, self.look_at_blue * 10)
                                    self.turnTimeout = 0
                                else:
                                    self.move(-0.1, 0)
                            else:
                                if(abs(-0.4 - self.x_robot) < 0.05 and abs(self.y_robot) < 0.05):
                                    self.motor(-self.look_at_blue * 10, self.look_at_blue * 10)
                                    self.turnTimeout = 0
                                else:
                                    self.move(-0.4, 0)
                        ############################# Goal Keeper
                        else:
                            self.turnTimeout = 0
                            self.move(self.yellow_goal_pos['x'] , self.y_goal_keeper)
                else:
                    if(self.role == 'forward'):
                        ############################# Dangerus Position
                        if(self.x_ball > 0.55 and self.x_robot > 0.55):
                            if(self.x_robot < self.x_ball - 0.08):
                                if(self.y_ball > 0):
                                    self.move(self.x_ball + 0.12, self.y_ball - 0.2)    
                                else:
                                    self.move(self.x_ball + 0.12, self.y_ball + 0.2)
                            elif(self.y_ball > 0):
                                if(self.y_robot < self.y_ball):
                                    self.move(self.x_ball + 0.05, self.y_ball)
                                else:
                                    self.move(self.x_ball - 0.2, self.y_ball)
                            else: 
                                if(self.y_robot > self.y_ball):
                                    self.move(self.x_ball + 0.05, self.y_ball)
                                else:
                                    self.move(self.x_ball - 0.2, self.y_ball)
                        ############################# Lack off progress forward robot
                        elif(self.passor and neutralPos['x'] != 0 and neutralPos['y'] != 0 and self.kickoff):
                            if(abs(neutralPos['x'] + 0.12 - self.x_robot) < 0.05 and abs(neutralPos['y'] - self.y_robot) < 0.05):
                                self.motor(-self.look_at_yellow * 10, self.look_at_yellow * 10)
                            else:
                                self.move(neutralPos['x'] + 0.12, neutralPos['y'])
                        ############################# Go to Pass position
                        elif self.passor and self.corner and not self.extraPassor:
                            if(abs((self.x_ball + 0.1) - self.x_robot) < 0.05 and abs(self.yellow_goal_pos['y'] - self.y_robot) < 0.05):
                                self.motor(-self.look_at_yellow * 10, self.look_at_yellow * 10)
                            else:
                                self.move(self.x_ball + 0.1, self.yellow_goal_pos['y'])
                        ############################# kick the ball in Pass position
                        elif (not self.extraPassor) and self.x_robot < -0.2 and self.y_robot > -0.1 and self.y_robot < 0.1 and self.x_ball < self.x_robot + 0.1:
                            if(self.ball_distance < 0.1 and abs(self.y_robot - self.y_ball) > 0.05):
                                if(self.y_ball > 0):
                                    self.motor(10,-10)
                                else:
                                    self.motor(-10,10)
                            else:
                                self.move(self.x_ball, 0)
                        ############################# Run away from goal keeper lack off progress position
                        elif(self.x_robot < self.x_ball and self.x_robot < -0.2 and self.y_robot > -0.2 and self.y_robot < 0.2):
                            if(self.y_robot > self.y_ball):
                                self.move((self.x_robot + self.x_ball)/2, self.y_ball + 0.5)    
                            else:
                                self.move((self.x_robot + self.x_ball)/2, self.y_ball - 0.5)
                        ############################# go behind the ball
                        elif(self.x_robot < self.x_ball):
                            if(self.x_ball > 0.3):
                                if(self.y_ball > 0):
                                    self.move(self.x_ball + 0.12, self.y_ball - 0.2)    
                                else:
                                    self.move(self.x_ball + 0.12, self.y_ball + 0.2)
                            else:
                                if(self.y_robot > self.y_ball):
                                    self.move(self.x_ball + 0.12, self.y_ball + 0.1)    
                                else:
                                    self.move(self.x_ball + 0.12, self.y_ball - 0.1)
                        ############################# Move towards the ball
                        elif(abs(self.y_robot - self.y_ball) > 0.1 and abs(self.x_robot - self.x_ball) > 0.1 ):#or (self.ball_speed > 0.02 and self.ball_speed < 1.1)):
                            if(self.y_ball > self.y2_ball and self.y_robot > self.y_ball and (self.ball_speed > 0.02 and self.ball_speed < 1.1)):
                                self.move(self.x_ball, self.y_ball)
                            elif(self.y_ball < self.y2_ball and self.y_robot < self.y_ball and (self.ball_speed > 0.02 and self.ball_speed < 1.1)):
                                self.move(self.x_ball, self.y_ball)
                            else:
                                self.move(self.x_ball + 0.2, self.y_future_ball)
                        ############################# Push the ball
                        else:
                            self.move(self.x_ball, self.y_ball)
                    else:
                        ############################# Goal Keeper kick off position in lack off progress
                        if(self.kickoff):
                            if(self.x_ball < 0):
                                if(abs(0.1 - self.x_robot) < 0.05 and abs(self.y_robot) < 0.05):
                                    self.motor(-self.look_at_yellow * 10, self.look_at_yellow * 10)
                                    self.turnTimeout = 0
                                else:
                                    self.move(0.1, 0)
                            else:
                                if(abs(0.4 - self.x_robot) < 0.05 and abs(self.y_robot) < 0.05):
                                    self.motor(-self.look_at_yellow * 10, self.look_at_yellow * 10)
                                    self.turnTimeout = 0
                                else:
                                    self.move(0.4, 0)
                        ############################# Goal Keeper
                        else:
                            self.turnTimeout = 0
                            self.move(self.blue_goal_pos['x'] , self.y_goal_keeper)