import numpy as np
from scipy.interpolate import CubicSpline
DEFAULT_NEIGHBOURHOOD_VAL = 100 #neighbourhood radius
DEFAULT_SIZE = 1.5 #Radius of chassis
MAX_SPEED = 1.5
MAX_ANGULAR = 0.3
DEFAULT_STATE=0
SEARCH_STATE=2
class Bot:
	"""
	Robot class
	"""
	def __init__(self, x, y, theta=0,\
		state=DEFAULT_STATE,\
		size=DEFAULT_SIZE,\
		neighbourhood_radius=DEFAULT_NEIGHBOURHOOD_VAL,\
		speed=MAX_SPEED, \
		max_turn_speed=MAX_ANGULAR, \
		verbose=False):

		#TODO: Bot ID 

		#Basic attributes
		self.x = x
		self.y = y
		self.theta = theta
		self.state = state
		self.size = size
		self.neighbourhood_radius = neighbourhood_radius
		self.max_speed = speed
		self.max_turn_speed = max_turn_speed
		self.verbose=verbose
		if(verbose):
			print("verbose",verbose)
		#	print("New bot spawned at: ("+str(x)+','+str(y)+')')

		self.sim = None #Reference to Simulation object that spawns the bot

		#Goal (weight given to goal in potential field)
		self.goal_given = False
		self.goal = [0,0]

		#For exploration
		self.explore_dir = np.pi*(2*np.random.rand()-1)
		print("state",self.state)
		print("size",self.size)	
		
	def dist(self, x, y):
		"""
		Returns the distance of the centre of the robot
		from the point x,y
		"""
		return np.linalg.norm([x-self.x, y-self.y])

	def neighbours(self, radius=None, single_state=False, state=None):
		"""
		Returns: List of neighbours
		Args:
			radius: Neighbourhood radius to use (default: self.neighbourhood_radius)
			single_state: If true, then return only neighbours of the given state

		(TODO) Enhancement:
		(2) Include self
		"""
		if radius == None:
			radius = self.neighbourhood_radius

		neighbours = []
		if self.sim == None:
			print("WARNING: Robot is not linked to a simulation")
			return neighbours

		for bot in self.sim.swarm:
			#Skip if a single state is needed
			if single_state:
				if bot.state != state:
					continue
			
			#Get distance to bot
			d = bot.dist(self.x, self.y)
			
			#Check if neighbour
			if d>0 and d<radius:
				neighbours.append(bot)

		return neighbours


	def step(self, step_size=0.008):
		x_ = self.x + step_size*np.cos(self.theta)
		y_ = self.y + step_size*np.sin(self.theta)

		if self.sim.check_free(x_,y_,self.size+0.01, ignore=self) or\
		not self.sim.check_free(self.x, self.y, self.size-0.01, ignore=self):
			#First condition checks if new location is free
			#Second condition checks for unstuck (current location not free)
			self.x, self.y = x_,y_
		else:
			pass
			
			'''
			print("Collision! Adjusting position.")
			separation_distance = 0.1
		    
			for _ in range(10):
				# Calculate a random angle to adjust the position
				random_angle = np.random.uniform(0, 2 * np.pi)
				
				# Calculate the adjustment vector
				adjustment_x = separation_distance * np.cos(random_angle)
				adjustment_y = separation_distance * np.sin(random_angle)
				
				# Calculate the adjusted position
				adjusted_x = self.x + adjustment_x
				adjusted_y = self.y + adjustment_y
				
				# Check if the adjusted position is free and maintains the desired separation
				if self.sim.check_free(adjusted_x, adjusted_y, self.size, ignore=self):
				    self.x, self.y = adjusted_x, adjusted_y
				    break
			'''
	def turn(self, angle):
		self.theta +=angle
		'''
		while self.theta>2*np.pi:
			self.theta-=2*np.pi
		while self.theta<0:
			self.theta+=2*np.pi
		'''
	#def move(self, direction, speed, step_size=130):
	def move(self, direction, speed, step_size=0.8):
		"""
		The robot moves one step in the given direction
		The size of step os step_size*speed
		The speed used is the minimum of speed param and self.max_speed
		The bot does not start moving till the direction is within max_turn_speed
		"""
		turn_angle = direction-self.theta
		#print("turn_angle",turn_angle)
		bank_angle = 30 * np.pi / 180  # 30 degrees
		turn_angle = np.arctan2(np.sin(direction - self.theta), np.cos(direction - self.theta) * np.cos(bank_angle))

        # Turn the plane
		self.turn(turn_angle)

        # Move the plane forward
		self.step(min(speed, self.max_speed) * step_size)
		'''
		while turn_angle>=np.pi:
			turn_angle-=2*np.pi
			#print("turn_angle-----------",turn_angle)
		while turn_angle<=-np.pi:
			turn_angle+=2*np.pi
			#print("turn_angle!!+++++++++",turn_angle)
		
		if np.abs(turn_angle)>self.max_turn_speed:
			#If direction-theta is large, only turn without moving
			self.turn(np.sign(turn_angle)*self.max_turn_speed)
			#if (self.state!=SEARCH_STATE):				
			self.step(min(speed,self.max_speed)*step_size)#$$$$$$$$$$$$$$we need to check in exploration
			
		else:
			self.turn(turn_angle)
			self.step(min(speed,self.max_speed)*step_size)
		'''
	def unstuck(self, r):
		"""
		Use only if no other option is available
		"""
		x,y = np.random.randn(2)*r
		while not self.sim.check_free(x,y,r,ignore=self):
			x,y = np.random.randn(2)*r
		self.x, self.y = x,y

	def set_sim(self, sim):
		self.sim = sim

	def set_goal(self, x, y):
		self.goal = (x,y)
		self.goal_given = True
		#self.state=?

	def cancel_goal(self):
		self.goal_given = False

	def goal_exists(self):
		return self.goal_given

	def get_state(self):
		return self.state

	def set_state(self, state):
		self.state = state

	def get_pose(self):
		return self.x, self.y, self.theta

	def get_position(self):
		return self.x, self.y

	def get_dir(self):
		return self.theta




