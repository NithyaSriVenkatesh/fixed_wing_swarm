print("Running area coverage example 1\nUsing source files for package imports\nPARAMETERS:\
	Using default hardcoded weight_dicts (listed in swarm_tasks/logs)")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

#Set demo parameters directly
import numpy as np
import random
swarm_tasks.utils.robot.DEFAULT_NEIGHBOURHOOD_VAL = 6
swarm_tasks.utils.robot.DEFAULT_SIZE= 0.4
swarm_tasks.utils.robot.MAX_SPEED = 5
swarm_tasks.utils.robot.MAX_ANGULAR: 1.0
np.random.seed(42)
random.seed(42)
from swarm_tasks.modules.dispersion import disp_field
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl
import swarm_tasks.controllers.potential_field as potf
import swarm_tasks.controllers.base_control as base_control


from swarm_tasks.tasks import area_coverage as cvg

import numpy as np

NEIGHBOURHOOD_RADIUS = 4.5


s = sim.Simulation(num_bots=10, env_name='rectangles', contents_file='attractors')

gui = viz.Gui(s)
gui.show_env()
gui.show_bots()
#gui.show_grid()
goals=[(180.43188888272857,  332.28061553604584),( 227.08835934535978,  325.44862052621806),( 273.7240076527256,  296.17329693726674),(294.043613032667,  270.3682822428898),( 271.12685897931516,  203.80641438257226)]
j=0
while 1:
	if j==len(goals):
		break
	for i,b in enumerate(s.swarm):
		
		current_position=[b.x,b.y]
		cmd =cvg.goal_area_cvg(i,b,goal[j])
		
		#Execute
		cmd.exec(b)
		print("2222222",i,b,goal)
		#print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
		dx=abs(goal[0]-current_position[0])
		dy=abs(goal[1]-current_position[1])						
		print("dx,dy",dx,dy)
		if(dx<=10 and dy<=10):
			j+=1
			
		cmd.exec(bot)
	s.update_grid()
	#gui.show_grid()
	gui.update()
	s.time_elapsed+=1

gui.run()
