import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl
import swarm_tasks.controllers.potential_field as potf
import swarm_tasks.controllers.base_control as base_control

from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules.dispersion import disp_field
from swarm_tasks.modules.formations import circle
from swarm_tasks.modules.formations import line

from swarm_tasks.modules.surround import surround_attractor
from swarm_tasks.modules import exploration as exp
from swarm_tasks.modules import follow

from swarm_tasks.modules import decisions

import swarm_tasks.utils.logging as logging


import numpy as np

s = sim.Simulation(env_name='empty_world', contents_file='attractors')
#s = sim.Simulation(env_name='rectangles', contents_file='attractors')
#s = sim.Simulation(env_name='empty_world')
#s = sim.Simulation(num_bots=15,env_name='empty_world', contents_file='attractors')
#s = sim.Simulation(num_bots=20, env_name='rectangles', contents_file='attractors',\
	#num_initial_states =2)


gui = viz.Gui(s)

gui.show_env()


gui.show_bots()

#s.swarm[0].set_goal(6,1)
#gui.show_grid()

i=0


while 1:
	for b in s.swarm:
		#b.move(3.1, 1)
		#cmd = ctrl.command.Cmd(speed=1, dir_=-2*i*np.pi/50)
		#cmd = ctrl.command.Cmd([2*np.cos(2*i*np.pi/50),2*np.sin(2*i*np.pi/50)])
		#cmd = potf.get_field((b.get_position()),b.sim)
		x,y = b.get_position()


		#cmd = potf.get_field((b.get_position()),b.sim, goal_set=b.goal_exists(), goal=b.goal)
		

		"""
		-------------
		BASE
		
		"""
		cmd = base_control.base_control(b)
		cmd+= base_control.obstacle_avoidance(b)
		
		"""
		------------
		#FORMATIONS
		"""
		cmd+=circle(b,4)
		#cmd+=line(b)
		"""
		-------------
		AGGR/DISP
		"""

		#cmd+= disp_field(b)*0.05
		#cmd+=aggr_centroid(b)
		#cmd+=aggr_field(b)

		#cmd+=aggr_field(b)*0.1
		#cmd+=surround_attractor(b)
		
		"""
		------------
		EXPLORATION
		"""
		#cmd += exp.explore(b)

		"""
		FOLLOW
		"""
		#cmd+=follow.follow_leader(b,s.swarm[0])
		"""
		-------------
		EXECUTE
		"""
		cmd.exec(b)

		"""
		-------------
		DECISIONS
		"""
		#decisions.switch_stoch(b,2, 0.001)
		#decisions.consensus(b)

	#if not iter_%100:
		#gui.show_grid(grid)
	

	#s.swarm[0].step()
	#s.swarm[2].step()
	#s.swarm[0].turn(0.05)
	

	#s.update_grid()
	#gui.show_grid()
	
	gui.update()
	s.time_elapsed+=1
	i+=1

	#Get & save logs
	robot_params = logging.log_robot()
	sim_params = s.get_sim_param_log()

	weight_params = "# WEIGHT PARAMETERS\n\nweight_dicts: default_weight_dicts"
	#logging.save_log([robot_params,sim_params, weight_params], "swarm_tasks/logs/basic_tests", "TEST_PARAMS_CIRCLE-", ext=".yaml")



gui.run()