print("Running foraging example 1\nUsing source files for package imports\nPARAMETERS:")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

#Set demo parameters directly
import numpy as np
import random
import time
swarm_tasks.utils.robot.DEFAULT_NEIGHBOURHOOD_VAL = 7
swarm_tasks.utils.robot.DEFAULT_SIZE= 0.4
swarm_tasks.utils.robot.MAX_SPEED = 1.5
swarm_tasks.utils.robot.MAX_ANGULAR: 0.3
np.random.seed(42)
random.seed(42)
from scipy.spatial import distance
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
	
#Required modules
import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl
import swarm_tasks.controllers.potential_field as potf
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.formations import line,circle	
from swarm_tasks.modules.dispersion import disp_field

from swarm_tasks.utils import robot as bot

from swarm_tasks.tasks import area_coverage as cvg
from swarm_tasks.modules.formations import line
from dronekit import connect, VehicleMode, LocationGlobalRelative
import socket
import threading
import locatePosition
import csv

csv_file_path = "/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/uav_data_log.csv" 
csv_file_path1 = "/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/waypoints.csv" 
nextwaypoint=0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('192.168.29.226', 12005)  #receive from .....rx.py


sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address2 = ('', 12008)  #receive from .....rx.py
sock2.bind(server_address2)
sock2.setblocking(0)

sock3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address3 = ('', 12002)  #receive from .....rx.py
sock3.bind(server_address3)

graph_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
graph_server_address = ('192.168.29.226', 12009)  #receive from .....rx.py

remove_socket= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
remove_bot_server_address = ('192.168.29.226', 12001)  #receive from .....rx.py

sock4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address4 = ('', 12011)  #receive from .....rx.py
sock.bind(server_address4)

num_bots=0
vehicles=[]
pos_array=[]
try:	
	vehicle1= connect('udpin:192.168.29.125:14551',baud=115200, heartbeat_timeout=30)
	vehicle1.airspeed = 12
	print('Drone1')
	vehicles.append(vehicle1)
	pos_array.append(1)
	num_bots+=1
except:
	
	pass
	print(	"Vehicle 1 is lost")
try:		
	vehicle2= connect('udpin:192.168.29.125:14552',baud=115200,heartbeat_timeout=30)
	vehicle2.airspeed = 12
	print('Drone2')
	num_bots+=1	
	vehicles.append(vehicle2)
	pos_array.append(2)
except:
	
	pass	
	print(	"Vehicle 2 is lost")

try:
	vehicle3= connect('udpin:192.168.29.125:14553',baud=115200,heartbeat_timeout=30)
	vehicle3.airspeed = 12
	print('Drone3')
	num_bots+=1
	vehicles.append(vehicle3)
	pos_array.append(3)
except:
	
	pass
	print(	"Vehicle 3 is lost")
try:		
	vehicle4= connect('udpin:192.168.29.125:14554',baud=115200,heartbeat_timeout=30)
	vehicle4.airspeed = 12
	print('Drone4')
	num_bots+=1
	vehicles.append(vehicle4)
	pos_array.append(4)
except:
	
	pass
	print(	"Vehicle 4 is lost")
try:		
	vehicle5= connect('udpin:192.168.29.125:14555',baud=115200,heartbeat_timeout=30)
	vehicle5.airspeed = 12
	print('Drone5')
	num_bots+=1
	vehicles.append(vehicle5)
	pos_array.append(5)
except:
	
	pass
	print(	"Vehicle 5 is lost")
try:
	vehicle6= connect('udpin:192.168.29.125:14556',baud=115200,heartbeat_timeout=30)
	vehicle6.airspeed = 12
	print('Drone6')
	num_bots+=1
	vehicles.append(vehicle6)
	pos_array.append(6)
except:
	
	pass	
	print(	"Vehicle 6 is lost")
try:
	vehicle7= connect('udpin:192.168.29.125:14557',baud=115200,heartbeat_timeout=30)
	vehicle7.airspeed = 12
	print('Drone7')
	num_bots+=1
	vehicles.append(vehicle7)
	pos_array.append(7)
except:
	
	pass
	print(	"Vehicle 7 is lost")
try:
	vehicle8= connect('udpin:192.168.29.125:14558',baud=115200,heartbeat_timeout=30)
	vehicle8.airspeed = 12
	print('Drone8')
	num_bots+=1
	vehicles.append(vehicle8)
	pos_array.append(8)
except:	
	pass
	print(	"Vehicle 8 is lost")
try:	
	vehicle9= connect('udpin:192.168.29.125:14559',baud=115200,heartbeat_timeout=30)
	vehicle9.airspeed = 12
	print('Drone9')
	num_bots+=1
	vehicles.append(vehicle9)
	pos_array.append(9)
except:
	pass
	print(	"Vehicle 9 is lost")
try:	
	vehicle10= connect('udpin:192.168.29.125:14560',baud=115200,heartbeat_timeout=30)
	vehicle10.airspeed = 12
	print('Drone10')
	vehicles.append(vehicle10)
	pos_array.append(10)
	num_bots+=1
except:
	pass
	print(	"Vehicle 10 is lost")
print(len(vehicles))
#vehicles.pop(1)
print(len(vehicles))

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checcks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)

vehicle1_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle1, 30))
vehicle2_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle2, 31))
vehicle3_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle3, 32))
vehicle4_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle4, 33))
vehicle5_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle5, 34))
vehicle6_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle6, 35))
vehicle7_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle7, 36))
vehicle8_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle8, 37))
vehicle9_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle9, 38))
vehicle10_thread = threading.Thread(target=arm_and_takeoff, args=(vehicle10, 39))
#time.sleep(15)
# Start the threads

vehicle1_thread.start()
vehicle2_thread.start()
vehicle3_thread.start()
vehicle4_thread.start()
vehicle5_thread.start()
vehicle6_thread.start()
vehicle7_thread.start()
vehicle8_thread.start()
vehicle9_thread.start()
vehicle10_thread.start()

    
# Wait for all threads to finish
vehicle1_thread.join()
vehicle2_thread.join()
vehicle3_thread.join()
vehicle4_thread.join()
vehicle5_thread.join()
vehicle6_thread.join()
vehicle7_thread.join()
vehicle8_thread.join()
vehicle9_thread.join()
vehicle10_thread.join()

#num_bots=10
count=0
origin=(12.924801, 80.042719)
endDistance=2000
same_height=30
different_height=[30,31,32,33,34,35,36,37,38,39]
#search_height=[15,18,21,24,27]
home_pos=[]
home_pos_lat_lon=[]
#vehicles = [globals()[f'vehicle{i}'] for i in range(1, num_bots+1)]
print(vehicles)
uav_home_pos=[]
current_lat_lon=[]
home_flag=False
home_flag1=False
search_flag=False
search_height_flag=False
return_height_flag=False
lost_vehicle_num=0
vehicle_lost_flag=False
# Iterate over the list of vehicles
robot1_x,robot1_y = 0,0
robot2_x,robot2_y = 0,0
robot3_x,robot3_y = 0,0
robot4_x,robot4_y = 0,0
robot5_x,robot5_y = 0,0
robot6_x,robot6_y = 0,0
robot7_x,robot7_y = 0,0
robot8_x,robot8_y = 0,0
robot9_x,robot9_y = 0,0
robot10_x,robot10_y = 0,0
robot11_x,robot11_y = 0,0
robot12_x,robot12_y = 0,0
robot13_x,robot13_y = 0,0
robot14_x,robot14_y = 0,0
robot15_x,robot15_y = 0,0
robot16_x,robot16_y = 0,0

slave_heal_ip=["192.168.29.125"]*len(vehicles)
heartbeat=[0]*len(vehicles)

vehicle_uav_heartbeat_flag=False
disperse_flag=False

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address1 = ('192.168.29.226', 12010)


def vehicle_collision_moniter_receive():	
        global index
        while 1:
        	index, address = sock3.recvfrom(1024)
        	print ("msg!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", index)
        	#vehicle_uav_heartbeat_flag=True
			
 			
collision_thread = threading.Thread(target=vehicle_collision_moniter_receive)
collision_thread.daemon=True
collision_thread.start()

'''
def heartbeat_monitor():	
        global uav_heartbeat
        while 1:
        	uav_heartbeat, address = sock3.recvfrom(1024)
        	print ("msg!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", uav_heartbeat)
        	uav_heartbeat=True
		#vehicle_uav_heartbeat=int(str(uav_heartbeat))
			
'''
                
def CHECK_network_connection():
    global slave_heal_ip
    global vehicles
    global lost_vehicle_num
    global vehicle_lost_flag
    while 1:
	    for i,iter_follower in enumerate(vehicles):
	    	response = os.system('ping -c 1 ' + slave_heal_ip[i])
	    	if response==0: # Link is OK.
	    		#print ("link is ok", slave_heal_ip[i])
	    		#message= "vehicle "+str(i+1)+" link okay"                       
	    		#sock1.sendto(str(message).encode(), server_address1) 
	    		pass
	    	else: # Link is down.
	 	    #print ("link is down", slave_heal_ip[i])
	 	    slave_heal_ip[i] = 'nolink'    
	 	    lost_vehicle_num=i+1
	 	    print("lostvehicle",lost_vehicle_num)
	 	    vehicle_lost_flag=True
	 	    remove_flag=True
	 	    message= "vehicle "+str(i+1)+" linkdown"                       
	 	    sock1.sendto(str(message).encode(), server_address1) 
		

def vehicle_heartbeat():
        global vehicles
        global slave_heal_ip
        global num_bots
        global lost_vehicle_num
        global vehicle_lost_flag
#	count=0
        #a=CHECK_network_connection()
        while 1:
        	
        	#time.sleep(0.2)
        	loc_1 = []
        	#for val_1 in range(1,numDrones+1):
        	for i,iter_follower in enumerate(vehicles):        	        
                	heartbeat[i]=vehicles[i].last_heartbeat
                					
                	#print( "heartbeat",heartbeat)
			#print(" Last Heartbeat: %s" , vehicles[i].last_heartbeat)
			#print("slave_heal_ip[i]",slave_heal_ip[i])
                	if(heartbeat[i]>10):
                		#print("Vehicle ", i+1 ,"No heartbeat")   	
                		lost_vehicle_num=i+1
                		print("lostvehicle",lost_vehicle_num)   
                		vehicle_lost_flag=True  
                		remove_flag=True           		
                		message= "vehicle "+str(i+1)+" has no heartbeat"                   		
                		sock1.sendto(str(message).encode(), server_address1) 
                		#print("Exception in network monitor")	
                		
       	#sock.sendto(str(heartbeat), server_address)   			

heartbeat_thread = threading.Thread(target=vehicle_heartbeat)
heartbeat_thread.daemon=True
heartbeat_thread.start()

network_thread = threading.Thread(target=CHECK_network_connection)
network_thread.daemon=True
network_thread.start()

robot_positions = [([0, 0]) for _ in range(20)]
print("##########")
print(vehicles)
for i,vehicle in enumerate(vehicles):
    print("@@@@@@@@@@@")
    '''
    if(pop_flag):
    	if(i>=pop_bot_index):
    		i+=1
    '''
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    home = vehicle.home_location
    
    print(home.lat,home.lon)
    while home.lat is None:
        print(" Waiting for home position...")
        time.sleep(1)
    # Process the lat and lon as needed
    print(f"Vehicle - Latitude: {home.lat}, Longitude: {home.lon}")
    x,y = locatePosition.geoToCart (origin, endDistance, [home.lat,home.lon])
    home_pos_lat_lon.append((home.lat,home.lon))
    print("x,y",x/2,y/2)
    home_pos.append((x / 2, y / 2))

    if i==0:
    	robot1_x,robot1_y={x/2},{y/2}
    elif i==1:
    	robot2_x,robot2_y={x/2},{y/2}
    elif i==2:
    	robot3_x,robot3_y={x/2},{y/2}
    elif i==3:
    	robot4_x,robot4_y={x/2},{y/2}
    elif i==4:
    	robot5_x,robot5_y={x/2},{y/2}
    elif i==5:
    	robot6_x,robot6_y={x/2},{y/2}
    elif i==6:
    	robot7_x,robot7_y={x/2},{y/2}
    elif i==7:
    	robot8_x,robot8_y={x/2},{y/2}	
    elif i==8:
    	robot9_x,robot9_y={x/2},{y/2}
    elif i==9:
    	robot10_x,robot10_y={x/2},{y/2}
    elif i==10:
    	robot11_x,robot11_y={x/2},{y/2}
    elif i==11:
    	robot12_x,robot12_y={x/2},{y/2}
    elif i==12:
    	robot13_x,robot13_y={x/2},{y/2}
    elif i==13:
    	robot14_x,robot14_y={x/2},{y/2}
    elif i==14:
    	robot15_x,robot15_y={x/2},{y/2}
    elif i==15:
    	robot16_x,robot16_y={x/2},{y/2}
    else:
	    print('invalid')
    msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
    	str(robot2_x) + ',' + str(robot2_y) + ',' +
	str(robot3_x) + ',' + str(robot3_y) + ',' +
	str(robot4_x) + ',' + str(robot4_y) + ',' +
	str(robot5_x) + ',' + str(robot5_y) + ',' +
	str(robot6_x) + ',' + str(robot6_y) + ',' +
	str(robot7_x) + ',' + str(robot7_y) + ',' +
	str(robot8_x) + ',' + str(robot8_y) + ',' +
	str(robot9_x) + ',' + str(robot9_y) + ',' +
	str(robot10_x) + ',' + str(robot10_y))
			       
    print("msg",msg)  	
#sent = graph_socket.sendto(str(msg).encode(), graph_server_address)


for i,vehicle in enumerate(vehicles):
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon

       # Process the lat and lon as needed
    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
    current_lat_lon.append((lat,lon))			    
    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
    print("x,y",x/2,y/2)
    uav_home_pos.append((x / 2, y / 2))  
    
    if i==0:
    	robot1_x,robot1_y={x/2},{y/2}
    elif i==1:
    	robot2_x,robot2_y={x/2},{y/2}
    elif i==2:
    	robot3_x,robot3_y={x/2},{y/2}
    elif i==3:
    	robot4_x,robot4_y={x/2},{y/2}
    elif i==4:
    	robot5_x,robot5_y={x/2},{y/2}
    elif i==5:
    	robot6_x,robot6_y={x/2},{y/2}
    elif i==6:
    	robot7_x,robot7_y={x/2},{y/2}
    elif i==7:
    	robot8_x,robot8_y={x/2},{y/2}	
    elif i==8:
    	robot9_x,robot9_y={x/2},{y/2}
    elif i==9:
    	robot10_x,robot10_y={x/2},{y/2}
    elif i==10:
    	robot11_x,robot11_y={x/2},{y/2}
    elif i==11:
    	robot12_x,robot12_y={x/2},{y/2}
    elif i==12:
    	robot13_x,robot13_y={x/2},{y/2}
    elif i==13:
    	robot14_x,robot14_y={x/2},{y/2}
    elif i==14:
    	robot15_x,robot15_y={x/2},{y/2}
    elif i==15:
    	robot16_x,robot16_y={x/2},{y/2}
    else:
	    print('invalid')
    msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
    	str(robot2_x) + ',' + str(robot2_y) + ',' +
	str(robot3_x) + ',' + str(robot3_y) + ',' +
	str(robot4_x) + ',' + str(robot4_y) + ',' +
	str(robot5_x) + ',' + str(robot5_y) + ',' +
	str(robot6_x) + ',' + str(robot6_y) + ',' +
	str(robot7_x) + ',' + str(robot7_y) + ',' +
	str(robot8_x) + ',' + str(robot8_y) + ',' +
	str(robot9_x) + ',' + str(robot9_y) + ',' +
	str(robot10_x) + ',' + str(robot10_y))
    print("msg..",msg)
sent = graph_socket.sendto(str(msg).encode(), graph_server_address)

area_covered = 0
start_time = time.time()
visited_positions = set()
data=""
same_alt_flag=False
elapsed_time= time.time() - start_time


index=0
flag_stop=False
return_flag=False
aggregate_flag=False
pop_flag_arr=[1]*num_bots
pop_flag=False
specific_bot_goal_flag=False
pop_bot_index=None
goal_bot_num=None
start_flag=False
circle_formation_flag=False
radius_of_earth = 6378100.0 # in meters
uav_home_flag=False
remove_flag=False
home_counter=0
circle_formation_count=0
#Initialize Simulation and GUI 
s = sim.Simulation(uav_home_pos,num_bots=len(vehicles), env_name='rectangles')
#gui = viz.Gui(s)
#gui.show_env()


while(1):
	try:            
		data, address = sock2.recvfrom(1050)
		print ("msg", data)
		if(data==b"store_uav_pos"):
			if os.path.exists(csv_file_path):
    				os.remove(csv_file_path)
			
			with open(csv_file_path, mode='w', newline='') as csv_file:
				csv_writer = csv.writer(csv_file)
				csv_writer.writerow(['X', 'Y'])  # Write header
				csv_writer.writerows(home_pos)
				csv_file.close()

		if(data==b"rtl"):			
			print("rtl!!!!!!!!!!!!!!!!")
			for i,b in enumerate(s.swarm):
				if(pop_flag_arr[i]==0):
					i+=1	
				vehicles[i].mode = VehicleMode("RTL")
				vehicles[i].close()
			'''
			print("Returning to Launch vehicle 1")
			vehicle1.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 2")
			vehicle2.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 3")
			vehicle3.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 4")
			vehicle4.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 5")
			vehicle5.mode = VehicleMode("RTL")			
			print("Returning to Launch vehicle 6")
			vehicle6.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 7")
			vehicle7.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 8")
			vehicle8.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 9")
			vehicle9.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 10")
			vehicle10.mode = VehicleMode("RTL")
			
			print("Returning to Launch vehicle 11")			
			vehicle11.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 12")
			vehicle12.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 13")
			vehicle13.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 14")
			vehicle14.mode = VehicleMode("RTL")
			print("Returning to Launch vehicle 15")
			vehicle15.mode = VehicleMode("RTL")		
						

			print("Close vehicle 1 object")
			vehicle1.close()
			print("Close vehicle 2 object")
			vehicle2.close()
			print("Close vehicle 3 object")
			vehicle3.close()
			print("Close vehicle 4 object")
			vehicle4.close()
			print("Close vehicle 5 object")
			vehicle5.close()			
			print("Close vehicle 6 object")
			vehicle6.close()
			print("Close vehicle 7 object")
			vehicle7.close()
			print("Close vehicle 8 object")
			vehicle8.close()			
			print("Close vehicle 9 object")
			vehicle9.close()
			print("Close vehicle 10 object")
			vehicle10.close()
					
			print("Close vehicle 11 object")
			vehicle11.close()
			print("Close vehicle 12 object")
			vehicle12.close()
			print("Close vehicle 13 object")
			vehicle13.close()
			print("Close vehicle 14 object")
			vehicle14.close()
			print("Close vehicle 15 object")
			vehicle15.close()						
			'''
			break
				
		if(data==b"remove") or (remove_flag):	
				'''
				for l in range(0,num_bots):
					pos_array.append(l+1)
				'''
				print("DDDDDDDDDDDDDDDDDDDDDDDDDDDD")
				if(vehicle_lost_flag):
					index=lost_vehicle_num
				print(pos_array)
				#index=9
				#pos_array.pop(4)
				#num_bots-=1		
				pop_flag=True
				print ("msg", index)
				for l in range(0,len(vehicles)):	
					if(int(index)==pos_array[l]):
						pop_bot_index=l
						print(l)
						break
				#pop_bot_index=int(index)
				print(pop_bot_index)
				pos_array.pop(pop_bot_index)
				print(len(pos_array))
				'''
				for l,key in enumerate(test_dict.keys()):
					print("Key:", key, "Value:", test_dict[key])
					if(key==pop_bot_index):
						print(l)
						pop_bot_index=l
				'''
				print("11111111111111111111",pop_bot_index)
				#pop_bot_index=	pop_bot_index
				#print(pop_bot_index)
				print(vehicles)
				#vehicles.pop(f'vehicle{pop_bot_index}')
				vehicles.pop(pop_bot_index)
				#vehicles.pop(pop_bot_index)
				print("555555555555",vehicles)
				#num_bots-=1
				#pop_bot_index=	pop_bot_index-1		
				s.remove_bot(pop_bot_index)
				print(len(vehicles))
				print('2222222222222')
				#pop_flag_arr[pop_bot_index]=0
				#remove_count+=1
				
				print("!!!!!!!!!!!!pop_flag_arr!!!!!!!!!!!",pop_flag_arr)
				#for i in range(3):
				print("*************")
				sent = remove_socket.sendto(str(pop_bot_index).encode(), remove_bot_server_address)
				print("pop index",pop_bot_index)		
		if(data==b"add"):				
				bot_pos=[154.37296943770545 ,259.40487368680743]
				#print ("msg", index)
				#decoded_index = index.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
				#bot_pos_x,bot_pos_y = decoded_index.split(",")
				#bot_pos=[float(str(bot_pos_x)),float(str(bot_pos_x))]
				if(pop_flag):
					bot_ind=pop_bot_index
				else:
					bot_ind=num_bots
				s.add_bot(bot_ind,bot_pos)
				#uav_home_pos.append((x / 2, y / 2))
				#uav_home_pos.append((bot_pos[0],bot_pos[1]))				
				uav_home_pos.insert(bot_ind,(bot_pos[0],bot_pos[1]))
				pop_flag=False
				pop_flag_arr[pop_bot_index]=1

		if(data==b"specific_bot_goal"):	
				try:
					print("bot_goal!!!!!!!!!!!!")
					index, address = sock3.recvfrom(1024)
					print("index",index)
					decoded_index = index.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
					uav, goal_lat, goal_lon = decoded_index.split(",")
					#uav,goalx,goaly=str(index).split(",")
					print(uav, goal_lat, goal_lon)
					goalx,goaly = locatePosition.geoToCart (origin, endDistance, [goal_lat, goal_lon])
					print("x,y",goalx,goaly)
					goal_bot_num=int(str(uav))
					print(goal_bot_num)
					goal_position=[float(goalx/2),float(goaly/2)]
					print(goal_position)
					goal_bot_num=goal_bot_num-1
					#specific_bot_goal_flag=True
					while 1:
						if(specific_bot_goal_flag):
							specific_bot_goal_flag=False
							break			
						for i,b in enumerate(s.swarm):
							'''
							if(pop_flag_arr[i]==0):
								i+=1						
							
							if(pop_flag):	
								if(i>=pop_bot_index):
									i+=1
							'''
							current_position=[b.x,b.y]	
							if(i==goal_bot_num):								
								dx=abs(goal_position[0]-current_position[0])
								dy=abs(goal_position[1]-current_position[1])						
								print("dx,dy",dx,dy)
								if(dx<=1 and dy<=1):
									specific_bot_goal_flag=True
									break					
								else:
									current_position = (b.x, b.y)						
									if(i==goal_bot_num):								
										print("i",i)
										#print("final_goals[goal_bot_num-1]",final_goals)
										#goal_position = final_goals[goal_bot_num-1]
										#goal_position = [23,40]
										print("goal_position",goal_position)
										b.set_goal(goal_position[0], goal_position[1])
										cmd = cvg.goal_area_cvg(i, b, goal_position)								
										print("cmd11111 exceuted")
								
										print("@@@@@@@@@@@@@@@@@")			
										#print("final_goals[goal_bot_num-1]",final_goals[goal_bot_num-1])
										print("!!!!!!!!!!!!!!!!")
										cmd.exec(b) 	
								
							#	print("bot_array",bot_array)
							#	print("current_position",current_position)	
							#	print("Drone 1 position",b.x,b.y)
								current_position = (b.x*2, b.y*2)						
								lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
								print ("Drone 1 lat,lon", lat,lon)
								if same_alt_flag:
									point1 = LocationGlobalRelative(lat,lon,same_height)
								else:
									point1 = LocationGlobalRelative(lat,lon,different_height[i])
								#print("point1",point1)
								vehicles[i].simple_goto(point1)
								if i==0:
								    robot1_x,robot1_y={b.x},{b.y}
								elif i==1:
								    robot2_x,robot2_y={b.x},{b.y}
								elif i==2:
								    robot3_x,robot3_y={b.x},{b.y}
								elif i==3:
								    robot4_x,robot4_y={b.x},{b.y}
								elif i==4:
								    robot5_x,robot5_y={b.x},{b.y}
								elif i==5:
								    robot6_x,robot6_y={b.x},{b.y}
								elif i==6:
								    robot7_x,robot7_y={b.x},{b.y}
								elif i==7:
								    robot8_x,robot8_y={b.x},{b.y}
								elif i==8:
								    robot9_x,robot9_y={b.x},{b.y}
								elif i==9:
								    robot10_x,robot10_y={b.x},{b.y}
								elif i==10:
								    robot11_x,robot11_y={b.x},{b.y}
								elif i==11:
								    robot12_x,robot12_y={b.x},{b.y}
								elif i==12:
								    robot13_x,robot13_y={b.x},{b.y}
								elif i==13:
								    robot14_x,robot14_y={b.x},{b.y}
								elif i==14:
								    robot15_x,robot15_y={b.x},{b.y}
								elif i==15:
								    robot16_x,robot16_y={b.x},{b.y}
								else:
								    print('invalid')
								msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
							       str(robot2_x) + ',' + str(robot2_y) + ',' +
							       str(robot3_x) + ',' + str(robot3_y) + ',' +
							       str(robot4_x) + ',' + str(robot4_y) + ',' +
							       str(robot5_x) + ',' + str(robot5_y) + ',' +
							       str(robot6_x) + ',' + str(robot6_y) + ',' +
							       str(robot7_x) + ',' + str(robot7_y) + ',' +
							       str(robot8_x) + ',' + str(robot8_y)+ ',' +
							       str(robot9_x) + ',' + str(robot9_y)+ ',' +
							       str(robot10_x) + ',' + str(robot10_y))
											       
										
							sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
						if(index==b"stop"):
							specific_bot_goal_flag=False
							break	
				except:
					print("exception")	
					break			
		if(data==b"same"):
				sent = graph_socket.sendto(str("same altitude").encode(), graph_server_address)				
				velocity_flag=True	
				print ("msg", index)
				decoded_index = index.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
				data1, height = decoded_index.split(",")
				#uav,goalx,goaly=str(index).split(",")
				print(data1, height,step)
				#print(type(data1),type(height),type(step))	
				same_alt_flag=True
				same_height=int(height)

				#print("Velocity zero called##############!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")	
				while True:					
					#print("velocity_flag",velocity_flag)						
					for i, b in enumerate(s.swarm):
						'''
						if(pop_flag):	
							if(i>=pop_bot_index):
								i+=1						
						
						if(pop_flag_arr[i]==0):
							i+=1
						'''
						#print("Velocity zero called##############!!!!!!!!!!!")
						cmd = potf.velocity(b.get_position(), b.sim, weights=potf.field_weights, order = 2, max_dist=5)
						cmd.exec(b)
						# Break out of the loop
						#print("Velocity zero!!!!!!!!!!!")			
						value=[b.x*2,b.y*2]						
						#print("Drone 1 position",b.x,b.y)
						lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
						#print ("Drone 1 lat,lon", lat,lon)
						if same_alt_flag:
							point1 = LocationGlobalRelative(lat,lon,same_height)
						else:
							point1 = LocationGlobalRelative(lat,lon,different_height[i])
						#print("point1",point1)
						vehicles[i].simple_goto(point1)
					
					if(index==b"resume"):
						sent = graph_socket.sendto(str("resume").encode(), graph_server_address)				
						index="data"
						velocity_flag=False
						print("1st")
						data="search"
						break
												
		if(data==b"different"):
				sent = graph_socket.sendto(str("different altitude").encode(), graph_server_address)				
				velocity_flag=True	
				print ("msg", index)
				decoded_index = index.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
				data1, height,step = decoded_index.split(",")
				#uav,goalx,goaly=str(index).split(",")
				print(data1, height,step)
				#print(type(data1),type(height),type(step))	
				same_alt_flag=False
				#height = int(height)
				for h in range(len(vehicles)):
					different_height[h] = int(height) + int(step) * h
				print("different_height",different_height)
				#print("Velocity zero called##############!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")	
				while True:					
					#print("velocity_flag",velocity_flag)						
					for i, b in enumerate(s.swarm):
						'''
						if(pop_flag):	
							if(i>=pop_bot_index):
								i+=1
						
						if(pop_flag_arr[i]==0):
							i+=1
						'''
						#print("Velocity zero called##############!!!!!!!!!!!")
						cmd = potf.velocity(b.get_position(), b.sim, weights=potf.field_weights, order = 2, max_dist=5)
						cmd.exec(b)
						# Break out of the loop
						#print("Velocity zero!!!!!!!!!!!")			
						value=[b.x*2,b.y*2]						
						#print("Drone 1 position",b.x,b.y)
						lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
						#print ("Drone 1 lat,lon", lat,lon)
						if same_alt_flag:
							point1 = LocationGlobalRelative(lat,lon,same_height)
						else:
							point1 = LocationGlobalRelative(lat,lon,different_height[i])
						#print("point1",point1)
						vehicles[i].simple_goto(point1)
					
					if(index==b"resume"):
						index="data"
						velocity_flag=False
						print("1st")
						data="search"
						break
				

		if(data==b"start") or (circle_formation_flag):			
			sent = graph_socket.sendto(str("circle formation").encode(), graph_server_address)	
			uav_home_pos=[]
			for vehicle in vehicles:
			    lat = vehicle.location.global_relative_frame.lat
			    lon = vehicle.location.global_relative_frame.lon

			    # Process the lat and lon as needed
			    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
			    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
			    print("x,y",x/2,y/2)
			    
			    uav_home_pos.append((x / 2, y / 2))						
			print("uav_home_pos",uav_home_pos)    
			#gui.close()
			s = sim.Simulation(uav_home_pos,num_bots=len(vehicles), env_name='rectangles2' )
			#s = sim.Simulation(num_bots, env_name='rectangles2' )
			#gui = viz.Gui(s)			
			print("circle_formation_count",circle_formation_count)
			
			while circle_formation_count<5000:
				circle_formation_flag=True
				start_flag=True
				for i,b in enumerate(s.swarm):
					circle_formation_count+=1
					cmd = base_control.base_control(i,b,home_pos[0])
					cmd+= base_control.obstacle_avoidance(i,b,home_pos[0])					
					#Behaviour
					cmd+=circle(b,100)
					cmd += disp_field(b)
					 
					print("###########")
					#Execute
					cmd.exec(b)
					print("#####")
					current_position = [b.x*2,b.y*2]
					print(current_position)
					print("Drone 1 position",b.x,b.y)
					lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
					
					#print ("Drone 1 lat,lon", lat,lon)
					if same_alt_flag:
						point1 = LocationGlobalRelative(lat,lon,same_height)
					else:
						point1 = LocationGlobalRelative(lat,lon,different_height[i])
					vehicles[i].simple_goto(point1)
					if i==0:
					    robot1_x,robot1_y={b.x},{b.y}
					elif i==1:
					    robot2_x,robot2_y={b.x},{b.y}
					elif i==2:
					    robot3_x,robot3_y={b.x},{b.y}
					elif i==3:
					    robot4_x,robot4_y={b.x},{b.y}
					elif i==4:
					    robot5_x,robot5_y={b.x},{b.y}
					elif i==5:
					    robot6_x,robot6_y={b.x},{b.y}
					elif i==6:
					    robot7_x,robot7_y={b.x},{b.y}
					elif i==7:
					    robot8_x,robot8_y={b.x},{b.y}
					elif i==8:
					    robot9_x,robot9_y={b.x},{b.y}
					elif i==9:
					    robot10_x,robot10_y={b.x},{b.y}
					elif i==10:
					    robot11_x,robot11_y={b.x},{b.y}
					elif i==11:
					    robot12_x,robot12_y={b.x},{b.y}
					elif i==12:
					    robot13_x,robot13_y={b.x},{b.y}
					elif i==13:
					    robot14_x,robot14_y={b.x},{b.y}
					elif i==14:
					    robot15_x,robot15_y={b.x},{b.y}
					elif i==15:
					    robot16_x,robot16_y={b.x},{b.y}
					else:
					    print('invalid')
					msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
				       str(robot2_x) + ',' + str(robot2_y) + ',' +
				       str(robot3_x) + ',' + str(robot3_y) + ',' +
				       str(robot4_x) + ',' + str(robot4_y) + ',' +
				       str(robot5_x) + ',' + str(robot5_y) + ',' +
				       str(robot6_x) + ',' + str(robot6_y) + ',' +
				       str(robot7_x) + ',' + str(robot7_y) + ',' +
				       str(robot8_x) + ',' + str(robot8_y) + ',' +
				       str(robot9_x) + ',' + str(robot9_y) + ',' +
				       str(robot10_x) + ',' + str(robot10_y))
								       
							
				sent = graph_socket.sendto(str(msg).encode(), graph_server_address)

				#gui.update()
					
			
		if(data==b"start1") or (start_flag):
			if(circle_formation_count>=5000):
				circle_formation_flag=False
				start_flag=True
			#time.sleep(0.1)
			uav_home_pos=[]
			for vehicle in vehicles:
			    lat = vehicle.location.global_relative_frame.lat
			    lon = vehicle.location.global_relative_frame.lon

			    # Process the lat and lon as needed
			    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
			    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
			    print("x,y",x/2,y/2)
			    
			    uav_home_pos.append((x / 2, y / 2))						
			print("uav_home_pos",uav_home_pos)    
			#gui.close()
			s = sim.Simulation(uav_home_pos,num_bots=len(vehicles), env_name='rectangles2' )
			#s = sim.Simulation(num_bots, env_name='rectangles2' )
			#gui = viz.Gui(s)
			'''
			home_counter=0
			
			for i,vehicle in enumerate(vehicles):
			    lat = vehicle.location.global_relative_frame.lat
			    lon = vehicle.location.global_relative_frame.lon

			    dist=gps_distance(lat,lon,home_pos_lat_lon[i][0],home_pos_lat_lon[i][1])
			    print("uav_home_flag",uav_home_flag)
			    print("home_counter",home_counter)
			    print("dist",dist)
			    if(dist<5):
			    	home_counter+=1
			    if(home_counter==num_bots):
			    	uav_home_flag=True
			    
			    # Process the lat and lon as needed
			    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
			    current_lat_lon.append((lat,lon))			    
			    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
			    print("x,y",x/2,y/2)
			    uav_home_pos.append((x / 2, y / 2))
			    
			'''
			sent = graph_socket.sendto(str("start").encode(), graph_server_address)				
			for b in s.swarm:
				multiple_goals=[(157.64131776911836,209.16847386901821), (360.12460897774656,143.7605934973306), (581.468738086984,77.37929794272104),  (599.2505498180748,112.92467247032282),  (602.6444203564429,153.256210887913), (508.0359474466252,199.22290470278386),  (356.44627567858686,213.57908756056852), (290.83411085872586,223.81593558812182)]
				multiple_goals1=[(157.64131776911836,209.16847386901821), (360.12460897774656,143.7605934973306), (581.468738086984,77.37929794272104)]
				#multiple_goals1=[(157.64131776911836,209.16847386901821), (360.12460897774656,143.7605934973306), (581.468738086984,77.37929794272104),(599.2505498180748,112.92467247032282),  (602.6444203564429,153.256210887913), (508.0359474466252,199.22290470278386),  (356.44627567858686,213.57908756056852), (302.83411085872586,223.81593558812182)]
				bot_paths={
					   0:[multiple_goals[0],multiple_goals[1],multiple_goals[2],multiple_goals[3],multiple_goals[4],multiple_goals[5],multiple_goals[6],multiple_goals[7]],				   
					   1:[multiple_goals[1],multiple_goals[2],multiple_goals[3],multiple_goals[4],multiple_goals[5],multiple_goals[6],multiple_goals[7]],
					   2:[multiple_goals[2],multiple_goals[3],multiple_goals[4],multiple_goals[5],multiple_goals[6],multiple_goals[7]],
					   3:[multiple_goals[3],multiple_goals[4],multiple_goals[5],multiple_goals[6],multiple_goals[7]],
					   4:[multiple_goals[4],multiple_goals[5],multiple_goals[6],multiple_goals[7]],
					   5:[multiple_goals[5],multiple_goals[6],multiple_goals[7]],
					   6:[multiple_goals[6],multiple_goals[7]],
					   7:[multiple_goals[7]],

				}
				
				YOUR_GOAL_THRESHOLD_DISTANCE=15
				YOUR_THRESHOLD_DISTANCE=15
				current_goal_index = [0] * len(vehicles)			
				goal_table=[0]*len(vehicles)
				print("goal_table",goal_table)
				current_table=[0]*len(vehicles)
				length_arr=[0]*len(vehicles)
				new_length_arr=[0]*len(vehicles)
				count=[0]*len(vehicles)
				step=0
				search_flag=False
				all_bot_reach_flag=False
				bot_array=[0]*len(vehicles)
				
				while 1:
					print("############")
					print("pop_flag_arr",pop_flag_arr)
					'''
					if(vehicle_uav_heartbeat_flag):
						search_height_flag=True
						pop_flag=True
						pop_bot_index=int(str(uav_heartbeat))
						#same_height+=3
					'''
					if(search_height_flag):
						break
					step+=1
					print("step",step)
					if(step==1):											
						for i,b in enumerate(s.swarm):
							print("%%%%%%%%%%")
							'''
							if(pop_flag_arr[i]==0):
								i+=1
							#print("step",step,i)
							'''
												#print("potf.reached_goals",potf.reached_goals)							
							#if(potf.reached_goals[i]==False):
							current_position = [b.x,b.y]
							#nearest_goal = min(multiple_goals1, key=lambda goal: distance.euclidean(current_position, goal))
							#goal_ind=multiple_goals.index(nearest_goal)							
							#goal_table[i]=multiple_goals.index(nearest_goal)
							
							#gui.update()
							print("!!!!!!!!!!!current_table!!!!!!!",current_table)
							with open('/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv','rt')as text:
								data = csv.reader(text)
								for row in data: 
									data = (row)
									#print ("loc",data[-1])
									nextwaypoint=int(data[-1])							
							print("nextwaypoint!!!!!!!!!!!!!",nextwaypoint)
							if (nextwaypoint+1)==len(multiple_goals):
								goal_table[i]=nextwaypoint
							else:
								goal_table[i]=nextwaypoint+1
							print("!!!!!!!!!!!goalTable!!!!!!!",goal_table)
							if i==0:
							    robot1_x,robot1_y={b.x},{b.y}
							elif i==1:
							    robot2_x,robot2_y={b.x},{b.y}
							elif i==2:
							    robot3_x,robot3_y={b.x},{b.y}
							elif i==3:
							    robot4_x,robot4_y={b.x},{b.y}
							elif i==4:
							    robot5_x,robot5_y={b.x},{b.y}
							elif i==5:
							    robot6_x,robot6_y={b.x},{b.y}
							elif i==6:
							    robot7_x,robot7_y={b.x},{b.y}
							elif i==7:
							    robot8_x,robot8_y={b.x},{b.y}
							elif i==8:
							    robot9_x,robot9_y={b.x},{b.y}
							elif i==9:
							    robot10_x,robot10_y={b.x},{b.y}
							elif i==10:
							    robot11_x,robot11_y={b.x},{b.y}
							elif i==11:
							    robot12_x,robot12_y={b.x},{b.y}
							elif i==12:
							    robot13_x,robot13_y={b.x},{b.y}
							elif i==13:
							    robot14_x,robot14_y={b.x},{b.y}
							elif i==14:
							    robot15_x,robot15_y={b.x},{b.y}
							elif i==15:
							    robot16_x,robot16_y={b.x},{b.y}
							else:
							    print('invalid')
							msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
						       str(robot2_x) + ',' + str(robot2_y) + ',' +
						       str(robot3_x) + ',' + str(robot3_y) + ',' +
						       str(robot4_x) + ',' + str(robot4_y) + ',' +
						       str(robot5_x) + ',' + str(robot5_y) + ',' +
						       str(robot6_x) + ',' + str(robot6_y) + ',' +
						       str(robot7_x) + ',' + str(robot7_y) + ',' +
						       str(robot8_x) + ',' + str(robot8_y) + ',' +
						       str(robot9_x) + ',' + str(robot9_y) + ',' +
						       str(robot10_x) + ',' + str(robot10_y))
						sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
						print("@@@@@@@@@@@@@@@@@")
					else:
						print("###################")
						'''
						if(num_bots==10):
							print("**********")
							time.sleep(0.003)
						elif(len(vehicles)==9):
							time.sleep(0.007)
						elif(len(vehicles)==8):
							time.sleep(0.009)
						'''
						print("&&&&&&&")
						for i,b in enumerate(s.swarm):							
							print(len(s.swarm))
							'''
							if(pop_flag_arr[i]==0):
								print(pop_flag_arr[i])
								i+=1
								print(i)
							
							if(pop_flag):	
								if(i>=pop_bot_index):
									i+=1
							'''
							current_position = [b.x,b.y]
							print("current_position",current_position)
							
							#count[goal_table[i]]=count[goal_table[i]]+1
							#print("counter array",count)
							#print("BOT {} reached",i)
							#print("goal_table[goal_index]",goal_table)
							ind=goal_table[i]
							print("ind",ind)
							next_goal=bot_paths[ind]
							print("next_goal",next_goal)
							if(step==2):
								length_arr[i]=len(bot_paths[ind])
								new_length_arr[i]=length_arr[i]
							if(count[i]==length_arr[i]):
								continue
							#print("length_arr",length_arr)
							#print("new_length_arr",new_length_arr)
							if(len(bot_paths[ind])<1):							
								#print("BOT {i} goal reached",i)
								continue
							#print("len(bot_paths[ind]",len(bot_paths[ind]))
							#for i,b in enumerate(s.swarm):
							#current_position=[b.x,b.y]
							goal=bot_paths[goal_table[i]][count[i]]
							print("goal",goal)
							print("multiple_goals[0]",multiple_goals[0])
							#if(goal==multiple_goals[1]):
							#	print("INitial time.sleep")
							#	time.sleep(20)
							#Base control
							#cmd = base_control.base_control(i,b,goal)
							#cmd+= base_control.obstacle_avoidance(i,b,goal)
							
							#Behaviour
							#cmd+=line(b)
							print("111111",i,b,goal)
							#cmd =cvg.goal_area_cvg(i,b,goal,use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[30.0,30.0])
							cmd =cvg.goal_area_cvg(i,b,goal)
							
							#Execute
							cmd.exec(b)
							print("2222222",i,b,goal)
							#print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
							dx=abs(goal[0]-current_position[0])
							dy=abs(goal[1]-current_position[1])						
							print("dx,dy",dx,dy)
							if(dx<=10 and dy<=10):
								bot_array[i]=1
								print("bot_array",bot_array)
								#if len(bot_array)==len(vehicles):
									#all_bot_reach_flag=True									
							count_1=0
							#print("count_1",count_1)
							for j in range(len(vehicles)):
								#print("************")
								if(bot_array[j]==1):
									count_1+=1
								if(count_1==len(vehicles)):
									count_1=0
									bot_array=[0]*len(vehicles)
									all_bot_reach_flag=True

							#print("bot_array",bot_array)
							if (all_bot_reach_flag==True):								
								#print("Goal_reached!!!!!!!!!!!")
								with open(r'/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv', 'a') as csvfile:
									fieldnames = ['waypoint']
									writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
									writer.writerow({'waypoint':multiple_goals.index(goal)})	
								new_length_arr[i]=new_length_arr[i]-1
								#print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
								#count[i]+=1
								for k in range(len(vehicles)):
									count[k]+=1
									
								all_bot_reach_flag=False
								bot_array=[0]*len(vehicles)
								#print("goal,bot_paths[ind][-1]",goal,bot_paths[ind][-1])
								if (goal==bot_paths[ind][-1]):
									print("Break")
									start_flag=False
									search_height_flag=True						
									break
					
							
							current_position = [b.x*2,b.y*2]
							#print("Drone 1 position",b.x,b.y)
							lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
							
							#print ("Drone 1 lat,lon", lat,lon)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)
							if i==0:
							    robot1_x,robot1_y={b.x},{b.y}
							elif i==1:
							    robot2_x,robot2_y={b.x},{b.y}
							elif i==2:
							    robot3_x,robot3_y={b.x},{b.y}
							elif i==3:
							    robot4_x,robot4_y={b.x},{b.y}
							elif i==4:
							    robot5_x,robot5_y={b.x},{b.y}
							elif i==5:
							    robot6_x,robot6_y={b.x},{b.y}
							elif i==6:
							    robot7_x,robot7_y={b.x},{b.y}
							elif i==7:
							    robot8_x,robot8_y={b.x},{b.y}
							elif i==8:
							    robot9_x,robot9_y={b.x},{b.y}
							elif i==9:
							    robot10_x,robot10_y={b.x},{b.y}
							elif i==10:
							    robot11_x,robot11_y={b.x},{b.y}
							elif i==11:
							    robot12_x,robot12_y={b.x},{b.y}
							elif i==12:
							    robot13_x,robot13_y={b.x},{b.y}
							elif i==13:
							    robot14_x,robot14_y={b.x},{b.y}
							elif i==14:
							    robot15_x,robot15_y={b.x},{b.y}
							elif i==15:
							    robot16_x,robot16_y={b.x},{b.y}
							else:
							    print('invalid')
							msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
						       str(robot2_x) + ',' + str(robot2_y) + ',' +
						       str(robot3_x) + ',' + str(robot3_y) + ',' +
						       str(robot4_x) + ',' + str(robot4_y) + ',' +
						       str(robot5_x) + ',' + str(robot5_y) + ',' +
						       str(robot6_x) + ',' + str(robot6_y) + ',' +
						       str(robot7_x) + ',' + str(robot7_y) + ',' +
						       str(robot8_x) + ',' + str(robot8_y) + ',' +
						       str(robot9_x) + ',' + str(robot9_y) + ',' +
						       str(robot10_x) + ',' + str(robot10_y))
										       
									
						sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
		
					#gui.update()
					
					if(index==b"stop"):
						start_flag=False
						break
					if(index==b"search"):
						search_flag=True
						
					if(search_height_flag):
						start_flag=False
						velocity_flag=True	
						same_alt_flag=True
						same_height=30
						
						if(pop_flag):
							same_height+=3
						
						alt=[0]*len(vehicles)
						alt_count=[0]*len(vehicles)
						alt_count1=0
						print("Velocity zero called##############!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")	
						while True:	
							#time.sleep(0.01)
							if(disperse_flag):
								break				
							print("velocity_flag",velocity_flag)						
							for i, b in enumerate(s.swarm):
								'''
								if(pop_flag):	
									if(i>=pop_bot_index):
										i+=1
								'''
								#print("Velocity zero called##############!!!!!!!!!!!")
								cmd = potf.velocity(b.get_position(), b.sim, weights=potf.field_weights, order = 2, max_dist=5)
								cmd.exec(b)
								# Break out of the loop
								#print("Velocity zero!!!!!!!!!!!")			
								value=[b.x*2,b.y*2]
								
								#gui.show_coverage(area_covered,elapsed_time)

								
								#print("Velocity Bot State",b.state)

								print("Drone 1 position",b.x,b.y)
								lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
								#print ("Drone 1 lat,lon", lat,lon)
								if same_alt_flag:
									point1 = LocationGlobalRelative(lat,lon,same_height)
								else:
									point1 = LocationGlobalRelative(lat,lon,different_height[i])
								#print("point1",point1)
								vehicles[i].simple_goto(point1)
							
								for i,vehicle in enumerate(vehicles):
									alt[i]=vehicle.location.global_relative_frame.alt 
									print("alt[vehicle]",alt[i])
									if same_height - 1.5 <= alt[i] <= same_height + 1.5:
										alt_count[i]=1
											#alt_count1+=1										
										if all(count==1 for count in alt_count):	
										#if(alt_count1==len(vehicles)):
											print("Reached target altitude")
											search_height_flag=False
											disperse_flag=True
											break
								
					if(index==b"disperse") or (disperse_flag):
							#	sent = graph_socket.sendto(str("disperse").encode(), graph_server_address)				
								index="data"	
								print("Returning home!!!!!")
								
								#multiple_goals=[ (221.04615149725555,254.56759356137087),  (221.9322780757031,255.69734634923498),(239.63579710743477,301.23115906075583), (294.0616775306884,277.02453408077747), (276.473974299815,211.34501506606307), (273.1208811487024,223.07687219898858), (239.12526531257475,223.8207250763061), (225.92999666314563,265.8292381347515), (210.5570108343163,318.95569808127925), (262.75298071251274,216.203910509002)]
								multiple_goals=[ (221.04615149725555,254.56759356137087),  (221.9322780757031,255.69734634923498),(239.63579710743477,301.23115906075583), (294.0616775306884,277.02453408077747), (276.473974299815,211.34501506606307), (196.58305830790317, 267.82263300526506), (239.12526531257475,223.8207250763061), (225.92999666314563,265.8292381347515), (210.5570108343163,318.95569808127925), (262.75298071251274,216.203910509002)]
								'''
								bot_paths={
									   0:[multiple_goals[0],multiple_goals[1],multiple_goals[4]],				   
									   1:[multiple_goals[1],multiple_goals[4]],				   
									   2:[multiple_goals[2],multiple_goals[1],multiple_goals[4]],				   
									   3:[multiple_goals[3],multiple_goals[4]],				   
									   
									   4:[multiple_goals[5],multiple_goals[6],multiple_goals[4]],
									   5:[multiple_goals[6],multiple_goals[4]],
									   6:[multiple_goals[7],multiple_goals[10],multiple_goals[6],multiple_goals[4]],
									   7:[multiple_goals[8],multiple_goals[1],multiple_goals[4]],
									   8:[multiple_goals[9],multiple_goals[2],multiple_goals[1],multiple_goals[4]],
									   9:[multiple_goals[10],multiple_goals[6],multiple_goals[4]],
									   
								}
								'''
								disperse_bot_goal=[0]*len(vehicles)
								YOUR_GOAL_THRESHOLD_DISTANCE=15
								YOUR_THRESHOLD_DISTANCE=15
								current_goal_index = [0] * len(vehicles)			
								goal_table=[0]*len(vehicles)
								print("goal_table",goal_table)
								current_table=[0]*len(vehicles)
								length_arr=[0]*len(vehicles)
								new_length_arr=[0]*len(vehicles)
								count=[0]*len(vehicles)
								step=0
								all_bot_reach_flag_rev=False
								bot_array_rev=[0]*len(vehicles)
								while True:
									for i,b in enumerate(s.swarm):
										'''
										if(pop_flag):	
											if(i>=pop_bot_index):
												i+=1
										'''
										current_position = [b.x,b.y]
										
										#cmd =cvg.goal_area_cvg(i,b,goal,use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[30.0,30.0])
										cmd =cvg.goal_area_cvg(i,b,multiple_goals[i])
										#cmd = base_control.base_control(i,b,goal)
										cmd+= base_control.obstacle_avoidance(i,b,multiple_goals[i])
										#Behaviour
										#cmd+=line(b)
										cmd.exec(b)
										
										dx=abs(multiple_goals[i][0]-current_position[0])
										dy=abs(multiple_goals[i][1]-current_position[1])						
										#print("dx,dy",dx,dy)
										if(dx<=3 and dy<=3):									
											#print("Goal_reached!!!!!!!!!!!")	
											new_length_arr[i]=new_length_arr[i]-1
											#print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
											disperse_bot_goal[i]=1
											print("disperse_bot_goal",disperse_bot_goal)
											#bot_array_rev[i]=1
										
										#if(goal==bot_paths[goal_table[i]][-1]):
										#	print("Drone ",i,"reached")
										#	bot_array_rev[i]=1
										
										if all(goal==1 for goal in disperse_bot_goal):
											disperse_flag=False
											search_flag=True
											print(search_flag,"search_flag")
											break
												
										current_position =[b.x*2,b.y*2]
										#print("Drone 1 position",b.x,b.y)
										lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
										#print ("Drone 1 lat,lon", lat,lon)
										if same_alt_flag:
											point1 = LocationGlobalRelative(lat,lon,same_height)
										else:
											point1 = LocationGlobalRelative(lat,lon,different_height[i])
										#print("point1",point1)
										vehicles[i].simple_goto(point1)	
										if i==0:
										    robot1_x,robot1_y={b.x},{b.y}
										elif i==1:
										    robot2_x,robot2_y={b.x},{b.y}
										elif i==2:
										    robot3_x,robot3_y={b.x},{b.y}
										elif i==3:
										    robot4_x,robot4_y={b.x},{b.y}
										elif i==4:
										    robot5_x,robot5_y={b.x},{b.y}
										elif i==5:
										    robot6_x,robot6_y={b.x},{b.y}
										elif i==6:
										    robot7_x,robot7_y={b.x},{b.y}
										elif i==7:
										    robot8_x,robot8_y={b.x},{b.y}
										elif i==8:
										    robot9_x,robot9_y={b.x},{b.y}
										elif i==9:
										    robot10_x,robot10_y={b.x},{b.y}
										elif i==10:
										    robot11_x,robot11_y={b.x},{b.y}
										elif i==11:
										    robot12_x,robot12_y={b.x},{b.y}
										elif i==12:
										    robot13_x,robot13_y={b.x},{b.y}
										elif i==13:
										    robot14_x,robot14_y={b.x},{b.y}
										elif i==14:
										    robot15_x,robot15_y={b.x},{b.y}
										elif i==15:
										    robot16_x,robot16_y={b.x},{b.y}
										else:
										    print('invalid')
										msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
									       str(robot2_x) + ',' + str(robot2_y) + ',' +
									       str(robot3_x) + ',' + str(robot3_y) + ',' +
									       str(robot4_x) + ',' + str(robot4_y) + ',' +
									       str(robot5_x) + ',' + str(robot5_y) + ',' +
									       str(robot6_x) + ',' + str(robot6_y) + ',' +
									       str(robot7_x) + ',' + str(robot7_y) + ',' +
									       str(robot8_x) + ',' + str(robot8_y) + ',' +
									       str(robot9_x) + ',' + str(robot9_y) + ',' +
									       str(robot10_x) + ',' + str(robot10_y))
													       
												
									sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
												#gui.update()						
									if(index==b"search") or (search_flag):
										disperse_flag=false
										search_flag=True
										break
												
									
					if(index==b"search") or (search_flag):
						sent = graph_socket.sendto(str("search").encode(), graph_server_address)				
						index="data"
						
						uav_home_pos=[]
						for vehicle in vehicles:
						    lat = vehicle.location.global_relative_frame.lat
						    lon = vehicle.location.global_relative_frame.lon

						    # Process the lat and lon as needed
						    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
						    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
						    print("x,y",x/2,y/2)
						    uav_home_pos.append((x / 2, y / 2))
						
						#gui.close()
						#s = sim.Simulation(uav_home_pos,num_bots=8, env_name='rectangles1' )
						utils.robot.DEFAULT_SIZE= 0.4
						s = sim.Simulation(uav_home_pos,num_bots=len(vehicles), env_name='rectangles1' )
						gui = viz.Gui(s)
						#gui.show_env()
						#gui.show_bots()
						gui.update()
						gui.close()
						
						print("Search Started")
							
						search_start_time = time.time()
						while 1:
							for i,b in enumerate(s.swarm):									
								area=gui.show_coverage(area_covered,elapsed_time)	
								if(area>79):
									aggregate_flag=True
									break													
								#coverage = s.update_grid()
								'''	
								if(pop_flag):	
									if(i>=pop_bot_index):
										i+=1
								'''
								#cmd = cvg.disp_exp_area_cvg(b)
								cmd = cvg.disp_exp_area_cvg(b, use_base_control=True, exp_weight_params=[25.0,25.5],disp_weight_params=[50.0,50.0]) 
								#cmd = cvg.disp_exp_area_cvg(b, use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[50.0,50.0]) 
								value=[b.x*2,b.y*2]
								cmd.exec(b)
								print("value",i,value)
								#print("b.x,b.y",b.x,b.y)
								position = (round(b.x), round(b.y))
								# Check if the position has not been visited before
								if position not in visited_positions:
								    visited_positions.add(position)
								robot_positions[i] = [b.x, b.y]
								visited_positions.update(tuple(map(int, pos)) for pos in robot_positions)
								#print("x,y",i,b.x,b.y)
								#print(f"Robot {i + 1}: x={b.x}, y={b.y}")
								#print("elapsed_time",elapsed_time)
								elapsed_time = time.time() - search_start_time
								#print("elapsed_time",elapsed_time)
								gui.show_coverage(area_covered,elapsed_time)
								
								
								#print("Bot State",b.state)
								if pop_flag_arr[i]==1:
									#print("Drone 1 position",b.x,b.y)
									lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
									#print ("Drone 1 lat,lon", lat,lon)
									if same_alt_flag:
										point1 = LocationGlobalRelative(lat,lon,same_height)
									else:
										point1 = LocationGlobalRelative(lat,lon,different_height[i])
									#print("point1",point1)
									vehicles[i].simple_goto(point1)
								
								if i==0:
								    robot1_x,robot1_y={b.x},{b.y}
								elif i==1:
								    robot2_x,robot2_y={b.x},{b.y}
								elif i==2:
								    robot3_x,robot3_y={b.x},{b.y}
								elif i==3:
								    robot4_x,robot4_y={b.x},{b.y}
								elif i==4:
								    robot5_x,robot5_y={b.x},{b.y}
								elif i==5:
								    robot6_x,robot6_y={b.x},{b.y}
								elif i==6:
								    robot7_x,robot7_y={b.x},{b.y}
								elif i==7:
								    robot8_x,robot8_y={b.x},{b.y}
								elif i==8:
								    robot9_x,robot9_y={b.x},{b.y}
								elif i==9:
								    robot10_x,robot10_y={b.x},{b.y}
								elif i==10:
								    robot11_x,robot11_y={b.x},{b.y}
								elif i==11:
								    robot12_x,robot12_y={b.x},{b.y}
								elif i==12:
								    robot13_x,robot13_y={b.x},{b.y}
								elif i==13:
								    robot14_x,robot14_y={b.x},{b.y}
								elif i==14:
								    robot15_x,robot15_y={b.x},{b.y}
								elif i==15:
								    robot16_x,robot16_y={b.x},{b.y}
								else:
								    print('invalid')
								msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
							       str(robot2_x) + ',' + str(robot2_y) + ',' +
							       str(robot3_x) + ',' + str(robot3_y) + ',' +
							       str(robot4_x) + ',' + str(robot4_y) + ',' +
							       str(robot5_x) + ',' + str(robot5_y) + ',' +
							       str(robot6_x) + ',' + str(robot6_y) + ',' +
							       str(robot7_x) + ',' + str(robot7_y) + ',' +
							       str(robot8_x) + ',' + str(robot8_y)+',' +
							       str(robot9_x) + ',' + str(robot9_y)+',' +
							       str(robot10_x) + ',' + str(robot10_y))
											       
							area_covered=s.update_grid()				
							sent = sock.sendto(str(msg).encode(), server_address)				
							#gui.update_trajectory_plot()
							#print("!!!!!!!!!!area_covered",area_covered)
							#coverage_area = (len(visited_positions) / (s.grid.size)) * 1000
							#print(f"Area covered = {coverage_area:.3f}%")		
							#elapsed_time = time.time() - search_start_time
							#gui.show_coverage(area_covered,elapsed_time)
							area=gui.show_coverage(area_covered,elapsed_time)
							print("!!!!!!!!!!!!AREA covered!!!!!!!!",area,elapsed_time)
							msg=(str('search')+','+str(area)+','+str(elapsed_time))
							sent = sock.sendto(str(msg).encode(), server_address)
			
							#s.update_grid()
							#gui.update()
							s.time_elapsed += 1   

							if(index==b"stop") or (flag_stop):
								search_flag=False
								break
								
							if(index==b"aggregate") or (aggregate_flag):
								search_flag=False
								sent = graph_socket.sendto(str("aggregate").encode(), graph_server_address)				
								index="data"	
								print("Returning home!!!!!")
								
								multiple_goals=[ (211.04615149725555,236.56759356137087),  (221.9322780757031,255.69734634923498),(239.63579710743477,301.23115906075583), (294.0616775306884,277.02453408077747), (274.26992917946745,237.7249442631597), (276.473974299815,211.34501506606307), (273.1208811487024,223.07687219898858), (239.12526531257475,220.8207250763061), (225.92999666314563,265.8292381347515), (210.5570108343163,318.95569808127925), (262.75298071251274,216.203910509002), (290.83411085872586,223.81593558812182)]
								
								
								bot_paths={
									   0:[multiple_goals[0],multiple_goals[1],multiple_goals[4]],				   
									   1:[multiple_goals[1],multiple_goals[4]],				   
									   2:[multiple_goals[2],multiple_goals[1],multiple_goals[4]],				   
									   3:[multiple_goals[3],multiple_goals[4]],				   
									   4:[multiple_goals[4]],
									   5:[multiple_goals[5],multiple_goals[6],multiple_goals[4]],
									   6:[multiple_goals[6],multiple_goals[4]],
									   7:[multiple_goals[7],multiple_goals[10],multiple_goals[6],multiple_goals[4]],
									   8:[multiple_goals[8],multiple_goals[1],multiple_goals[4]],
									   9:[multiple_goals[9],multiple_goals[2],multiple_goals[1],multiple_goals[4]],
									   10:[multiple_goals[10],multiple_goals[6],multiple_goals[4]],
									   11:[multiple_goals[11],multiple_goals[4]],
								}
								
								YOUR_GOAL_THRESHOLD_DISTANCE=15
								YOUR_THRESHOLD_DISTANCE=15
								current_goal_index = [0] * len(vehicles)			
								goal_table=[0]*len(vehicles)
								print("goal_table",goal_table)
								current_table=[0]*len(vehicles)
								length_arr=[0]*len(vehicles)
								new_length_arr=[0]*len(vehicles)
								count=[0]*len(vehicles)
								step=0
								all_bot_reach_flag_rev=False
								bot_array_rev=[0]*len(vehicles)
								while True:
									#time.sleep(0.01)
									if(return_height_flag):
										break
									#print("############")
									step+=1
									#print("step",step)
									if(step==1):										
										for i,b in enumerate(s.swarm):
											#print("************")
											#print("potf.reached_goals",potf.reached_goals)							
											#if(potf.reached_goals[i]==False):
											current_position = [b.x,b.y]
											nearest_goal = min(multiple_goals, key=lambda goal: distance.euclidean(current_position, goal))
											#print("current_position , nearest_goal",current_position,nearest_goal)
											goal_table[i]=multiple_goals.index(nearest_goal)
											#print("!!!!!!!!!!!current_table!!!!!!!",current_table)
											print("!!!!!!!!!!!goalTable!!!!!!!",goal_table)
										#print("%%%%%%%")
											
									
									else:
										#print("@@@@@")
										for i,b in enumerate(s.swarm):
											'''
											if(pop_flag):	
												if(i>=pop_bot_index):
													i+=1
											'''
											current_position = [b.x,b.y]
											#print("current_position",current_position)
											
											#count[goal_table[i]]=count[goal_table[i]]+1
											#print("counter array",count)
											#print("BOT {} reached",i)
											#print("goal_table[goal_index]",goal_table)
											ind=goal_table[i]
											#print("ind",ind)
											next_goal=bot_paths[ind]
											#print("next_goal",next_goal)
											if(step==2):
												length_arr[i]=len(bot_paths[ind])
												new_length_arr[i]=length_arr[i]
											if(count[i]==length_arr[i]):
												continue
											#print("length_arr",length_arr)
											#print("new_length_arr",new_length_arr)
											if(len(bot_paths[ind])<1):							
												print("BOT {i} goal reached",i)
												continue
											#print("len(bot_paths[ind]",len(bot_paths[ind]))
											#for i,b in enumerate(s.swarm):
											#current_position=[b.x,b.y]
											goal=bot_paths[goal_table[i]][count[i]]
											#print("goal",goal)
											#cmd =cvg.goal_area_cvg(i,b,goal,use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[30.0,30.0])
											cmd =cvg.goal_area_cvg(i,b,goal)
											#cmd = base_control.base_control(i,b,goal)
											#cmd+= base_control.obstacle_avoidance(i,b,goal)
											#Behaviour
											#cmd+=line(b)
											cmd.exec(b)
											
											dx=abs(goal[0]-current_position[0])
											dy=abs(goal[1]-current_position[1])						
											#print("dx,dy",dx,dy)
											if(dx<=5 and dy<=5):									
												#print("Goal_reached!!!!!!!!!!!")	
												new_length_arr[i]=new_length_arr[i]-1
												print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
												count[i]+=1
												#bot_array_rev[i]=1
											
											if(goal==bot_paths[goal_table[i]][-1]):
												print("Drone ",i,"reached")
												bot_array_rev[i]=1
											print("bot_array_rev",bot_array_rev)
											count_1=0	
											for j in range(len(vehicles)):
												print("************")
												if(bot_array_rev[j]==1):
													count_1+=1
												print("count_1",count_1)
												if(count_1==len(vehicles)):
													#all_bot_reach_flag_1=True
													aggregate_flag=False
													return_height_flag=True
													print(return_height_flag,"return_height_flag")
													break
											'''
											if(new_length_arr[i]==0):
												print("Break")
												cmd = base_control.base_control(i,b,goal)
												cmd+= base_control.obstacle_avoidance(i,b,goal)
												#Behaviour
												cmd+=circle(b,35)
												cmd+=disp_field(b)
												#Execute
												cmd.exec(b)
												return_height_flag=True
												break
												#exit()
											'''			
											current_position =[b.x*2,b.y*2]
											#print("Drone 1 position",b.x,b.y)
											lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
											#print ("Drone 1 lat,lon", lat,lon)
											if same_alt_flag:
												point1 = LocationGlobalRelative(lat,lon,same_height)
											else:
												point1 = LocationGlobalRelative(lat,lon,different_height[i])
											#print("point1",point1)
											vehicles[i].simple_goto(point1)	
											if i==0:
											    robot1_x,robot1_y={b.x},{b.y}
											elif i==1:
											    robot2_x,robot2_y={b.x},{b.y}
											elif i==2:
											    robot3_x,robot3_y={b.x},{b.y}
											elif i==3:
											    robot4_x,robot4_y={b.x},{b.y}
											elif i==4:
											    robot5_x,robot5_y={b.x},{b.y}
											elif i==5:
											    robot6_x,robot6_y={b.x},{b.y}
											elif i==6:
											    robot7_x,robot7_y={b.x},{b.y}
											elif i==7:
											    robot8_x,robot8_y={b.x},{b.y}
											elif i==8:
											    robot9_x,robot9_y={b.x},{b.y}
											elif i==9:
											    robot10_x,robot10_y={b.x},{b.y}
											elif i==10:
											    robot11_x,robot11_y={b.x},{b.y}
											elif i==11:
											    robot12_x,robot12_y={b.x},{b.y}
											elif i==12:
											    robot13_x,robot13_y={b.x},{b.y}
											elif i==13:
											    robot14_x,robot14_y={b.x},{b.y}
											elif i==14:
											    robot15_x,robot15_y={b.x},{b.y}
											elif i==15:
											    robot16_x,robot16_y={b.x},{b.y}
											else:
											    print('invalid')
											msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
										       str(robot2_x) + ',' + str(robot2_y) + ',' +
										       str(robot3_x) + ',' + str(robot3_y) + ',' +
										       str(robot4_x) + ',' + str(robot4_y) + ',' +
										       str(robot5_x) + ',' + str(robot5_y) + ',' +
										       str(robot6_x) + ',' + str(robot6_y) + ',' +
										       str(robot7_x) + ',' + str(robot7_y) + ',' +
										       str(robot8_x) + ',' + str(robot8_y) + ',' +
										       str(robot9_x) + ',' + str(robot9_y) + ',' +
										       str(robot10_x) + ',' + str(robot10_y))
														       
													
										sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
													#gui.update()														
									#print("&&&&&&&&&&&&")											
									if(index==b"stop"):
										sent = graph_socket.sendto(str("stop").encode(), graph_server_address)				
										index="data"
										print("#####################################")
										flag_stop=True
										data=b"stop"
										break
									if(index==b"goal_start"):
										index="data"
										print("#####################################")
										flag_stop=False
								
									if(return_height_flag):
										velocity_flag=True	
										same_alt_flag=False
										same_height=30
										alt=[0]*len(vehicles)
										alt_count=[0]*len(vehicles)
										alt_count1=0
										print("Velocity zero called##############!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")	
										while True:	
											if(return_flag):
												print("REturn####")
												break				
											print("velocity_flag",velocity_flag)						
											for i, b in enumerate(s.swarm):
												'''
												if(pop_flag):	
													if(i>=pop_bot_index):
														i+=1
												'''
												#print("Velocity zero called##############!!!!!!!!!!!")
												cmd = potf.velocity(b.get_position(), b.sim, weights=potf.field_weights, order = 2, max_dist=5)
												cmd.exec(b)
												# Break out of the loop
												#print("Velocity zero!!!!!!!!!!!")			
												value=[b.x*2,b.y*2]												
												#gui.show_coverage(area_covered,elapsed_time)
												#print("Velocity Bot State",b.state)

												print("Drone 1 position",b.x,b.y)
												lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
												#print ("Drone 1 lat,lon", lat,lon)
												if same_alt_flag:
													point1 = LocationGlobalRelative(lat,lon,same_height)
												else:
													point1 = LocationGlobalRelative(lat,lon,different_height[i])
												#print("point1",point1)
												vehicles[i].simple_goto(point1)
												print("return_flag",return_flag)
												for i,vehicle in enumerate(vehicles):
													alt[i]=vehicle.location.global_relative_frame.alt 
													print("alt[vehicle]",alt[i])
													#if same_height - 1.5 <= alt[i] <= same_height+1.5: 											
													if different_height[i] - 1.5 <= alt[i] <= different_height[i]+1.5:
														alt_count[i]=1
														#alt_count1+=1
														if all(count==1 for count in alt_count):		
														#if(alt_count1==len(vehicles)):
															print("Reached target altitude")
															return_flag=True
															return_height_flag=false
															break
								
									
									if(index==b"return") or (return_flag):
										index="data"
										sent = graph_socket.sendto(str("return").encode(), graph_server_address)				
										uav_home_pos=[]
										for vehicle in vehicles:
										    lat = vehicle.location.global_relative_frame.lat
										    lon = vehicle.location.global_relative_frame.lon

										    # Process the lat and lon as needed
										    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
										    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
										    print("x,y",x/2,y/2)
										    uav_home_pos.append((x / 2, y / 2))						
										#gui.close()
										s = sim.Simulation(uav_home_pos,num_bots=len(vehicles), env_name='rectangles2' )
										#s = sim.Simulation(num_bots, env_name='rectangles2' )
										#gui = viz.Gui(s)
										#gui.show_env()
										#gui.show_bots()
										#gui.update()
										
										for b in s.swarm:
											'''
											if(pop_flag):	
												if(i>=pop_bot_index):
													i+=1
											'''
											multiple_goals=[(157.64131776911836,209.16847386901821), (360.12460897774656,143.7605934973306), (581.468738086984,77.37929794272104),  (599.2505498180748,112.92467247032282),  (602.6444203564429,153.256210887913), (508.0359474466252,199.22290470278386),(356.44627567858686,213.57908756056852), (290.65645997127996,232.81715396747316),]
											multiple_goals1=[(356.44627567858686,213.57908756056852), (290.65645997127996,232.81715396747316)]
											bot_paths={
												   0:[multiple_goals[0]],				   
												   1:[multiple_goals[1],multiple_goals[0]],
												   2:[multiple_goals[2],multiple_goals[1],multiple_goals[0]],
												   3:[multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
												   4:[multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
												   5:[multiple_goals[5],multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
												   6:[multiple_goals[6],multiple_goals[5],multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
												   7:[multiple_goals[7],multiple_goals[6],multiple_goals[5],multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
												  

											}
											
											YOUR_GOAL_THRESHOLD_DISTANCE=15
											YOUR_THRESHOLD_DISTANCE=15
											current_goal_index = [0] * len(vehicles)			
											goal_table=[0]*len(vehicles)
											print("goal_table",goal_table)
											current_table=[0]*len(vehicles)
											length_arr=[0]*len(vehicles)
											new_length_arr=[0]*len(vehicles)
											count=[0]*len(vehicles)
											step=0
											search_flag=False
											bot_array_return=[0]*len(vehicles)
											all_bot_reach_flag_1=False

											while 1:
												'''
												if(num_bots==10):
													time.sleep(0.003)
												elif(len(vehicles)==9):
													time.sleep(0.007)
												elif(len(vehicles)==8):
													time.sleep(0.009)
												'''
												#print("############")
												step+=1
												#print("step",step)
												#if(home_flag):
												#	break
												if(step==1):
													print("step",step)
													for i,b in enumerate(s.swarm):
														#print("potf.reached_goals",potf.reached_goals)							
														#if(potf.reached_goals[i]==False):
														current_position = [b.x,b.y]
														#nearest_goal = min(multiple_goals1, key=lambda goal: distance.euclidean(current_position, goal))
														#print("current_position , nearest_goal",current_position,nearest_goal)
														#goal_ind=multiple_goals.index(nearest_goal)
														with open('/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv','rt')as text:
															data = csv.reader(text)
															for row in data: 
																data = (row)
																print ("loc",data[-1])
																nextwaypoint=int(data[-1])							
														
														print("nextwaypoint",nextwaypoint)
														goal_table[i]=nextwaypoint
														'''
														print("goal_ind",goal_ind)
														print(len(bot_paths[goal_ind]),"len(bot_paths[goal_ind])")
														print("len(bot_paths[goal_ind+1])",len(bot_paths[goal_ind+1]))
														if (len(bot_paths[goal_ind]))>(len(bot_paths[goal_ind+1])):
															goal_table[i]=multiple_goals.index(nearest_goal)+1								
															print("multiple_goals.index(nearest_goal)+1",multiple_goals.index(nearest_goal)+1)								
														#print("current_position , nearest_goal",current_position,nearest_goal)
														else:
															goal_table[i]=multiple_goals.index(nearest_goal)
														'''
														#goal_table[i]=multiple_goals.index(nearest_goal)
														#print("!!!!!!!!!!!current_table!!!!!!!",current_table)
														print("!!!!!!!!!!!goalTable!!!!!!!",goal_table)
														
												
												else:
													for i,b in enumerate(s.swarm):
														current_position = [b.x,b.y]
														#print("current_position",current_position)
														
														#count[goal_table[i]]=count[goal_table[i]]+1
														#print("counter array",count)
														#print("BOT {} reached",i)
														#print("goal_table[goal_index]",goal_table)
														ind=goal_table[i]
														#print("ind",ind)
														next_goal=bot_paths[ind]
														#print("next_goal",next_goal)
														print("!!!!!!!!!!!goalTable!!!!!!!",goal_table)
														if(step==2):
															length_arr[i]=len(bot_paths[ind])
															new_length_arr[i]=length_arr[i]
														if(count[i]==length_arr[i]):
															continue
														#print("length_arr",length_arr)
														#print("new_length_arr",new_length_arr)
														if(len(bot_paths[ind])<1):							
															print("BOT {i} goal reached",i)
															continue
														#print("len(bot_paths[ind]",len(bot_paths[ind]))
														#for i,b in enumerate(s.swarm):
														#current_position=[b.x,b.y]
														goal=bot_paths[goal_table[i]][count[i]]
														print("goal",goal)
														#Base control
														#cmd = base_control.base_control(i,b,goal)
														#cmd+= base_control.obstacle_avoidance(i,b,goal)
														
														#Behaviour
														#cmd+=line(b)
														#cmd =cvg.goal_area_cvg(i,b,goal,use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[30.0,30.0])
														cmd =cvg.goal_area_cvg(i,b,goal)
														#Execute
														cmd.exec(b)
														#print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
														dx=abs(goal[0]-current_position[0])
														dy=abs(goal[1]-current_position[1])						
														#print("dx,dy",dx,dy)
														if(dx<=10 and dy<=10):
															bot_array_return[i]=1
															#print("bot_array",bot_array)
								
														count_1=0
														print("count_1",count_1)
														for j in range(len(vehicles)):
															#print("************")
															if(bot_array_return[j]==1):
																count_1+=1
															if(count_1==len(vehicles)):
																count_1=0
																bot_array_return=[0]*len(vehicles)
																all_bot_reach_flag_1=True

														#print("bot_array",bot_array)
														if (all_bot_reach_flag_1==True):								
															#print("Goal_reached!!!!!!!!!!!")	
															with open(r'/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv', 'a') as csvfile:
																fieldnames = ['waypoint']
																writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
																writer.writerow({'waypoint':multiple_goals.index(goal)})	
															new_length_arr[i]=new_length_arr[i]-1
															print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
															#count[i]+=1
															for k in range(len(vehicles)):
																count[k]+=1
															all_bot_reach_flag_1=False
															bot_array_return=[0]*len(vehicles)
															print("goal,bot_paths[ind][-1]",goal,bot_paths[ind][-1])
															if (goal==bot_paths[ind][-1]):
																print("Break")
																return_flag=False
																home_flag=True						
																break		
														'''			
														if (new_length_arr[i]==-1):
															print("Break")
															cmd = base_control.base_control(i,b,goal)
															cmd+= base_control.obstacle_avoidance(i,b,goal)
															#Behaviour
															cmd+=line(b)
															#Execute
															cmd.exec(b)
															home_flag=True
															break	
														'''
														current_position = [b.x*2,b.y*2]
														print("Drone 1 position",b.x,b.y)
														lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
														#print ("Drone 1 lat,lon", lat,lon)
														if same_alt_flag:
															point1 = LocationGlobalRelative(lat,lon,same_height)
														else:
															point1 = LocationGlobalRelative(lat,lon,different_height[i])
														vehicles[i].simple_goto(point1)
														
														if i==0:
														    robot1_x,robot1_y={b.x},{b.y}
														elif i==1:
														    robot2_x,robot2_y={b.x},{b.y}
														elif i==2:
														    robot3_x,robot3_y={b.x},{b.y}
														elif i==3:
														    robot4_x,robot4_y={b.x},{b.y}
														elif i==4:
														    robot5_x,robot5_y={b.x},{b.y}
														elif i==5:
														    robot6_x,robot6_y={b.x},{b.y}
														elif i==6:
														    robot7_x,robot7_y={b.x},{b.y}
														elif i==7:
														    robot8_x,robot8_y={b.x},{b.y}
														elif i==8:
														    robot9_x,robot9_y={b.x},{b.y}
														elif i==9:
														    robot10_x,robot10_y={b.x},{b.y}
														elif i==10:
														    robot11_x,robot11_y={b.x},{b.y}
														elif i==11:
														    robot12_x,robot12_y={b.x},{b.y}
														elif i==12:
														    robot13_x,robot13_y={b.x},{b.y}
														elif i==13:
														    robot14_x,robot14_y={b.x},{b.y}
														elif i==14:
														    robot15_x,robot15_y={b.x},{b.y}
														elif i==15:
														    robot16_x,robot16_y={b.x},{b.y}
														else:
														    print('invalid')
														msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
													       str(robot2_x) + ',' + str(robot2_y) + ',' +
													       str(robot3_x) + ',' + str(robot3_y) + ',' +
													       str(robot4_x) + ',' + str(robot4_y) + ',' +
													       str(robot5_x) + ',' + str(robot5_y) + ',' +
													       str(robot6_x) + ',' + str(robot6_y) + ',' +
													       str(robot7_x) + ',' + str(robot7_y) + ',' +
													       str(robot8_x) + ',' + str(robot8_y) + ',' +
													       str(robot9_x) + ',' + str(robot9_y) + ',' +
													       str(robot10_x) + ',' + str(robot10_y))
																	       
																
													sent = graph_socket.sendto(str(msg).encode(), graph_server_address)													
																	
											
												#gui.update()
												if(index==b"stop"):
													break
												if(index==b"uav_pos"):
													home_pos=[]
													print("home_pos",home_pos)
													with open(csv_file_path, mode='r') as csv_file:
													    csv_reader = csv.DictReader(csv_file)
													    for row in csv_reader:
													    	x = float(row['X'])
													    	y = float(row['Y'])
													    	home_pos.append((x, y))
													print("home_pos",home_pos)
													home_flag=True
													#break
												
												if(index==b"home") or (home_flag):
													sent = graph_socket.sendto(str("home").encode(), graph_server_address)				
													bot_array_home=[0]*len(vehicles)
													all_bot_reach_flag_home=False
													print("Home.......")
														
													while 1:
														if(home_flag1):
															exit()
														for i,b in enumerate(s.swarm):
															'''
															if(pop_flag):	
																if(i>=pop_bot_index):
																	i+=1
															'''
															current_position = [b.x,b.y]
															#cmd =cvg.goal_area_cvg(i,b,home_pos[i],use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[30.0,30.0])
															cmd =cvg.goal_area_cvg(i,b,home_pos[i])
															goal=home_pos[i]
															#Execute
															cmd.exec(b)
															
															dx=abs(goal[0]-current_position[0])
															dy=abs(goal[1]-current_position[1])						
															print("dx,dy",dx,dy)
															
															if(dx<=0.1 and dy<=0.1):
																bot_array_home[i]=1
																#print("bot_array",bot_array)	
															count_home=0
															print("count_home",count_home)
															for j in range(len(vehicles)):
																#print("************")
																if(bot_array_home[j]==1):
																	count_home+=1
																if(count_home==len(vehicles)):
																	count_home=0
																	bot_array_home=[0]*len(vehicles)
																	all_bot_reach_flag_home=True

															#print("bot_array",bot_array)
															if (all_bot_reach_flag_home==True):
																with open(r'/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv', 'a') as csvfile:
																	fieldnames = ['waypoint']
																	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
																	writer.writerow({'waypoint':-1})	
																home_flag1=True
																home_flag=False
																break	
																						
															current_position = [b.x*2,b.y*2]
															#print("Drone 1 position",b.x,b.y)
															lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
															#print ("Drone 1 lat,lon", lat,lon)
															if same_alt_flag:
																point1 = LocationGlobalRelative(lat,lon,same_height)
															else:
																point1 = LocationGlobalRelative(lat,lon,different_height[i])
															vehicles[i].simple_goto(point1)
															if i==0:
															    robot1_x,robot1_y={b.x},{b.y}
															elif i==1:
															    robot2_x,robot2_y={b.x},{b.y}
															elif i==2:
															    robot3_x,robot3_y={b.x},{b.y}
															elif i==3:
															    robot4_x,robot4_y={b.x},{b.y}
															elif i==4:
															    robot5_x,robot5_y={b.x},{b.y}
															elif i==5:
															    robot6_x,robot6_y={b.x},{b.y}
															elif i==6:
															    robot7_x,robot7_y={b.x},{b.y}
															elif i==7:
															    robot8_x,robot8_y={b.x},{b.y}
															elif i==8:
															    robot9_x,robot9_y={b.x},{b.y}
															elif i==9:
															    robot10_x,robot10_y={b.x},{b.y}
															elif i==10:
															    robot11_x,robot11_y={b.x},{b.y}
															elif i==11:
															    robot12_x,robot12_y={b.x},{b.y}
															elif i==12:
															    robot13_x,robot13_y={b.x},{b.y}
															elif i==13:
															    robot14_x,robot14_y={b.x},{b.y}
															elif i==14:
															    robot15_x,robot15_y={b.x},{b.y}
															elif i==15:
															    robot16_x,robot16_y={b.x},{b.y}
															else:
															    print('invalid')
															msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
														       str(robot2_x) + ',' + str(robot2_y) + ',' +
														       str(robot3_x) + ',' + str(robot3_y) + ',' +
														       str(robot4_x) + ',' + str(robot4_y) + ',' +
														       str(robot5_x) + ',' + str(robot5_y) + ',' +
														       str(robot6_x) + ',' + str(robot6_y) + ',' +
														       str(robot7_x) + ',' + str(robot7_y) + ',' +
														       str(robot8_x) + ',' + str(robot8_y) + ',' +
														       str(robot9_x) + ',' + str(robot9_y) + ',' +
														       str(robot10_x) + ',' + str(robot10_y))
																		       
																	
														sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
														#gui.update()
														if(index==b"stop"):
															home_flag=False
															home_flag1=True
															
													if(index==b"stop"):
														break
		if(data==b"return"):
			sent = graph_socket.sendto(str("return").encode(), graph_server_address)
							
			uav_home_pos=[]
			for vehicle in vehicles:
			    lat = vehicle.location.global_relative_frame.lat
			    lon = vehicle.location.global_relative_frame.lon

			    # Process the lat and lon as needed
			    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
			    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
			    print("x,y",x/2,y/2)
			    
			    uav_home_pos.append((x / 2, y / 2))						
			print("uav_home_pos",uav_home_pos)    
			#gui.close()
			s = sim.Simulation(uav_home_pos,num_bots=len(vehicles), env_name='rectangles2' )
			#s = sim.Simulation(num_bots, env_name='rectangles2' )
			#gui = viz.Gui(s)
			#gui.show_env()
			#gui.show_bots()
			#gui.update()
			
			print("EEEEEEEEEEEEEEEEEEEEEEEE")
			for b in s.swarm:
				print("!!!!!!!!!!!!!!!!!!!!")	
				''' 
				if(pop_flag):	
					if(i>=pop_bot_index):
						i+=1
				'''
				multiple_goals=[ (157.64131776911836,209.16847386901821), (360.12460897774656,143.7605934973306), (581.468738086984,77.37929794272104),  (599.2505498180748,112.92467247032282),  (602.6444203564429,153.256210887913), (508.0359474466252,199.22290470278386),(356.44627567858686,213.57908756056852), (290.65645997127996,232.81715396747316),]
				multiple_goals1=[(157.64131776911836,209.16847386901821), (360.12460897774656,143.7605934973306), (581.468738086984,77.37929794272104),  (599.2505498180748,112.92467247032282),  (602.6444203564429,153.256210887913), (508.0359474466252,199.22290470278386),(356.44627567858686,213.57908756056852), (290.65645997127996,232.81715396747316),]
				bot_paths={
					   0:[multiple_goals[0]],				   
					   1:[multiple_goals[1],multiple_goals[0]],
					   2:[multiple_goals[2],multiple_goals[1],multiple_goals[0]],
					   3:[multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
					   4:[multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
					   5:[multiple_goals[5],multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
					   6:[multiple_goals[6],multiple_goals[5],multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],
					   7:[multiple_goals[7],multiple_goals[6],multiple_goals[5],multiple_goals[4],multiple_goals[3],multiple_goals[2],multiple_goals[1],multiple_goals[0]],

				}
				
				YOUR_GOAL_THRESHOLD_DISTANCE=15
				YOUR_THRESHOLD_DISTANCE=15
				current_goal_index = [0] * len(vehicles)			
				goal_table=[0]*len(vehicles)
				print("goal_table",goal_table)
				current_table=[0]*len(vehicles)
				length_arr=[0]*len(vehicles)
				new_length_arr=[0]*len(vehicles)
				count=[0]*len(vehicles)
				step=0
				search_flag=False
				bot_array_return=[0]*len(vehicles)
				all_bot_reach_flag_1=False
				home_flag1=False
				while 1:	
					'''
					if(num_bots==10):
						time.sleep(0.003)
					elif(len(vehicles)==9):
						time.sleep(0.007)
					elif(len(vehicles)==8):
						time.sleep(0.009)
					'''
					if(home_flag):
						break
					print("############")
					step+=1
					print("step",step)
					if(step==1):
						print("step",step)
						for i,b in enumerate(s.swarm):
							print(len(s.swarm))	#print("potf.reached_goals",potf.reached_goals)							
							#if(potf.reached_goals[i]==False):
							current_position = [b.x,b.y]
							#nearest_goal = min(multiple_goals1, key=lambda goal: distance.euclidean(current_position, goal))
							#print("current_position , nearest_goal",current_position,nearest_goal)
							#goal_ind=multiple_goals.index(nearest_goal)						
							#goal_table[i]=multiple_goals.index(nearest_goal)
							#print("!!!!!!!!!!!current_table!!!!!!!",current_table)
							with open('/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv','rt')as text:
								data = csv.reader(text)
								for row in data: 
									data = (row)
									#print ("loc",data[-1])
									nextwaypoint=int(data[-1])							
							
							print("nextwaypoint",i,nextwaypoint)
							goal_table[i]=nextwaypoint
							print("!!!!!!!!!!!goalTable!!!!!!!",goal_table)
							
					
					else:
						print("$$$$$$$$$$$$$$$$")
						for i,b in enumerate(s.swarm):
							print("***********")
						
							current_position = [b.x,b.y]
							#print("current_position",current_position)
							
							#count[goal_table[i]]=count[goal_table[i]]+1
							#print("counter array",count)
							#print("BOT {} reached",i)
							#print("goal_table[goal_index]",goal_table)
							ind=goal_table[i]
							#print("ind",ind)
							next_goal=bot_paths[ind]
							#print("next_goal",next_goal)
							print("!!!!!!!!!!!goalTable!!!!!!!",goal_table)
							if(step==2):
								length_arr[i]=len(bot_paths[ind])
								new_length_arr[i]=length_arr[i]
							if(count[i]==length_arr[i]):
								continue
							#print("length_arr",length_arr)
							#print("new_length_arr",new_length_arr)
							if(len(bot_paths[ind])<1):							
								print("BOT {i} goal reached",i)
								continue
							#print("len(bot_paths[ind]",len(bot_paths[ind]))
							#for i,b in enumerate(s.swarm):
							#current_position=[b.x,b.y]
							goal=bot_paths[goal_table[i]][count[i]]
							print("goal",goal)
							#Base control
							#cmd = base_control.base_control(i,b,goal)
							#cmd+= base_control.obstacle_avoidance(i,b,goal)
							
							#Behaviour
							#cmd+=line(b)
							#cmd =cvg.goal_area_cvg(i,b,goal,use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[30.0,30.0])
							cmd =cvg.goal_area_cvg(i,b,goal)
							#Execute
							cmd.exec(b)
							#print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
							dx=abs(goal[0]-current_position[0])
							dy=abs(goal[1]-current_position[1])						
							#print("dx,dy",dx,dy)
							if(dx<=10 and dy<=10):
								bot_array_return[i]=1
								#print("bot_array",bot_array)
	
							count_1=0
							print("count_1",count_1)
							for j in range(len(vehicles)):
								#print("************")
								if(bot_array_return[j]==1):
									count_1+=1
								if(count_1==len(vehicles)):
									count_1=0
									bot_array_return=[0]*len(vehicles)
									all_bot_reach_flag_1=True

							#print("bot_array",bot_array)
							if (all_bot_reach_flag_1==True):								
								#print("Goal_reached!!!!!!!!!!!")	
								with open(r'/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv', 'a') as csvfile:
									fieldnames = ['waypoint']
									writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
									writer.writerow({'waypoint':multiple_goals.index(goal)})	
								new_length_arr[i]=new_length_arr[i]-1
								print("new_length_arr[i],length_arr[i]-1",new_length_arr[i],length_arr[i]-1)
								#count[i]+=1
								for k in range(len(vehicles)):
									count[k]+=1
								all_bot_reach_flag_1=False
								bot_array_return=[0]*len(vehicles)
								print("goal,bot_paths[ind][-1]",goal,bot_paths[ind][-1])
								if (goal==bot_paths[ind][-1]):
									print("Break")
									home_flag=True						
									break		
							'''			
							if (new_length_arr[i]==-1):
								print("Break")
								cmd = base_control.base_control(i,b,goal)
								cmd+= base_control.obstacle_avoidance(i,b,goal)
								#Behaviour
								cmd+=line(b)
								#Execute
								cmd.exec(b)
								home_flag=True
								break	
							'''
							current_position = [b.x*2,b.y*2]
							#print("Drone 1 position",b.x,b.y)
							lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
							#print ("Drone 1 lat,lon", lat,lon)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)
							if i==0:
							    robot1_x,robot1_y={b.x},{b.y}
							elif i==1:
							    robot2_x,robot2_y={b.x},{b.y}
							elif i==2:
							    robot3_x,robot3_y={b.x},{b.y}
							elif i==3:
							    robot4_x,robot4_y={b.x},{b.y}
							elif i==4:
							    robot5_x,robot5_y={b.x},{b.y}
							elif i==5:
							    robot6_x,robot6_y={b.x},{b.y}
							elif i==6:
							    robot7_x,robot7_y={b.x},{b.y}
							elif i==7:
							    robot8_x,robot8_y={b.x},{b.y}
							elif i==8:
							    robot9_x,robot9_y={b.x},{b.y}
							elif i==9:
							    robot10_x,robot10_y={b.x},{b.y}
							elif i==10:
							    robot11_x,robot11_y={b.x},{b.y}
							elif i==11:
							    robot12_x,robot12_y={b.x},{b.y}
							elif i==12:
							    robot13_x,robot13_y={b.x},{b.y}
							elif i==13:
							    robot14_x,robot14_y={b.x},{b.y}
							elif i==14:
							    robot15_x,robot15_y={b.x},{b.y}
							elif i==15:
							    robot16_x,robot16_y={b.x},{b.y}
							else:
							    print('invalid')
							msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
						       str(robot2_x) + ',' + str(robot2_y) + ',' +
						       str(robot3_x) + ',' + str(robot3_y) + ',' +
						       str(robot4_x) + ',' + str(robot4_y) + ',' +
						       str(robot5_x) + ',' + str(robot5_y) + ',' +
						       str(robot6_x) + ',' + str(robot6_y) + ',' +
						       str(robot7_x) + ',' + str(robot7_y) + ',' +
						       str(robot8_x) + ',' + str(robot8_y) + ',' +
						       str(robot9_x) + ',' + str(robot9_y) + ',' +
						       str(robot10_x) + ',' + str(robot10_y))
										       
									
						sent = graph_socket.sendto(str(msg).encode(), graph_server_address)													
																															
		
			
					#gui.update()
					
					if(index==b"uav_pos"):
						home_pos=[]
						print("home_pos",home_pos)
						with open(csv_file_path, mode='r') as csv_file:
						    csv_reader = csv.DictReader(csv_file)
						    for row in csv_reader:
						    	x = float(row['X'])
						    	y = float(row['Y'])
						    	home_pos.append((x, y))
						print("home_pos",home_pos)
						home_flag=True
						#break
					if(index==b"home") or (home_flag):
						sent = graph_socket.sendto(str("home").encode(), graph_server_address)				
						bot_array_home=[0]*len(vehicles)
						all_bot_reach_flag_home=False
						print("Home.......")
							
						while 1:
							if(home_flag1):
								exit()
							for i,b in enumerate(s.swarm):
								'''
								if(pop_flag):	
									if(i>=pop_bot_index):
										i+=1
								'''
								current_position = [b.x,b.y]
								#cmd =cvg.goal_area_cvg(i,b,home_pos[i],use_base_control=True, exp_weight_params=[15.0,15.5],disp_weight_params=[30.0,30.0])
								cmd =cvg.goal_area_cvg(i,b,home_pos[i])
								goal=home_pos[i]
								#Execute
								cmd.exec(b)
								
								dx=abs(goal[0]-current_position[0])
								dy=abs(goal[1]-current_position[1])						
								print("dx,dy",dx,dy)
								
								if(dx<=0.1 and dy<=0.1):
									bot_array_home[i]=1
									#print("bot_array",bot_array)	
								count_home=0
								print("count_home",count_home)
								for j in range(len(vehicles)):
									#print("************")
									if(bot_array_home[j]==1):
										count_home+=1
									if(count_home==len(vehicles)):
										count_home=0
										bot_array_home=[0]*len(vehicles)
										all_bot_reach_flag_home=True

								#print("bot_array",bot_array)
								if (all_bot_reach_flag_home==True):
									with open(r'/home/dhaksha/Downloads/dce_swarm1/dce_swarm_nav/swarm_tasks-main/swarm_tasks/Examples/basic_tasks/pos.csv', 'a') as csvfile:
										fieldnames = ['waypoint']
										writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
										writer.writerow({'waypoint':-1})	
									home_flag1=True
										
															
								current_position = [b.x*2,b.y*2]
								#print("Drone 1 position",b.x,b.y)
								lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
								#print ("Drone 1 lat,lon", lat,lon)
								if same_alt_flag:
									point1 = LocationGlobalRelative(lat,lon,same_height)
								else:
									point1 = LocationGlobalRelative(lat,lon,different_height[i])
								vehicles[i].simple_goto(point1)
								if i==0:
								    robot1_x,robot1_y={b.x},{b.y}
								elif i==1:
								    robot2_x,robot2_y={b.x},{b.y}
								elif i==2:
								    robot3_x,robot3_y={b.x},{b.y}
								elif i==3:
								    robot4_x,robot4_y={b.x},{b.y}
								elif i==4:
								    robot5_x,robot5_y={b.x},{b.y}
								elif i==5:
								    robot6_x,robot6_y={b.x},{b.y}
								elif i==6:
								    robot7_x,robot7_y={b.x},{b.y}
								elif i==7:
								    robot8_x,robot8_y={b.x},{b.y}
								elif i==8:
								    robot9_x,robot9_y={b.x},{b.y}
								elif i==9:
								    robot10_x,robot10_y={b.x},{b.y}
								elif i==10:
								    robot11_x,robot11_y={b.x},{b.y}
								elif i==11:
								    robot12_x,robot12_y={b.x},{b.y}
								elif i==12:
								    robot13_x,robot13_y={b.x},{b.y}
								elif i==13:
								    robot14_x,robot14_y={b.x},{b.y}
								elif i==14:
								    robot15_x,robot15_y={b.x},{b.y}
								elif i==15:
								    robot16_x,robot16_y={b.x},{b.y}
								else:
								    print('invalid')
								msg = (str(robot1_x) + ',' + str(robot1_y) + ',' +
							       str(robot2_x) + ',' + str(robot2_y) + ',' +
							       str(robot3_x) + ',' + str(robot3_y) + ',' +
							       str(robot4_x) + ',' + str(robot4_y) + ',' +
							       str(robot5_x) + ',' + str(robot5_y) + ',' +
							       str(robot6_x) + ',' + str(robot6_y) + ',' +
							       str(robot7_x) + ',' + str(robot7_y) + ',' +
							       str(robot8_x) + ',' + str(robot8_y) + ',' +
							       str(robot9_x) + ',' + str(robot9_y) + ',' +
							       str(robot10_x) + ',' + str(robot10_y))
											       
										
							sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
							#gui.update()	
							if(index==b"stop"):
								home_flag1=True
								
					if(index==b"stop"):
						sent = graph_socket.sendto(str("stop").encode(), graph_server_address)				
						break
					
						


	except:
		#print("Exception!!!!!!!!")	
		a=0				
