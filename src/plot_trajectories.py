from __future__ import print_function

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from math import sin,cos,pi


class Trajectory():
    def __init__(self):
        self.obj_init_pos = []
        self.obj_final_pos = []
        self.eef_pos_vector = []
        
    def get_obj_init_pos(self):
        return self.obj_init_pos
    def set_obj_init_pos(self, x, y, z):
        self.obj_init_pos = [x, y, z]
        
    def get_obj_final_pos(self):
        return self.obj_final_pos
    def set_obj_final_pos(self, x, y, z):
        self.obj_final_pos = [x, y, z]        
        
    def add_eef_pos(self, x, y, z):
        self.eef_pos_vector.append([x,y,z])
    def get_eef_pos_vector(self):
        return self.eef_pos_vector
    def print_me(self):
        print('Trajectory values: ')
        print(self.obj_init_pos)
        print(self.obj_final_pos)
        print(self.eef_pos_vector)


'''
a
'''
def get_trajs(filename):
    trajs_vector = []
    lines = open(filename, 'r').readlines()
    for line in lines:
        curr_traj = Trajectory()
        pos_rot_vector = line[:-2].split(',')
        curr_traj.set_obj_init_pos(pos_rot_vector[0],
                                   pos_rot_vector[1],
                                   pos_rot_vector[2])
        curr_traj.set_obj_final_pos(pos_rot_vector[6],
                                    pos_rot_vector[7],
                                    pos_rot_vector[8])
        wp_pos_rot_vector = pos_rot_vector[12:]
#        print('Length wp_pos_rot_vector', len(wp_pos_rot_vector))
#        print('Nb WPs', len(wp_pos_rot_vector)//6)
        pos = 0
        while pos < len(wp_pos_rot_vector):
            curr_traj.add_eef_pos(wp_pos_rot_vector[pos+0],
                                  wp_pos_rot_vector[pos+1],
                                  wp_pos_rot_vector[pos+2])
            pos += 6
        trajs_vector.append(curr_traj)
#        print()
#        curr_traj.print_me()
    return trajs_vector        

'''
a
'''
def gen_init_eef(radius, nb_init_pos, obj_pos):
    nb_init_pos += 2
    list_radians = [0]
    tmp = np.linspace(pi/(180.0/360), pi/180, nb_init_pos-1)
    tmp2 = tmp[:-1].tolist()
    tmp2.reverse()
    list_radians = list_radians + tmp2
    list_x_axis = []
    list_y_axis = []
    for a in list_radians[1:]:
        list_x_axis.append(obj_pos[0] + (cos(a))*radius)
        list_y_axis.append(obj_pos[1] + (sin(a))*radius)    
    return list_x_axis, list_y_axis, np.zeros(len(list_x_axis))

'''
a
'''
def plot_trajs(radius, 
               obj_pos,
               traj_vector):
    
    nb_traj = 0
    while nb_traj < len(traj_vector): 
#    while nb_traj == 0:     
        print('-------------> INIT POS', nb_traj//4)
        for nb_traj_tmp in range(0,4):
            print('TRAJECTORY', nb_traj + nb_traj_tmp)
            curr_traj = traj_vector[nb_traj + nb_traj_tmp]
            eef_pos_vector = curr_traj.get_eef_pos_vector()
            eef_pos_vector_x = [t[0] for t in eef_pos_vector]
            eef_pos_vector_y = [t[1] for t in eef_pos_vector]
                    
            # generate circle positions
            list_x_axis, list_y_axis, list_z_axis = \
                gen_init_eef(radius, nb_initial_pos, obj_pos)
            
            ''' Plot canvas ''' 
            fig = plt.figure()
            fig.set_size_inches(7,7)
            ax = fig.add_axes([0, 0, 1, 1])
            
            # set axis limits
            plt.xlim(obj_pos[0]-0.5, obj_pos[0]+0.5)
            plt.ylim(obj_pos[1]-0.5, obj_pos[1]+0.5)
         
            # plot obj_init_pos
            origin = Rectangle((obj_pos[0]-obj_side/2, 
                                obj_pos[1]-obj_side/2), 
                               obj_side, obj_side, fc="grey")
            ax.add_patch(origin)
             
            # plot the big circle points
            ax.plot(list_x_axis,list_y_axis,'o',c='r', markersize = 2)     
            
            # plot traj
            ax.plot(eef_pos_vector_x, eef_pos_vector_y, 
                    '-')
            ax.plot(eef_pos_vector_x, eef_pos_vector_y, 
                    '*', c='grey')
            
            plt.show()
            plt.close()            
        
        nb_traj += 4
        
    
'''
a
'''
if __name__ == '__main__':
    nb_initial_pos = 8
    obj_pos = [0.65, 0.1]
    obj_side = 0.06
    radius = 0.2
    
    traj_vector = get_trajs("../data.csv")
    print('Nb trajs :', len(traj_vector))
    plot_trajs(radius, 
               obj_pos,
               traj_vector)
    
    
    
