"""
https://github.com/yashveerjain/djikstra
"""

import cv2
import numpy as np
import os
import queue
import sys
import argparse
import math

class Djikstra():
    def __init__(self, start_pos = [0,0], goal_pos = [600,250], map_size=(250,600)) -> None:
        self.map = np.zeros(map_size)
        # map_size (H,W) = (Y axis,X axis)

        # start_pos = (x,y) user input
        # because index start from 0, so handling edge cases when the pos is same as the height or width of the map
        if start_pos[1] >= map_size[0]:
            start_pos[1] = map_size[0]-1
        if start_pos[0] >= map_size[1]:
            start_pos[0] = map_size[1]-1
        # edit to make it according to array index which is top left as origin to bottom left as origin
        self.start_pos = (start_pos[0],map_size[0]-1-start_pos[1]) 

        # goal_pos = (x,y) user input
        # because index start from 0, so handling edge cases when the pos is same as the height or width of the map
        if goal_pos[1] >= map_size[0]:
            goal_pos[1] = map_size[0]-1
        if goal_pos[0] >= map_size[1]:
            goal_pos[0] = map_size[1]-1
        # edit to make it according to array index which is top left as origin to bottom left as origin
        self.goal_pos = (goal_pos[0],map_size[0]-1-goal_pos[1]) 

        ## adding the obstacle to blank map
        self.create_map()

        if self.map[self.goal_pos[1], self.goal_pos[0],0] > 0 or self.map[self.start_pos[1],self.start_pos[0],0] > 0:
            print("please enter node again, as it is coinciding with the obstacles")
            sys.exit(1)

        cv2.imwrite("map.png", self.map)
        self.map = cv2.circle(self.map,self.start_pos,3,(0,255,0),2)
        self.map = cv2.circle(self.map,self.goal_pos,3,(0,0,255),2)

        cv2.imwrite("map_st_gl.png", self.map)

        # Djikstra variables initialization
        self.node_state = queue.PriorityQueue()
        self.parent_child_index = {0:0}
        self.visited_nodes = {0: self.start_pos}
        self.node_state.put((0,0,self.start_pos))
        self.goal_node_idx = None
        self.total_cost = 0 # cost to reach to goal from start pos

    def get_line_equation(self, coord1, coord2, map_coord, clearance, inverse = False):
        # reference : https://byjus.com/jee/distance-between-2-parallel-lines/
        m = (coord2[1]-coord1[1])/(coord2[0]-coord1[0])
        c = coord1[1] - m * coord1[0]

        if inverse:
            # new equation of line, distance = |c1 - c2|/sqrt(1+m^2); distance between parallel lines 
            c2 = c - clearance * math.sqrt((1+m**2)) 
        else:
            # new equation of line, distance = |c2 - c1|/sqrt(1+m^2); distance between parallel lines 
            c2 = clearance * math.sqrt((1+m**2)) + c

        return map_coord[1] - m*map_coord[0] - c2

    def create_map(self):
        self.map = np.zeros((250,600,3))
        clearance = 5

        ## draw rectangles
        self.map = cv2.rectangle(self.map, pt1=(100,0), pt2=(150,100), color=(0,255,0), thickness=-1)

        ## draw rectangles
        self.map = cv2.rectangle(self.map, pt1=(100,150), pt2=(150,250), color=(0,255,0), thickness=-1)

        ## draw hexagone
        pts = np.array([
            [150+150-65, 87],
            [150+150-65, 162],
            [150+150,125+75],
            [150+150+65, 162],
            [150+150+65, 87],
            [150+150,125-75]],dtype=np.int32)
        # self.map = cv2.polylines(self.map,[pts],isClosed=True, thickness=1, color=(150,0,0))
        self.map = cv2.fillPoly(self.map,[pts],color=(0,255,0))

        ## draw triangle
        pts = np.array([
            [300+160, 25],
            [300+160, 225],
            [300+160+50,125]],dtype=np.int32)
        # self.map = cv2.polylines(self.map,[pts],isClosed=True, thickness=1, color=(150,0,0))
        self.map = cv2.fillPoly(self.map,[pts], color=(0,255,0))


        ## Adding clearance
        ## Adding clearance
        for y in range(self.map.shape[0]):
            for x in range(self.map.shape[1]):
                # creating boundary for upper rectangle
                if (100-clearance < x and 150+clearance > x and 100+clearance > y):
                    self.map[y,x,0] = 255 # blue color for clearance boundary
                
                #  creating boundary for lower rectangle
                elif (100-clearance < x and 150+clearance > x and 250-100-clearance < y):
                    self.map[y,x,0] = 255 # blue color for clearance boundary
                
                # creating boundary for hexagon
                elif (
                    (235-clearance < x and 365+clearance > x) # xaxis 
                    and (self.get_line_equation((235,87),(300,50),(x,y),clearance,inverse=True)>0) # y must be less in this eq
                    and (self.get_line_equation((300,50),(365,87),(x,y),clearance,inverse=True)>0) # y must be less in this eq
                    and (self.get_line_equation((235,162),(300,200),(x,y),clearance)<0) # y must be large wrt x in this eq
                    and (self.get_line_equation((300,200),(365,162),(x,y),clearance)<0) # y must be large wrt x in this eq
                    ):
                    self.map[y,x,0] = 255 # blue color for clearance boundary
                elif (
                    (460-clearance < x and 510+clearance > x) # xaxis 
                    and (self.get_line_equation((460,25),(510,125),(x,y),clearance-2,inverse=True)>0) # y must be less in this eq
                    and (self.get_line_equation((460,225),(510,125),(x,y),clearance-2)<0) # y must be less in this eq
                    ):
                    self.map[y,x,0] = 255 # blue color for clearance boundary
                elif x<=clearance or 600-clearance<=x or clearance>=y or 250 - clearance<=y:
                    self.map[y,x,0]=255  
        self.map = self.map.astype(np.uint8)
        
        # plt.imshow(self.map,cmap="gray")
    
    def moveRight(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1
        new_pos = (x+1,y)
        if new_pos[0] >= W: 
            return False, curr_pos
        if map[new_pos[1], new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost, new_pos)

    def moveLeft(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1
        new_pos = (x-1,y)
        if new_pos[0] < 0: 
            return False, curr_pos
        if map[new_pos[1], new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost, new_pos)

    def moveUp(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1
        new_pos = (x,y-1)
        if new_pos[1] <0:
            return False, curr_pos
        if map[new_pos[1],new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost,new_pos)

    def moveDown(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1
        new_pos = (x,y+1)
        if new_pos[1] >= H:
            return False, curr_pos
        if map[new_pos[1],new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost,new_pos)


    def moveUpLeft(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1.4
        new_pos = (x-1,y-1)
        if new_pos[1] < 0 or new_pos[0] < 0:
            return False, curr_pos
        if map[new_pos[1],new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost,new_pos)


    def moveDownLeft(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1.4
        new_pos = (x-1,y+1)
        if new_pos[1] >= H or new_pos[0] < 0:
            return False, curr_pos
        if map[new_pos[1],new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost,new_pos)

    def moveUpRight(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1.4
        new_pos = (x+1,y-1)
        if new_pos[0] >= W or new_pos[1] < 0:
            return False, curr_pos
        if map[new_pos[1],new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost,new_pos)

    def moveDownRight(self, curr_pos: tuple, map: np.ndarray):
        x,y  = curr_pos
        H,W,_ = map.shape
        cost = 1.4
        new_pos = (x+1,y+1)
        if new_pos[0] >= W or new_pos[1] >= H:
            return False, curr_pos
        if map[new_pos[1],new_pos[0],0]>0:
            return False, curr_pos
        return True, (cost,new_pos)

    def isGoalNode(self, curr_node, goal_node):
        return curr_node==goal_node
    
    def run(self):
        # this map will keep track of visited nodes too, by assigning value 1 to visited node.
        temp_map = self.map.copy()
        temp_map[self.start_pos[1], self.start_pos[0],0] = 1
        node_counter = 0
        print("Started Searching-----------")
        while not self.node_state.empty():
            prev_cost, parent_idx, prev_pos = self.node_state.get()
            # print(prev_cost, parent_idx, prev_pos)
            for func in [self.moveDown, self.moveDownLeft, self.moveDownRight, self.moveLeft, self.moveRight, self.moveUp, self.moveUpLeft, self.moveUpRight]:
                st, new_node_data = func(prev_pos, temp_map)
                # print(st)
                if not st:
                    continue
                
                # get new node cost and pos
                new_cost, new_pos = new_node_data
                
                # increase node index and add the new node cost and state to queue
                node_counter += 1 
                new_cost += prev_cost
                self.parent_child_index[node_counter] = parent_idx
                self.node_state.put((new_cost, node_counter, new_pos))

                # Keep track of visited nodes by marking them 1, which can check in 8 functions above
                temp_map[new_pos[1],new_pos[0],0] = 1
                self.visited_nodes[node_counter] = new_pos

                if self.isGoalNode(new_pos, self.goal_pos):
                    total_cost = new_cost
                    self.goal_node_idx = node_counter
                    print("Got the goal Node!")
                    break

            if self.goal_node_idx:
                break
        
        ## Start Backtracking
        # start2goal_poses = self.backtrack()

    def backtrack(self):
        print("Backtracking -------")
        s2g_pos = []
        idx = self.goal_node_idx
        indices = [idx]

        while idx!=0:
            idx = self.parent_child_index[idx]
            indices.append(idx)
        s2g_idx = sorted(indices)

        print("Start -> goal nodes ---")
        for idx in s2g_idx:
            pos =  self.visited_nodes[idx]
            print(pos)
            s2g_pos.append(pos)
        return s2g_pos
    
    def plot_path(self):
        new_canvas = self.map.copy().astype(np.uint8)
        size = (new_canvas.shape[1],new_canvas.shape[0])
        # Below VideoWriter object will create
        # a frame of above defined The output 
        # is stored in 'filename.avi' file.
        result = cv2.VideoWriter('optimal_path.avi', 
                                cv2.VideoWriter_fourcc(*'MJPG'),
                                20, size)
        s2g_poses = self.backtrack()
        for pos in s2g_poses:
            # print(pos)
            new_canvas[pos[1],pos[0]] = 255

            result.write(new_canvas)
            
            # Display the frame
            # saved in the file
            cv2.imshow('Frame', new_canvas)

            # Press S on keyboard 
            # to stop the process
            if cv2.waitKey(1) & 0xFF == ord('s'):
                break
        # plt.show()
        result.release()
            
        # Closes all the frames
        cv2.destroyAllWindows()

    def plot_search(self):
        new_canvas = self.map.copy()
        
        size = (new_canvas.shape[1],new_canvas.shape[0])
        # Below VideoWriter object will create
        # a frame of above defined The output 
        # is stored in 'filename.avi' file.
        result = cv2.VideoWriter('node_exploration.avi', 
                                cv2.VideoWriter_fourcc(*'MJPG'),
                                180*8, size)
        
        for pos in self.visited_nodes.values():
            # print(pos)
            new_canvas[pos[1],pos[0]] = 255

            # saved in the file
            cv2.imshow('Frame', new_canvas)

            result.write(new_canvas)

            # Press S on keyboard 
            # to stop the process
            if cv2.waitKey(1) & 0xFF == ord('s'):
                break
            
        
        result.release()
        # Closes all the frames
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--start_pos', nargs='+', type=int, required=True)
    parser.add_argument('--goal_pos', nargs='+', type=int, required=True)
    args = parser.parse_args()
    start_pos = list(args.start_pos)
    goal_pos = list(args.goal_pos)

    print("Start Pos recieved : ", start_pos)
    print("Goal Pos recieved : ", goal_pos)

    dji = Djikstra(start_pos=start_pos, goal_pos=goal_pos)
    dji.run()

    # uncomment this line if want to see whole djikstra search mapping
    # dji.plot_search()
    
    # this will backtrack the optimal path
    dji.plot_path()