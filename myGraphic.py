from Tkinter import *
import Tkinter as tk
import threading
import time

global gui

class Gui:
    def __init__(self):
        self.window = Tk()
        self.window.title("Roadrunner State")

        self.inter_height = 400
        self.inter_width = 400
        self.window_height = 2*self.inter_height+100
        self.window_width = self.inter_width

        self.canvas = Canvas(self.window, width=self.window_width, height=self.window_height)
        self.canvas.pack()
        self.drawIntersection(self.inter_height, self.inter_width, 50, "Predicted Status for Advising")
        self.drawInsideIntersection(self.inter_height, self.inter_width, 50)
        self.drawIntersection(self.inter_height, self.inter_width, 100+self.inter_height, "Current Status in Intersection")


        #==================================
        self.time_matrix = None

        self.advised_time_matrix = None
        self.turning_signs_list = []


        lane_space = self.inter_width/(3*2+2)
        self.matrix_string = dict()
        for xdx in range(1, 13):
            for ydx in range(1, 13):
                self.matrix_string[(xdx,ydx)] = StringVar()
                label = Label(self.canvas, textvariable=self.matrix_string[(xdx,ydx)], text="0", width=len("0"))
                self.canvas.create_window(lane_space*1.25+lane_space*(xdx-1)/2, 37+lane_space*(14+1-ydx)/2, window=label)

        self.stop_flag = False
        #self.graph_thread = threading.Thread(target = updateGraph, args = (self,))
        #self.graph_thread.start()
        self.window.update_idletasks()
        self.window.update()


    def stopShowing(self):
        self.stop_flag = True
        self.window.destroy()

    def setTimeMatrix(self, matrix):
        self.time_matrix = matrix
    def setAdviseMatrix(self, matrix):
        self.advised_time_matrix = matrix

    def updateInfo(self):
        # Update time_matrix
        if self.time_matrix != None:
            for xdx in range(1, 13):
                for ydx in range(1, 13):
                    self.matrix_string[(xdx,ydx)].set(str(int(self.time_matrix[xdx][ydx])))

        # Redraw all the advised lines
        if self.advised_time_matrix != None:
            lane_space = self.inter_width/(3*2+2)
            for lines in self.turning_signs_list:
                self.canvas.delete(lines)
            self.turning_signs_list = []
            for key, lane in self.advised_time_matrix.items():
                direction = key[0]
                turning = key[1]
                sublane_id = lane % 3
                if direction == 0:
                    start_pt_x = 4.5*lane_space + sublane_id*lane_space
                    start_pt_y = 50+7.8*lane_space
                    end_pt_x = None
                    end_pt_y = None

                    if turning == 'S':
                        end_pt_x = start_pt_x
                        end_pt_y = start_pt_y-0.5*lane_space
                    elif turning == 'L':
                        end_pt_x = start_pt_x - 0.5*lane_space
                        end_pt_y = start_pt_y
                    elif turning == 'R':
                        end_pt_x = start_pt_x + 0.5*lane_space
                        end_pt_y = start_pt_y

                    temp_line = self.canvas.create_line(start_pt_x, start_pt_y, end_pt_x, end_pt_y, fill="black", width=2, arrow=tk.LAST)
                    self.turning_signs_list.append(temp_line)

                elif direction == 2:
                    start_pt_x = 3.5*lane_space - sublane_id*lane_space
                    start_pt_y = 50+0.2*lane_space
                    end_pt_x = None
                    end_pt_y = None

                    if turning == 'S':
                        end_pt_x = start_pt_x
                        end_pt_y = start_pt_y+0.5*lane_space
                    elif turning == 'L':
                        end_pt_x = start_pt_x + 0.5*lane_space
                        end_pt_y = start_pt_y
                    elif turning == 'R':
                        end_pt_x = start_pt_x - 0.5*lane_space
                        end_pt_y = start_pt_y
                    temp_line = self.canvas.create_line(start_pt_x, start_pt_y, end_pt_x, end_pt_y, fill="black", width=2, arrow=tk.LAST)
                    self.turning_signs_list.append(temp_line)

                elif direction == 1:
                    start_pt_x = 7.8*lane_space
                    start_pt_y = 50+3.5*lane_space - sublane_id*lane_space
                    end_pt_x = None
                    end_pt_y = None

                    if turning == 'S':
                        end_pt_x = start_pt_x - 0.5*lane_space
                        end_pt_y = start_pt_y
                    elif turning == 'L':
                        end_pt_x = start_pt_x
                        end_pt_y = start_pt_y + 0.5*lane_space
                    elif turning == 'R':
                        end_pt_x = start_pt_x
                        end_pt_y = start_pt_y - 0.5*lane_space
                    temp_line = self.canvas.create_line(start_pt_x, start_pt_y, end_pt_x, end_pt_y, fill="black", width=2, arrow=tk.LAST)
                    self.turning_signs_list.append(temp_line)

                elif direction == 3:
                    start_pt_x = 0.2*lane_space
                    start_pt_y = 50+4.5*lane_space + sublane_id*lane_space
                    end_pt_x = None
                    end_pt_y = None

                    if turning == 'S':
                        end_pt_x = start_pt_x + 0.5*lane_space
                        end_pt_y = start_pt_y
                    elif turning == 'L':
                        end_pt_x = start_pt_x
                        end_pt_y = start_pt_y - 0.5*lane_space
                    elif turning == 'R':
                        end_pt_x = start_pt_x
                        end_pt_y = start_pt_y + 0.5*lane_space
                    temp_line = self.canvas.create_line(start_pt_x, start_pt_y, end_pt_x, end_pt_y, fill="black", width=2, arrow=tk.LAST)
                    self.turning_signs_list.append(temp_line)






    def drawIntersection(self, height, width, topBias, title):
        lane_space = width/(3*2+2)

        self.canvas.create_text(width/2,topBias-25,fill="black",font="Arial 20 bold", text=title)

        self.canvas.create_line(0, topBias+lane_space, lane_space, topBias+lane_space, fill="black", width=2)
        self.canvas.create_line(lane_space, topBias, lane_space, topBias+lane_space, fill="black", width=2)
        self.canvas.create_line(lane_space*7, topBias+lane_space, lane_space*8, topBias+lane_space, fill="black", width=2)
        self.canvas.create_line(lane_space, topBias+lane_space*7, lane_space, topBias+lane_space*8, fill="black", width=2)
        for idx in range(2, 4):
            self.canvas.create_line(0, topBias+lane_space*idx, lane_space, topBias+lane_space*idx, fill="black", width=1,dash=(4, 4))
            self.canvas.create_line(lane_space*idx, topBias, lane_space*idx, topBias+lane_space, fill="black", width=1,dash=(4, 4))
            self.canvas.create_line(lane_space*7, topBias+lane_space*idx, lane_space*8, topBias+lane_space*idx, fill="black", width=1,dash=(4, 4))
            self.canvas.create_line(lane_space*idx, topBias+lane_space*7, lane_space*idx, topBias+lane_space*8, fill="black", width=1,dash=(4, 4))

        self.canvas.create_line(0, topBias+lane_space*4, lane_space, topBias+lane_space*4, fill="black", width=2)
        self.canvas.create_line(lane_space*4, topBias, lane_space*4, topBias+lane_space, fill="black", width=2)
        self.canvas.create_line(lane_space*7, topBias+lane_space*4, lane_space*8, topBias+lane_space*4, fill="black", width=2)
        self.canvas.create_line(lane_space*4, topBias+lane_space*7, lane_space*4, topBias+lane_space*8, fill="black", width=2)
        for idx in range(5, 7):
            self.canvas.create_line(0, topBias+lane_space*idx, lane_space, topBias+lane_space*idx, fill="black", width=1,dash=(4, 4))
            self.canvas.create_line(lane_space*idx, topBias, lane_space*idx, topBias+lane_space, fill="black", width=1,dash=(4, 4))
            self.canvas.create_line(lane_space*7, topBias+lane_space*idx, lane_space*8, topBias+lane_space*idx, fill="black", width=1,dash=(4, 4))
            self.canvas.create_line(lane_space*idx, topBias+lane_space*7, lane_space*idx, topBias+lane_space*8, fill="black", width=1,dash=(4, 4))

        self.canvas.create_line(0, topBias+lane_space*7, lane_space, topBias+lane_space*7, fill="black", width=2)
        self.canvas.create_line(lane_space*7, topBias, lane_space*7, topBias+lane_space, fill="black", width=2)
        self.canvas.create_line(lane_space*7, topBias+lane_space*7, lane_space*8, topBias+lane_space*7, fill="black", width=2)
        self.canvas.create_line(lane_space*7, topBias+lane_space*7, lane_space*7, topBias+lane_space*8, fill="black", width=2)

        self.canvas.create_line(0, topBias+lane_space*8, lane_space*8, topBias+lane_space*8, fill="gray", width=3)


    def drawInsideIntersection(self, height, width, topBias):
        lane_space = width/(3*2+2)

        for idx in range(2, 15):
            self.canvas.create_line(lane_space, topBias+lane_space/2*idx, lane_space*7, topBias+lane_space/2*idx, fill="black", width=1,dash=(4, 4))
            self.canvas.create_line(lane_space/2*idx, topBias+lane_space, lane_space/2*idx, topBias+lane_space*7, fill="black", width=1,dash=(4, 4))

    def updateGraph(self):
        self.updateInfo()

        self.window.update_idletasks()
        self.window.update()
