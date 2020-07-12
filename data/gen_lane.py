from __future__ import absolute_import
from __future__ import print_function
import os
import sys
k = 0

length = 227.2
attach = 186.4
def gen_node(x, y):
    with open("data/lane.nod.xml", "w") as nets:
        print('<nodes>', file=nets)
        for i in range(x + 2): # y-axis
            for j in range(y + 2): # x-axis 1
                if i == 0 or i == x+1:
                    if j != 0 and j != y+1:
                        if i == 0:
                            print('<node id="%s" x="%f" y="%f" type="priority" />' %('00'+str(i)+'_00'+str(j),i*length-attach,j*length),file=nets)       
                        if i == x+1:
                            print('<node id="%s" x="%f" y="%f" type="priority" />' %('00'+str(i)+'_00'+str(j),i*length+attach,j*length),file=nets)       
                elif j == 0:
                    print('<node id="%s" x="%f" y="%f" type="priority" />' %('00'+str(i)+'_00'+str(j),i*length,j*length-attach),file=nets)       
                elif j == y+1:
                    print('<node id="%s" x="%f" y="%f" type="priority" />' %('00'+str(i)+'_00'+str(j),i*length,j*length+attach),file=nets)    
                else:   
                    print('<node id="%s" x="%f" y="%f" type="priority" />' %('00'+str(i)+'_00'+str(j),i*length,j*length),file=nets)        

               
        print('</nodes>', file=nets)


def gen_edge(x, y):
    with open("data/lane.edg.xml", "w") as edges:
        print('<edges>', file=edges)

        for j in range(1, y + 1):
            for i in range(1, x+1):
                # Left
                print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(i)+'_00'+str(j)+'_1','00'+str(i-1)+'_00'+str(j),'00'+str(i)+'_00'+str(j)),file=edges)
                # Up
                print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(i)+'_00'+str(j)+'_2','00'+str(i)+'_00'+str(j+1),'00'+str(i)+'_00'+str(j)),file=edges)
                # Right
                print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(i)+'_00'+str(j)+'_3','00'+str(i+1)+'_00'+str(j),'00'+str(i)+'_00'+str(j)),file=edges)
                # Down
                print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(i)+'_00'+str(j)+'_4','00'+str(i)+'_00'+str(j-1),'00'+str(i)+'_00'+str(j)),file=edges)
    
        # x-Axis going out
        for i in range(1, x+1):
            print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(i)+'_00'+str(0)+'_2','00'+str(i)+'_00'+str(1),'00'+str(i)+'_00'+str(0)),file=edges)
            print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(i)+'_00'+str(y+1)+'_4','00'+str(i)+'_00'+str(y),'00'+str(i)+'_00'+str(y+1)),file=edges)
        for j in range(1, y+1):
            print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(0)+'_00'+str(j)+'_3','00'+str(1)+'_00'+str(j),'00'+str(0)+'_00'+str(j)),file=edges)
            print('<edge id="%s" from="%s" to="%s" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %('00'+str(x+1)+'_00'+str(j)+'_1','00'+str(x)+'_00'+str(j),'00'+str(x+1)+'_00'+str(j)),file=edges)
            
        print('</edges>', file=edges)

def gen_connection(x, y):
    with open("data/lane.con.xml", "w") as cons:
        print('<connections>', file=cons)

        for j in range(1, y + 1):
            for i in range(1, x+1):
                for k in range(3):
                # 1-left
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_1','00'+str(i)+'_00'+str(j+1)+'_4',k,k),file=cons)
                # 1-straight
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_1','00'+str(i+1)+'_00'+str(j)+'_1',k,k),file=cons)
                # 1-right
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_1','00'+str(i)+'_00'+str(j-1)+'_2',k,k),file=cons)
                
                # 2-left
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_2','00'+str(i-1)+'_00'+str(j)+'_3',k,k),file=cons)
                # 2-straight
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_2','00'+str(i)+'_00'+str(j-1)+'_2',k,k),file=cons)
                # 2-right
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_2','00'+str(i+1)+'_00'+str(j)+'_1',k,k),file=cons)
                
                # 3-left
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_3','00'+str(i)+'_00'+str(j-1)+'_2',k,k),file=cons)
                # 3-straight
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_3','00'+str(i-1)+'_00'+str(j)+'_3',k,k),file=cons)
                # 3-right
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_3','00'+str(i)+'_00'+str(j+1)+'_4',k,k),file=cons)
                
                # 4-left
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_4','00'+str(i-1)+'_00'+str(j)+'_3',k,k),file=cons)
                # 4-straight
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_4','00'+str(i)+'_00'+str(j+1)+'_4',k,k),file=cons)
                # 4-right
                    print('<connection from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(j)+'_4','00'+str(i+1)+'_00'+str(j)+'_1',k,k),file=cons)            
        for i in range(1, x+1):
            print('<delete from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(0)+'_2','00'+str(i)+'_00'+str(1)+'_4',2,2),file=cons)            
            print('<delete from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(i)+'_00'+str(x+1)+'_4','00'+str(i)+'_00'+str(x)+'_2',2,2),file=cons)            
        for j in range(1, y+1):
            print('<delete from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(0)+'_00'+str(j)+'_3','00'+str(1)+'_00'+str(j)+'_1',2,2),file=cons)            
            print('<delete from="%s" to="%s" fromLane="%d" toLane="%d"/>' %('00'+str(x+1)+'_00'+str(j)+'_1','00'+str(x)+'_00'+str(j)+'_3',2,2),file=cons)            

        print('</connections>', file=cons)

def gen_tlc(x, y):
    with open("data/lane.tlc.xml", "w") as tlcs:
        for i in range(x + 2): # y-axis
            for j in range(y + 2): # x-axis 1
                if i == 0 or i == x+1:
                    if j != 0 and j != y+1:
                        print('<tlLogic id="%s" type="static">' %('00'+str(i)+'_00'+str(j)), file=tlcs)
                        print('<phase duration="31" state="GGggggggGGgggggg"/>' ,file=tlcs)       
                        print('</tlLogic>', file=tlcs)
os.system('cd /usr/local/Cellar/sumo/1.4.0/share/sumo/tools')
def gen_net(i):
    gen_node(i,i)
    gen_edge(i,i)
    gen_connection(i,i)
for i in range(50,51):
    gen_net(i)
    node = '/Users/jacksonwang/Downloads/V2X/Roadrunner_v2_traffic_light/data/lane.nod.xml'
    edge = '/Users/jacksonwang/Downloads/V2X/Roadrunner_v2_traffic_light/data/lane.edg.xml'
    con = '/Users/jacksonwang/Downloads/V2X/Roadrunner_v2_traffic_light/data/lane.con.xml'
    out = '/Users/jacksonwang/Downloads/V2X/Roadrunner_v2_traffic_light/data/lane{}by{}.net.xml'.format(i,i)

    os.system("netconvert --node-files={} --edge-files={} --connection-files={} --output-file={}".format(node,edge,con,out))

