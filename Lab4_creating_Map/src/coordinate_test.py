#!/usr/bin/python3
from project_4.geometry import to_grid, to_world

correct = 0
total = 0

def grid_test(x, y, result, origin_x=0.0, origin_y=0.0, size_x=10, size_y=10, resolution=1.0):
    global correct, total
    
    test_coordinates = to_grid(x,y,origin_x,origin_y,size_x,size_y,resolution)
    if test_coordinates != result:
        print ('Failed grid test: ',x,y,origin_x,origin_y,size_x,size_y,resolution)
        print ('\tExpected:',result)
        print ('\tReceived:',test_coordinates)
    else:
        correct += 1
    total += 1        
    

def world_test(gx, gy, result, origin_x=0.0, origin_y=0.0, size_x=10, size_y=10, resolution=1.0):
    global correct, total
    
    test_coordinates = to_world(gx,gy,origin_x,origin_y,size_x,size_y,resolution)
    if test_coordinates != result:
        print ('Failed world test: ',gx,gy,origin_x,origin_y,size_x,size_y,resolution)
        print ('\tExpected:',result)
        print ('\tReceived:',test_coordinates)
    else:
        correct += 1
    total += 1        

    
grid_test(0,0,(0,0))
grid_test(5,5,(5,5))
grid_test(5,6,(5,6))
grid_test(5.5, 6.5,(5,6))
grid_test(-1,-1,None)
grid_test(11,11,None)
grid_test(9,10,None)
grid_test(10,9,None)


grid_test(0,0,(0,0), size_x=20, size_y=20, resolution=0.5)
grid_test(5,5,(10,10), size_x=20, size_y=20, resolution=0.5)
grid_test(5,6,(10,12), size_x=20, size_y=20, resolution=0.5)
grid_test(5.5, 6.5,(11,13), size_x=20, size_y=20, resolution=0.5)

grid_test(0,0,(5,5),origin_x=-5,origin_y=-5)
grid_test(-4.5,2.1,(0,7),origin_x=-5,origin_y=-5)

world_test(0, 0, (0.5, 0.5))
world_test(1, 0, (1.5, 0.5))
world_test(0, 1, (0.5, 1.5))
world_test(5, 5, (2.75, 2.75), resolution=0.5)
world_test(0, 0, (-4.5, -4.5), origin_x=-5, origin_y=-5)
world_test(9, 9, (4.5, 4.5), origin_x=-5, origin_y=-5)


print ("Passed %d/%d"%(correct, total))
