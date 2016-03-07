#define AStar_original
// Call this script before using any other AStar function. If you want
// to change the number or size of cells later, you have to first call
// AStar_free() and then initialize the system again.
// 
// Arguments:
// 0 - Number of cells horizontally
// 1 - Number of cells vertically
// 2 - Width of a cell
// 3 - Height of a cell
// 4 - X offset of the grid
// 5 - Y offset of the grid
// 6 - Allow diagonal movement
// 7 - Allow cutting through obstacles when moving diagonally
// 8 - Cost for moving straight (left, right, up, down) between two cells
// 9 - Cost for moving diagonally between two cells
// 10 - Coordinates in arguments - 0...expect real coordinates (0,20,40...)
//                                1...expect grid coordinates (0,1,2...)
// 11 - Path returning mode - 0...use and return an internal path
//                           1...always return a new path
// 
// For arguments 6 to 10: -2 means to use the default value.
// Arguments 6 to 10 can be changed later without reinitializing the engine,
//  using the AStar_setAll script.
// Arguments 8 and 9 can only take values larger than 0 (except -2).
//
// Returns: nothing

instance_create(0,0,objPathFinder);
with (objPathFinder) {
    def_allowDiag = 1;
    def_allowCut = 0;
    def_costNormal = 10;
    def_costDiag = 14;
    def_coordMode = 1;
    
    allowDiag = def_allowDiag;
    allowCut = def_allowCut;
    costNormal = def_costNormal;
    costDiag = def_costDiag;
    coordMode = def_coordMode;

    GRID_WIDTH = argument0;
    GRID_HEIGHT = argument1;
    CELL_WIDTH = argument2;
    CELL_HEIGHT = argument3;
    X_OFFSET = argument4;
    Y_OFFSET = argument5;
    if (argument6 != -2) allowDiag = argument6;
    if (argument7 != -2) allowCut = argument7;
    if (argument8 > 0) costNormal = argument8;
    if (argument9 > 0) costDiag = argument9;
    if (argument10 != -2) coordMode = argument10;
    pathMode = argument11;
    
    var a;
    for (a=0; a<GRID_WIDTH; a+=1) {
        for (b=0; b<GRID_HEIGHT; b+=1) {
            cells[a,b] = ds_map_create();
            ds_map_add(cells[a,b],"x",a);
            ds_map_add(cells[a,b],"y",b);
            ds_map_add(cells[a,b],"blocked",0);
            ds_map_add(cells[a,b],"dir0",0);
            ds_map_add(cells[a,b],"dir1",0);
            ds_map_add(cells[a,b],"dir2",0);
            ds_map_add(cells[a,b],"dir3",0);
            ds_map_add(cells[a,b],"dir4",0);
            ds_map_add(cells[a,b],"dir5",0);
            ds_map_add(cells[a,b],"dir6",0);
            ds_map_add(cells[a,b],"dir7",0);
            ds_map_add(cells[a,b],"pathParent",-1);
            ds_map_add(cells[a,b],"costFromStart",0);
            ds_map_add(cells[a,b],"estimatedCostToGoal",0);
        }
    }
    
    if (!pathMode) {
        thePath = path_add();
        path_set_closed(thePath,0);
    }
}


#define AStar_free_new
// Arguments: none
// Returns: nothing

with (objPathFinder) {
    var a;
    for (a=0; a<GRID_WIDTH; a+=1) {
        for (b=0; b<GRID_HEIGHT; b+=1) {
            ds_map_destroy(cells[a,b]);
        }
    }
    if (!pathMode)
        path_delete(thePath);
    instance_destroy();
}

#define AStar_setAll
// Arguments:
// 0 - Allow diagonal movement
// 1 - Allow cutting through obstacles when moving diagonally
// 2 - Cost for moving straight (left, right, up, down) between two cells
// 3 - Cost for moving diagonally between two cells
// 4 - Coordinates in arguments - 0...expect real coordinates (0,20,40...)
//                                1...expect grid coordinates (0,1,2...)
//
// For all arguments: -1 means no change, -2 restores the default value.
// Arguments 2 and 3 can only take values larger than 0 (except -1 or -2).
//
// Returns: nothing

if (argument0 == -2)
    objPathFinder.allowDiag = objPathFinder.def_allowDiag;
else if (argument0 != -1)
    objPathFinder.allowDiag = argument0;
if (argument1 == -2)
    objPathFinder.allowCut = objPathFinder.def_allowCut;
else if (argument1 != -1)
    objPathFinder.allowCut = argument1;
if (argument2 == -2)
    objPathFinder.costNormal = objPathFinder.def_costNormal;
else if (argument2 > 0)
    objPathFinder.costNormal = argument2;
if (argument3 == -2)
    objPathFinder.costDiag = objPathFinder.def_costDiag;
else if (argument3 > 0)
    objPathFinder.costDiag = argument3;
if (argument4 == -2)
    objPathFinder.coordMode = objPathFinder.def_coordMode;
else if (argument4 != -1)
    objPathFinder.coordMode = argument4;

#define AStar_setBlocked
// Arguments:
// 0 - x
// 1 - y
// 2 - 0...Free, 1...Blocked
//
// Returns: nothing

with (objPathFinder) {
    if (coordMode)
        ds_map_replace(cells[argument0,argument1],"blocked",argument2);
    else
        ds_map_replace(cells[floor((argument0-X_OFFSET)/CELL_WIDTH),
                       floor((argument1-Y_OFFSET)/CELL_HEIGHT)],
                       "blocked",argument2);
}

#define AStar_setDirBlocked
// Arguments:
// 0 - x
// 1 - y
// 2 - Direction - 0 to 7 (east to southeast)
// 3 - 0...Free, 1...Blocked
//
// Returns: nothing

with (objPathFinder) {
    if (coordMode)
        ds_map_replace(cells[argument0,argument1],
                       "dir"+string(argument2),argument3);
    else
        ds_map_replace(cells[floor((argument0-X_OFFSET)/CELL_WIDTH),
                       floor((argument1-Y_OFFSET)/CELL_HEIGHT)],
                       "dir"+string(argument2),argument3);
}

#define AStar_findPath_new
// Arguments:
// 0 - x1
// 1 - y1
// 2 - x2
// 3 - y2
// 4 - center (true/false)
//
// Returns:
// path id if successful, -1 if unsuccessful

with (objPathFinder) {

var startNode,goalNode,openList,closedList,node,neighbors,i,neighborNode,
    isOpen,isClosed,costFromStart;

if (coordMode) {
    startNode = cells[argument0,argument1];
    goalNode = cells[argument2,argument3];
    argument0 = argument0 * CELL_WIDTH + X_OFFSET;
    argument1 = argument1 * CELL_HEIGHT + Y_OFFSET;
} else {
    startNode = cells[floor((argument0-X_OFFSET)/CELL_WIDTH),
                floor((argument1-Y_OFFSET)/CELL_HEIGHT)];
    goalNode = cells[floor((argument2-X_OFFSET)/CELL_WIDTH),
               floor((argument3-Y_OFFSET)/CELL_HEIGHT)];
}

if (ds_map_find_value(goalNode,"blocked"))
    return -1;

openList = ds_priority_create();
closedList = ds_list_create();

ds_map_replace(startNode,"costFromStart",0);
ds_map_replace(startNode,"estimatedCostToGoal",AStar_getEstimatedCost(startNode,goalNode));
ds_map_replace(startNode,"pathParent",-1);
ds_priority_add(openList,startNode,ds_map_find_value(startNode,"costFromStart")+ds_map_find_value(startNode,"estimatedCostToGoal"));
while (!ds_priority_empty(openList)) {
    node = ds_priority_delete_min(openList);
    if (node == goalNode) {
        ds_priority_destroy(openList);
        ds_list_destroy(closedList);
        return AStar_constructPath(goalNode,argument0,argument1,argument4);
    }
    neighbors = AStar_getNeighbors(node);
    for (i=0; i<ds_list_size(neighbors); i+=1) {
        neighborNode = ds_list_find_value(neighbors,i);
        if (ds_priority_find_priority(openList,neighborNode) > 0)
            isOpen = 1;
        else isOpen = 0;
        if (ds_list_find_index(closedList,neighborNode) > -1)
            isClosed = 1;
        else isClosed = 0;
        
        costFromStart = ds_map_find_value(node,"costFromStart") + AStar_getCost(node,neighborNode);
        if ((!isOpen && !isClosed) || (costFromStart < ds_map_find_value(neighborNode,"costFromStart"))) {
            ds_map_replace(neighborNode,"pathParent",node);
            ds_map_replace(neighborNode,"costFromStart",costFromStart);
            ds_map_replace(neighborNode,"estimatedCostToGoal",
                           AStar_getEstimatedCost(neighborNode,goalNode));
            if (isClosed)
                ds_list_delete(closedList,ds_list_find_index(closedList,neighborNode));
            if (!isOpen) {
                ds_priority_add(openList,neighborNode,ds_map_find_value(neighborNode,"costFromStart")
                                +ds_map_find_value(neighborNode,"estimatedCostToGoal"));
            }
        }
    }
    ds_list_destroy(neighbors);
    ds_list_add(closedList,node);
}

ds_priority_destroy(openList);
ds_list_destroy(closedList);

return -1;

}

#define AStar_getCost_new
// Arguments:
// 0 - node 1
// 1 - node 2
//
// Returns:
// exact cost of moving from node 1 to node 2

if ((ds_map_find_value(argument0,"x") - ds_map_find_value(argument1,"x")) *
   (ds_map_find_value(argument0,"y") - ds_map_find_value(argument1,"y")) == 0)
  return costNormal;
return costDiag;

#define AStar_getEstimatedCost_new
// Arguments:
// 0 - node 1
// 1 - node 2
//
// Returns:
// estimated cost of moving from node 1 to node 2

if (allowDiag) {
    var xDist,yDist;
    xDist = abs(ds_map_find_value(argument0,"x") - ds_map_find_value(argument1,"x"));
    yDist = abs(ds_map_find_value(argument0,"y") - ds_map_find_value(argument1,"y"));
    if (xDist >= yDist)
        return costDiag * yDist + costNormal * (xDist - yDist);
    else
        return costDiag * xDist + costNormal * (yDist - xDist);
}
return abs(ds_map_find_value(argument0,"x") - ds_map_find_value(argument1,"x"))
       + abs(ds_map_find_value(argument0,"y") - ds_map_find_value(argument1,"y"));

#define AStar_getNeighbors_new
// Arguments:
// 0 - node
//
// Returns:
// list of nodes that are accesible in one step from the argument node

var list,a,b,c,d,listGet,a1,b1;
list = ds_list_create();
a = ds_map_find_value(argument0,"x");
b = ds_map_find_value(argument0,"y");

for (c=0; c<8; c+=!allowDiag+1) {
    if (!ds_map_find_value(cells[a,b],"dir"+string(c))) {
        d = c * pi / 4;
        a1 = a + round(cos(d));
        b1 = b - round(sin(d));
        
        if (a1 >= 0 && b1 >= 0 && a1 < GRID_WIDTH && b1 < GRID_HEIGHT) {
            listGet = cells[a1,b1];
            can = 1;
            if (!allowCut) && (c mod 2) {
                if (ds_map_find_value(cells[a,b1],"blocked"))
                or (ds_map_find_value(cells[a1,b],"blocked"))
                    can = 0;
            }
            if (can)
            if (!ds_map_find_value(listGet,"blocked"))
            if (!ds_map_find_value(listGet,"dir"+string((c+4) mod 8)))
                ds_list_add(list,listGet);
        }
    }
}

return list;

#define AStar_constructPath_new
// Arguments:
// 0 - goal node
// 1 - start node x
// 2 - start node y
// 3 - center (true/false)
//
// Returns:
// path id

var node,pathList,a,b;
node = argument0;

pathList = ds_list_create();
while (ds_map_find_value(node,"pathParent") > -1) {
    ds_list_insert(pathList,0,node);
    node = ds_map_find_value(node,"pathParent");
}

if (pathMode) {
    thePath = path_add();
    path_set_closed(thePath,0);
}
else
    path_clear_points(thePath);

path_add_point(thePath,argument1,argument2,100);
for (a=0; a<ds_list_size(pathList); a+=1) {
    node = ds_list_find_value(pathList,a);
    path_add_point(thePath,(ds_map_find_value(node,"x")+argument3/2)
                   *CELL_WIDTH+X_OFFSET,(ds_map_find_value(node,"y")
                   +argument3/2)*CELL_HEIGHT+Y_OFFSET,100);
}

ds_list_destroy(pathList);

return thePath;