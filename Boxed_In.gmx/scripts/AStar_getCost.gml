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
