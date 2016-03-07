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
