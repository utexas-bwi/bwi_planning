#begin_lua

loc_table={}
loc_table["cor"] = 0
loc_table["o1"] = 1
loc_table["o2"] = 2
loc_table["o3"] = 3
loc_table["lab1"] = 4
door_table={}
door_table["d1"] = 0
door_table["d2"] = 1
door_table["d3"] = 2
door_table["d4"] = 3
door_table["d5"] = 4
function dis(a,b,c)
	d1 = door_table[tostring(a)]
	d2 = door_table[tostring(b)]
	if d1==d2 then return 1 end
	loc = loc_table[tostring(c)]
	if loc==0 then
		if d1==0 then
			if d2==1 then return 10 end
			if d2==2 then return 15 end
			if d2==3 then return 10 end
			if d2==4 then return 15 end
		end
		if d1==1 then
			if d2==0 then return 10 end
			if d2==2 then return 10 end
			if d2==3 then return 10 end
			if d2==4 then return 10 end
		end
		if d1==2 then
			if d2==0 then return 15 end
			if d2==1 then return 10 end
			if d2==3 then return 15 end
			if d2==4 then return 10 end
		end
		if d1==3 then
			if d2==0 then return 10 end
			if d2==1 then return 10 end
			if d2==2 then return 15 end
			if d2==4 then return 15 end
		end
		if d1==4 then
			if d2==0 then return 15 end
			if d2==1 then return 10 end
			if d2==2 then return 10 end
			if d2==3 then return 15 end
		end
	end
	if loc==4 then
		if d1==3 then
			if d2==4 then return 15 end
		end
		if d1==4 then
			if d2==3 then return 15 end
		end
	end
	return 1
end

#end_lua.
