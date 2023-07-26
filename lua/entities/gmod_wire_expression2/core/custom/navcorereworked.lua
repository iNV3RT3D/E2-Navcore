E2Lib.RegisterExtension("ReworkedNavcore", false, "Add various functions for pathfinding relating to navmeshes", "Players generating complex navmeshes without command console restrictions set can lag the server, default restrictions should work fine")

NavCore = {}

local StepHeight = 18
local FallHeight = 200
local SlopeLimit = 30
local FollowerSize = 22
local FollowerHeight = 50
local RemoveDist = 22
local StepsMaxAmount = CreateConVar( "navcore_stepsmax", "20", FCVAR_ARCHIVE, "The max amount of steps allowed per tick", 0 )

NavCore.CurrentPathing = nil

local lastStepInterval = CurTime()
local StepsAvailable = WireLib.RegisterPlayerTable()

hook.Add("Think", "ResetSteps", function()
	for I, P in ipairs(player.GetAll()) do
		StepsAvailable[P] = StepsMaxAmount:GetInt()
	end
end)

function NavCore.UseSteps(player, steps)
	local availablesteps = StepsAvailable[player]
	local clamped = math.Clamp(availablesteps, 0, steps)
	StepsAvailable[player] = StepsAvailable[player] - clamped
	return clamped
end

function NavCore.PathToVector(path) --Gets all navareas in a table and converts to a table of vectors of each areas center
	local returntable = {}
	for i, area in pairs( path ) do
		returntable[i] = area:GetCenter()
    end
    return returntable
end

function NavCore.InitializeNavPathfinder(pather, startarea, endarea, prop)
	
	local ValidCheck = function(area, secarea)
		area = navmesh.GetNavAreaByID(area)
		secarea = navmesh.GetNavAreaByID(secarea)
		
		--close_area = area:GetClosestPointOnArea( secarea:GetCenter() )
		--close_secarea = secarea:GetClosestPointOnArea( area:GetCenter() )
		
		if (area:ComputeAdjacentConnectionHeightChange( secarea ) > prop.StepHeight) or (area:ComputeAdjacentConnectionHeightChange( secarea ) < -prop.FallHeight) then
			return false
		end
		
		local dt = {start = secarea:GetCenter()+Vector(0,0,10), endpos = secarea:GetCenter()+Vector(0,0,prop.FollowerHeight), collisiongroup = COLLISION_GROUP_DEBRIS}
		local trace = util.TraceLine( dt )
		if trace.Hit then
			return false
		end
		
		dt = {start = secarea:GetCenter()+Vector(0,0,prop.FollowerHeight), endpos = secarea:GetCenter()+Vector(0,0,-50), collisiongroup = COLLISION_GROUP_DEBRIS}
		trace = util.TraceLine( dt )
		local hitang = trace.HitNormal:Angle()
		hitang.pitch = math.NormalizeAngle(hitang.pitch+90)
		if math.abs(hitang.pitch) > prop.SlopeLimit then
			return false
		end
		
		dt = {start = secarea:GetCenter()+Vector(0,0,prop.FollowerHeight/2), endpos = secarea:GetCenter()+Vector(0,0,prop.FollowerHeight/2), maxs = Vector(prop.FollowerSize,prop.FollowerSize,1), mins = -Vector(prop.FollowerSize,prop.FollowerSize,1), collisiongroup = COLLISION_GROUP_DEBRIS}
		trace = util.TraceHull( dt )
		
		if(trace.Hit)then
			return false
		end
		
		--[[
		local traces = 0
		local maxtraces = 8
		while traces<maxtraces do
			dt = {start = secarea:GetCenter()+Vector(0,0,prop.FollowerHeight/2), endpos = secarea:GetCenter()+Vector(0,0,prop.FollowerHeight/2)+Angle(0,360/maxtraces*traces,0):Forward()*FollowerSize}
			trace = util.TraceLine( dt )
			
			traces = traces + 1
			
			if(trace.HitWorld) then return false end
		end
		]]--
		
		return true
	end

	Finders:NavInitialize(pather, startarea, endarea, ValidCheck, nil, nil)
end

function NavCore.GetNearestPointBounds(navarea,vector,dist) --gets closest point on area that has a distance from the edges
	local MaxBounds = navarea:GetCenter()+Vector(navarea:GetSizeX()/2-dist,navarea:GetSizeY()/2-dist,200)
	local MinBounds = navarea:GetCenter()-Vector(navarea:GetSizeX()/2-dist,navarea:GetSizeY()/2-dist,-200)
	local Closest = navarea:GetClosestPointOnArea(vector)
	
	local X = math.Clamp( Closest.x, MinBounds.x, MaxBounds.x )
	local Y = math.Clamp( Closest.y, MinBounds.y, MaxBounds.y )
	
	if((navarea:GetSizeX()/2<dist))then
		X = navarea:GetCenter().x
	end
	
	if((navarea:GetSizeY()/2<dist))then
		Y = navarea:GetCenter().y
	end
	
	return Vector(X,Y,Closest.z)
end

function NavCore.ToClose(point, tocheck, self) --check if a vector is to close to another based on the navSet.. functions
	local prop = self.data.navprop

	if(point == nil)then
		return false
	end
	if(tocheck == nil)then
		return false
	end
	local Dir = point-tocheck
	if(Dir:Length2DSqr()<(prop.RemoveDist*prop.RemoveDist))then
		return true
	end
	return false
end

function NavCore.PathToVectorNearest(path, removepoint, start, goal, self) --Gets all navareas in a table and converts to a table of vectors of nearest point within bounds
	local prop = self.data.navprop
	
	local path = table.Reverse(path)
	
	local returntable = {}
	local point
	local point2
	for i, area in pairs( path ) do
		local p2exist
		local p1exist
		if((i+1)<=#path)then
			p2exist = area:GetClosestPointOnArea(path[i+1]:GetCenter())
			p2exist = (p2exist+path[i+1]:GetClosestPointOnArea(area:GetCenter()))/2
		end
		if((i-1)>0)then
			p1exist = area:GetClosestPointOnArea(path[i-1]:GetCenter())
			p1exist = (p1exist+path[i-1]:GetClosestPointOnArea(area:GetCenter()))/2
		end
		
		if((i+1)<=#path)then
			point2 = NavCore.GetNearestPointBounds(area, p2exist, prop.FollowerSize/2)
		else
			point2 = nil
		end
		if((i-1)>0)then
			point = NavCore.GetNearestPointBounds(area, p1exist, prop.FollowerSize/2)
		else
			point = area:GetCenter()
		end
		if(removepoint!=nil)then
			if(!NavCore.ToClose(removepoint, point, self))then
				table.insert(returntable, point)
			end
			if(!NavCore.ToClose(removepoint, point2, self))then
				table.insert(returntable, point2)
			end
		else
			table.insert(returntable, point)
			table.insert(returntable, point2)
		end
    end
    return table.Reverse(returntable)
end

function NavCore.NavareaMeetsRequirements(navarea, self) --Checks if navarea has enough space for a follower to get through, as well as isn't to steep
	local prop = self.data.navprop
	
	local tr = util.TraceLine( {
		start = navarea:GetCenter()+Vector(0,0,100),
		endpos = navarea:GetCenter()-Vector(0,0,100),
		mask = MASK_PLAYERSOLID_BRUSHONLY	
	} )
	
	if(tr.Hit)then
		if ( tr.HitNormal.z<((90-prop.SlopeLimit)/90) ) then
			return false
		end
	end
	
	local trh = util.TraceHull( {
		start = navarea:GetCenter()+Vector(0,0,prop.StepHeight*2),
		endpos = navarea:GetCenter()+Vector(0,0,prop.FollowerHeight),
		maxs = Vector(prop.FollowerSize,prop.FollowerSize,1)/2,
		mins = -Vector(prop.FollowerSize,prop.FollowerSize,1)/2,
		mask = MASK_PLAYERSOLID_BRUSHONLY	
	} )
		
	if ( trh.Hit || trh.StartSolid ) then
		return false
	end
	
	return true
end

function NavCore.NavareaNeighborChecks(current, neighbor, self) --Some of the less intense neighbor checks relating to step and fall heights
	
	local att = neighbor:GetAttributes()
	
	local prop = self.data.navprop

	local curpos = current:GetClosestPointOnArea(neighbor:GetCenter())
	local neipos = neighbor:GetClosestPointOnArea(current:GetCenter())
	
	if(!neighbor:HasAttributes( NAV_MESH_STAIRS ) or !current:HasAttributes( NAV_MESH_STAIRS ))then
		if((curpos.z-neipos.z)>prop.FallHeight)then
			return false
		end
		if((neipos.z-curpos.z)>prop.StepHeight)then
			return false
		end
	end
	return true
end

function NavCore.SharedCheck(current, neighbor, params)
	if !NavCore.NavareaNeighborChecks(current, neighbor, params.self) then return false end
	if !NavCore.NavareaMeetsRequirements(neighbor, params.self) then return false end
	return true
end

function NavCore.FindNearestValidArea(Pos, Range, Maxup, Maxdown, self) --Check within a radius for the closest navarea that meets navSet.. requirements
	local Dist = math.huge
	local Nearest = nil
	local possible = navmesh.Find(Pos, Range, Maxup, Maxdown)
	
	for k, v in pairs( possible ) do
		local closestpoint = v:GetClosestPointOnArea( Pos )
		
		if(closestpoint:DistToSqr( Pos )<Dist)then
			if(NavCore.NavareaMeetsRequirements(v, self ))then
				Dist = closestpoint:DistToSqr( Pos )
				Nearest = v
			end
		end

	end
	
	return Nearest
	
end

function NavCore.CheckNavProperties( self ) --Check if an E2 is somehow missing navigation properties
	if(!self.data.navprop)then
		self.data.navprop = {}
		local quick = self.data.navprop
		quick.StepHeight = StepHeight
		quick.FallHeight = FallHeight
		quick.SlopeLimit = SlopeLimit
		quick.FollowerSize = FollowerSize
		quick.FollowerHeight = FollowerHeight
		quick.RemoveDist = RemoveDist
	end
end

function NavCore.IsCNavArea(possible) --Is this value a CNavArea
	if (type(possible) == "CNavArea") then
		return true
	end
end

e2function number navarea:isValid()
	return IsValid(this) and 1 or 0
end

registerType("navarea", "xna", nil, 
	nil,
	nil,
	function(ret)
		if not ret then return end
		if not NavCore.IsCNavArea(ret) then throw("Somehow xna is type: ", type(ret)) end
	end,
	function(xna)
		return NavCore.IsCNavArea(xna)
	end
)

registerType("pather", "xpa", nil, 
	nil,
	nil,
	function(ret)
		if not ret then return end
		if not NavCore.IsCNavArea(ret) then throw("Somehow xpa is type: ", type(ret)) end
	end,
	function(xpa)
		return NavCore.IsCNavArea(xpa)
	end
)

e2function navarea operator=(navarea lhs, navarea rhs)
	local scope = self.Scopes[ args[4] ]
	scope[lhs] = rhs
	scope.vclk[lhs] = true
	return rhs
end

e2function pather operator=(pather lhs, pather rhs)
	local scope = self.Scopes[ args[4] ]
	scope[lhs] = rhs
	scope.vclk[lhs] = true
	return rhs
end

__e2setcost(1)
e2function void navSetStepHeight(number height)
	NavCore.CheckNavProperties( self )
	self.data.navprop.StepHeight = height
end

e2function void navSetFallHeight(number height)
	NavCore.CheckNavProperties( self )
	self.data.navprop.FallHeight = height
end

e2function void navSetSlopeLimit(number limit)
	NavCore.CheckNavProperties( self )
	self.data.navprop.SlopeLimit = limit
end

e2function void navSetFollowerSize(number size)
	NavCore.CheckNavProperties( self )
	self.data.navprop.FollowerSize = size
end

e2function void navSetFollowerHeight(number height)
	NavCore.CheckNavProperties( self )
	self.data.navprop.FollowerHeight = height
end

e2function void navSetFollowerHeight(number height)
	NavCore.CheckNavProperties( self )
	self.data.navprop.FollowerHeight = height
end

e2function void navSetNextPointDist(number range)
	NavCore.CheckNavProperties( self )
	self.data.navprop.RemoveDist = range
end

__e2setcost(10)
e2function navarea navFindNearestNavArea(vector findfrom)
	return navmesh.GetNearestNavArea( findfrom )
end

e2function navarea navFindNearestValidNavArea(vector findfrom)
	return NavCore.FindNearestValidArea(findfrom, 3000, 3000, 3000, self)
end

e2function vector navarea:navAreaGetCenter()
	return this:GetCenter()
end

e2function vector navarea:navAreaGetClosestPoint(vector to)
	return this:GetClosestPointOnArea( to )
end


__e2setcost(20)
e2function number navBeginPathSimple(vector start, vector goal)
	self.pather = Pathfind:CreatePathfinder()
	
	local startarea = NavCore.FindNearestValidArea(start, 3000, 3000, 3000, self)
	local endarea = NavCore.FindNearestValidArea(goal, 3000, 3000, 3000, self)
	if(IsValid(startarea) and IsValid(endarea))then
		--self.pather:PathFindBegin(startarea, endarea)
		local prop = self.data.navprop
		
		NavCore.InitializeNavPathfinder(self.pather, startarea, endarea, prop)

		return 1
	end
	return 0
end
--local pather = Pathfind.CreateNavmeshPathfinder()
__e2setcost(50)
e2function string navPathStepSimple(number steps)
	local usesteps = NavCore.UseSteps( self.player, steps )

	local state = self.pather:Step(usesteps)

	if state == true then return "Complete" end
	if state == false then return "Failed" end
	if(!self.pather.Pathing) then return "Not Pathing" end
	return "In Progress"
end

__e2setcost(20)
e2function number navPathAreasChecked()
	if !(self.pather) then return -1 end
	return self.pather.Checked
end

__e2setcost(20)
e2function number navTotalAreas()
	return navmesh.GetNavAreaCount()
end

e2function array navReturnPathSimple()
	local Path = self.pather.Path
	--NavCore.PathToVectorNearest(path, removepoint, start, goal, self)
	--function NavCore.PathToVectorNearest(path, removepoint, self)
	if !Path or #Path == 0 then return {} end
	if(#Path == 1)then 
		if(self.pather.GoalVec)then
			return {self.pather.GoalVec} 
		else
			return {Path[1]:GetCenter()} 
		end
	end
	local RPath = table.Reverse(Path)
	RPath = NavCore.PathToVectorNearest(RPath, RPath[1]:GetCenter(), RPath[1]:GetCenter(), RPath[#RPath]:GetCenter(), self )
	return RPath
end

registerCallback("construct",
	function(self)
		NavCore.CheckNavProperties( self )
		--table.insert( PlayerE2PathingQueue, self.entity )
	end
)

registerCallback("destruct",
	function(self)
		--table.RemoveByValue( PlayerE2PathingQueue, self.entity )
	end
)
