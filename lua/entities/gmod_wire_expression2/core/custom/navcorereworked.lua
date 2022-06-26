E2Lib.RegisterExtension("ReworkedNavcore", false, "Add various functions for pathfinding relating to navmeshes", "Players generating complex navmeshes without command console restrictions set can lag the server, default restrictions should work fine")

--local sbox_E2_Navcore = CreateConVar("sbox_E2_Navcore", "2", FCVAR_ARCHIVE)

NavCore = {}

local StepHeight = 18
local FallHeight = 200
local SlopeLimit = 30
local FollowerSize = 22
local FollowerHeight = 50
local RemoveDist = 22
local CheckAreaLimit = CreateConVar( "navcore_areachecklimit", "5000", FCVAR_ARCHIVE, "How many areas a pathfind is able to check without finding the goal before using what it has", 0 )
local CheckRateLimit = CreateConVar( "navcore_pathfindratelimit", "0.3", FCVAR_ARCHIVE, "How quickly a player can run pathfinding functions", 0 )

local LastPathing = WireLib.RegisterPlayerTable()

--Below commented areas was for a built in queueing system, as the pathfind rate limit is shared for all E2s of a player. However having people coding their own within E2 might work out better.
--I might add back in if people find it necessary

--local PlayerE2PathingQueue = WireLib.RegisterPlayerTable()

function NavCore.CanRunAstar( player, self ) --Check if it hasn't been to soon since the previous pathfind
	local cur, last = CurTime(), LastPathing[player] or 0
	if((cur-last)<CheckRateLimit:GetFloat())then return false end
	--if(PlayerE2PathingQueue[1] != self.entity)then return false end
	--table.remove( PlayerE2PathingQueue, 1 )
	--table.insert( PlayerE2PathingQueue, self.entity )
	return true
end

function NavCore.CanRunAstarUpdate( player ) --Set the time since last pathfind to now
	LastPathing[player] = CurTime()
end

function NavCore.Astar( self, start, goal ) --Main portion of pathfinding code
	if( !IsValid(start)||!IsValid(goal) ) then return false end
	if( start == goal ) then return true end
	
	start:ClearSearchLists()
	
	start:AddToOpenList()
	
	local Closest = start
	
	local cameFrom = {}
	
	start:SetCostSoFar( 0 )
	start:SetTotalCost( NavCore.h_cost_estimate(start, goal) )
	start:UpdateOnOpenList()
	
	local checkedareas = 0
	local validitychecks = 0
	
	while( !start:IsOpenListEmpty() ) do
		local current = start:PopOpenList()
		
		if(current == goal) then
			return NavCore.reconstruct_path(cameFrom, current, self)
		end
		
		current:AddToClosedList()
		
		if(checkedareas>CheckAreaLimit:GetInt())then
			return NavCore.reconstruct_path(cameFrom, Closest, self)
		end
		
		if(self.data.BadAreas[current:GetID()])then
			continue
		end
		
		if(!self.data.GoodAreas[current:GetID()])then
			if(validitychecks<3)then
				if(!NavCore.NavareaMeetsRequirements(current, self))then
					self.data.BadAreas[current:GetID()] = true
					continue
				else
					self.data.GoodAreas[current:GetID()] = true
				end
				validitychecks = validitychecks + 1
			end
		end
		
		for k, neighbor in pairs( current:GetAdjacentAreas() ) do
			local newCostSoFar = current:GetCostSoFar() + NavCore.h_cost_estimate( current, neighbor ) + 200 --The extra value is make paths with large amounts of navareas less perferable, usually choosing a path further from limit
			
			checkedareas = checkedareas + 1
			if(!NavCore.NavareaNeighborChecks(current, neighbor, self))then
				continue
			end
			
			if( ( neighbor:IsOpen() || neighbor:IsClosed() ) && (neighbor:GetCostSoFar() <= newCostSoFar) ) then
				continue
			else
				neighbor:SetCostSoFar( newCostSoFar )
				neighbor:SetTotalCost( newCostSoFar + NavCore.h_cost_estimate( neighbor, goal ) )
				
				if( neighbor:IsClosed() )then
					neighbor:RemoveFromClosedList()
				end
				
				if(neighbor:IsOpen())then
					neighbor:UpdateOnOpenList()
				else
					neighbor:AddToOpenList()
				end
				
				if((Closest:GetCenter():DistToSqr(goal:GetCenter()))>(neighbor:GetCenter():DistToSqr(goal:GetCenter())))then
					Closest = neighbor
				end
				
				cameFrom[neighbor:GetID()] = current:GetID()
			end
		end
	end
	
	return false
	
end

function NavCore.h_cost_estimate(first, second)
	return first:GetCenter():Distance(second:GetCenter())
end

function NavCore.reconstruct_path(from, start, self) --Rebuild path from the last area backwards
	local total_path = { start }
	
	local current = start:GetID()
	while ( from[current] ) do
		current = from[ current ]
		if(!NavCore.NavareaMeetsRequirements(navmesh.GetNavAreaByID( current ), self))then
			self.data.BadAreas[current] = true
		end
		table.insert( total_path, navmesh.GetNavAreaByID( current ) )
	end
	return total_path
end

function NavCore.PathToVector(path) --Gets all navareas in a table and converts to a table of vectors of each areas center
	local returntable = {}
	for i, area in pairs( path ) do
		returntable[i] = area:GetCenter()
    end
    return returntable
end

function NavCore.GetNearestPointBounds(navarea,vector,dist) --gets closest point on area that has a distance from the edges
	local MaxBounds = navarea:GetCenter()+Vector(navarea:GetSizeX()/2-dist,navarea:GetSizeY()/2-dist,200)
	local MinBounds = navarea:GetCenter()-Vector(navarea:GetSizeX()/2-dist,navarea:GetSizeY()/2-dist,-200)
	local Closest = navarea:GetClosestPointOnArea(vector)
	
	local X = math.Clamp( Closest.x, MinBounds.x, MaxBounds.x )
	local Y = math.Clamp( Closest.y, MinBounds.y, MaxBounds.y )
	
	local aim = Entity( 1 ):GetEyeTrace().HitPos
	
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
	local returntable = {}
	for i, area in pairs( path ) do
		local point
		if((i+1)<=#path)then
			point2 = NavCore.GetNearestPointBounds(area, path[i+1]:GetCenter(), FollowerSize/2)
		else
			point2 = nil
		end
		if((i-1)>0)then
			point = NavCore.GetNearestPointBounds(area, path[i-1]:GetCenter(), FollowerSize/2)
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
    return returntable
end

function NavCore.NavareaMeetsRequirements(navarea, self) --Checks if navarea has enough space for a follower to get through, as well as isn't to steep

	local prop = self.data.navprop
	
	local tr = util.TraceLine( {
		start = navarea:GetCenter()+Vector(0,0,20),
		endpos = navarea:GetCenter()-Vector(0,0,20),
		mask = MASK_PLAYERSOLID_BRUSHONLY
	} )
	
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
	
	if ( tr.HitNormal.z<((90-prop.SlopeLimit)/90) ) then
		return false
	end
	
	return true
end

function NavCore.NavareaNeighborChecks(current, neighbor, self) --Some of the less intense neighbor checks relating to step and fall heights
	
	local prop = self.data.navprop

	local curpos = current:GetClosestPointOnArea(neighbor:GetCenter())
	local neipos = neighbor:GetClosestPointOnArea(current:GetCenter())
	
	if((curpos.z-neipos.z)>prop.FallHeight)then
		return false
	end
	if((neipos.z-curpos.z)>prop.StepHeight)then
		return false
	end
	return true
end

function NavCore.FindNearestValidArea(Pos, Range, Maxup, Maxdown, self) --Check within a radius for the closest navarea that meets navSet.. requirements
	local Dist = math.huge
	local Nearest = nil
	local possible = navmesh.Find(Pos, Range, Maxup, Maxdown)
	
	for k, v in pairs( possible ) do
		local closestpoint = v:GetClosestPointOnArea( Pos )
		
		if(closestpoint:DistToSqr( Pos )<Dist)then
			if(NavCore.NavareaMeetsRequirements(v, self))then
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

__e2setcost(1)
e2function number navCanPathFind()
	return NavCore.CanRunAstar( self.player, self )
end

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

e2function navarea operator=(navarea lhs, navarea rhs)
	local scope = self.Scopes[ args[4] ]
	scope[lhs] = rhs
	scope.vclk[lhs] = true
	return rhs
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

__e2setcost(500)
e2function array navGenPathSimple(vector startpos, vector goalpos)
	if(NavCore.CanRunAstar(self.player, self))then
		local startvec = Vector(startpos[1],startpos[2],startpos[3])
		local goalvec = Vector(goalpos[1],goalpos[2],goalpos[3])
		
		if(startvec == nil)then
			local startvec = startpos
		end
		if(goalvec == nil)then
			local goalvec = goalpos
		end
		
		local start = startvec
		
		NavCore.CheckNavProperties( self )
		
		local goal = goalvec

		local navstart = NavCore.FindNearestValidArea( start, 3000, StepHeight, FallHeight, self )
		local navgoal = NavCore.FindNearestValidArea( goal, 3000, StepHeight, FallHeight, self )
		
		self.data.BadAreas = self.data.BadAreas or {}
		self.data.GoodAreas = self.data.GoodAreas or {}
		
		self.data.PrevPath = self.data.PrevPath or {}
		
		NavCore.CanRunAstarUpdate( self.player )
		Path = NavCore.Astar( self, navstart, navgoal, filter )
		
		if(Path == true)then
			self.data.PrevPath = {goalpos}
			return {goalpos}
		end
		if(Path == false)then
			return {}
		end
		
		Path = NavCore.PathToVectorNearest(Path, start, start, goal, self )
		self.data.PrevPath = Path
	end
	
	if(self.data.PrevPath)then
		return self.data.PrevPath
	end
	return {}
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
