classdef RRTPlanner
    %RRTPlanner Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % default start and goal [x y]
        start = [0.2 0.5];
        goal = [0.8 0.5];
        
        % default state space [xmin xmax ymin ymax]
        space = [-1 1 -1 1];
        
        % default max. number vertices
        N = 2000;
        
        % default incremental distance in meter
        epsilon = 0.01;
        
    end
    
    methods
%% setStart
% Set the start position        
        function setStart(self,start)
            if self.isOutsideCFree(start)
                error('The start is outside of state space.');
            end
                
            self.start = start;
        end
%% setGoal
% Set the goal position        
        function setGoal(self,goal)
            if self.isOutsideCFree(goal)
                error('The goal is outside of state space.');
            end
                
            self.goal = goal;
        end
%% setSpace
% Set the configuration space
        function setSpace(self,xmin,xmax,ymin,ymax)
            if (xmin<xmax && ymin<ymax)
                self.space = [xmin xmax ymin ymax];
            else
                error('Invalid minimum and maximum.');
            end
        end
%% setMaxVertices
% Set the maximum number of vertices allowed in tree
        function setMaxVertices(self,N)
            if ( (N>0) & (mod(N,1)==0) )
                self.N = N;
            else
                error('N must be a positive integer.');
            end
        end
%% setIncrement
% Set the edge length
        function setIncrement(self,epsilon)
            if epsilon>0
                self.epsilon = epsilon;
            else
                error('Epsilon must be positive.');
            end
        end
%% isOutsideCFree
% Check if state is outside state space
        function result = isOutsideCFree(self,state)
            if (state(1)<self.space(1) || state(1)>self.space(2) || ...
                state(2)<self.space(3) || state(2)>self.space(4) )
                result = true;
                return;
            else
                result = false;
                return;
            end
        end        
%% solve
% Initialize RRT, build tree and find solution
        function result = solve(self)
            % init
            
        end
    end
