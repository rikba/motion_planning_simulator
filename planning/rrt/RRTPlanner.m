% A simple RRT motion planner

classdef RRTPlanner
%#codegen
    %RRTPlanner Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % default start and goal [x y]
        start = [0.0 0.0];
        goal = [0.8 0.2];
        
        % default state space [xmin xmax ymin ymax]
        space = [-1 1 -1 1];
        
        % default max. number vertices
        N = 2000;
        
        % default edge length in meter
        epsilon = 0.1;
        
        % tree structure
        rrt = struct('vertices',[],'numvertices',[], ...
            'edges',[]);
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
            if ( (N>0) && (mod(N,1)==0) )
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

%% sample
% Samples uniformly in state space. In expectation try to connect to the
% goal every 100 samples.
        function q_rand = sample(self)
            x = self.space(1) + (self.space(2)-self.space(1))*rand;
            y = self.space(3) + (self.space(4)-self.space(3))*rand;
            q_rand = [x y];
            if (rand>0.99)
                q_rand = self.goal;
            end
        end

%% addVertex
% Adds a vertex to the tree.
        function vertex_to_add = addVertex(self,q_new)
            vertex_to_add = q_new;
        end
%% addEdge
% Adds an edge from q_nearest to q_new to the tree.
        function edge_to_add = addEdge(self,q_nearest_idx,q_new)
            rrt_edge = q_new - self.rrt.vertices(q_nearest_idx);
            edge_start_idx = q_nearest_idx;
            edge_goal_idx = self.rrt.numvertices;
            edge_to_add = table(rrt_edge, edge_start_idx, edge_goal_idx);
        end
%% findNearestVertex
% Finds the nearest neighbor. Naive search.
        function [q_nearest_idx, d_min] = findNearestVertex(self,q_rand)
            d_min = Inf;
            for i = 1:self.rrt.numvertices
                d = norm(self.rrt.vertices(i,:) - q_rand);
                if(d < d_min)
                    d_min = d;
                    q_nearest_idx = i;
                end
            end
        end
 %% rrt_connect
 % connects the sampled vertex and the nearest vertex depending on their
 % distance
        function q_new = rrt_connect(self, q_nearest_idx, q_nearest_dist, q_rand)
            if(q_nearest_dist>self.epsilon)
                q_new = self.rrt.vertices(q_nearest_idx,:) + ...
                    self.epsilon * ...
                    (q_rand - self.rrt.vertices(q_nearest_idx,:)) / ...
                    q_nearest_dist;
            else
                q_new = q_rand;
            end
        end
     
         
%% solve
% Initialize RRT, build tree and find solution
        function reached = solve(self)
            t_rrt = tic;
            % init
            % tree
            self.rrt.vertices(1,:) = self.start;
            self.rrt.numvertices = 1;
            rrt_edge = [];
            edge_start_idx = [];
            edge_goal_idx = [];
            self.rrt.edges = table(rrt_edge, edge_start_idx, edge_goal_idx);
            % figure
            figure();
            hold on;
            start_goal = [self.start; self.goal];
            plot(start_goal(1,1),start_goal(1,2),'o','MarkerEdgeColor','k',...
                'MarkerFaceColor','g','MarkerSize',10);
            plot(start_goal(2,1),start_goal(2,2),'o','MarkerEdgeColor','k',...
                'MarkerFaceColor','r','MarkerSize',10);
            axis(self.space);
            
            
            reached = false;
            i = 1;
            % extend
            while (~reached && i<=self.N)
                % sample
                q_rand = sample(self);
                % nearest neighbor
                [q_nearest_idx, q_nearest_dist] = findNearestVertex(self, q_rand);
                % connect
                q_new = rrt_connect(self, q_nearest_idx, q_nearest_dist, q_rand);
                % addVertex
                self.rrt.vertices(end+1,:) = addVertex(self,q_new);
                self.rrt.numvertices = self.rrt.numvertices + 1;
                % addEdge
                self.rrt.edges = [self.rrt.edges; addEdge(self, q_nearest_idx, q_new)];
                i = i + 1;
                % check goal connection
                if q_new == self.goal
                    reached = true;
                end
                
                % draw vertices and edges
                edge_matrix = [self.rrt.vertices(self.rrt.edges.edge_start_idx(end),:);...
                    self.rrt.vertices(self.rrt.edges.edge_goal_idx(end),:)];
                plot(edge_matrix(:,1),edge_matrix(:,2),'color','k');
                
                
            end            
            t_elapsed = toc(t_rrt);
            if reached
                display('RRT solved!');
                fprintf('Solution time: %.3f s \n', t_elapsed);
            else
                display('No solution found.'); 
            end
            fprintf('Number of vertices: %d \n', self.rrt.numvertices);
        end
    end
end
