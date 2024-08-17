% close all; clear; clc
function newStates = AStar_path_poly(Init_x, Init_y, obs_map, cTEB)
% Main file for the A* algo.
%% First creat the map
% Map: Obstacle=-1,Target = 0,Robot=1,Space=2
max_x = 20;
max_y = 20;
% dr=1.2;
MAP=2*(ones(max_x,max_y));
% Init_x = 1.2;
% Init_y = 1.2;
Init_x = floor(Init_x);
Init_y = floor(Init_y);
dx = round(cTEB(1))
dy = round(cTEB(2))
% define target
Target_x = 16;
Target_y = 16;
MAP(Target_x,Target_y) = 0;

% define obstacles
% Obs_x = [5,5,6,6,6,6,7,7,7,7,7,7,8,8,8,8,8,9,9,9];
% Obs_y = [7,8,6,7,8,9,6,7,8,9,10,11,7,8,9,10,11,9,10,11];
% % Obs_x = [5,5,6,6,6,6,7,7,7,7,7,7,8,8,8,8,8,9,9,9,12,12,13,13,13];
% % Obs_y = [7,8,6,7,8,9,6,7,8,9,10,11,7,8,9,10,11,9,10,11,11,12,11,12,12];
% % num_obs = length(Obs_x);
% Ox1 = Obs_x+dr;
% Ox2 = Obs_x-dr;
% Oy1 = Obs_y+dr;
% Oy2 = Obs_y-dr;
% Aug_obsx = [Obs_x,Obs_x,Ox1,Ox2,Ox1,Ox1,Ox2,Ox2];
% Aug_obsy = [Oy1, Oy2, Obs_y,Obs_y ,Oy1,Oy2,Oy1,Oy2];
% Obs_x = [Obs_x,Aug_obsx];
% Obs_y = [Obs_y, Aug_obsy];
% num_obs = length(Obs_x);



for i = 1 : obs_map.num_obs
    x_range = obs_map.xv(i,1)-dx:obs_map.xv(i,2)+dx;
    y_range = obs_map.yv(i,1)-dx:obs_map.yv(i,3)+dy;
    obs_region = combvec(x_range,y_range);
    for j = 1:length(obs_region)
        MAP(obs_region(1,j),obs_region(2,j)) = -1;
    end
end

% define the initial point
% Init_x = 1;
% Init_y = 1;
MAP(Init_x,Init_y) = 1;

% plot the map
% figure
% set(gcf,'unit','normalized','position',[0.2,0.1,0.64,0.8]);
% plot(Target_x,Target_y,'g*','lineWidth', 2)
% hold on
% plot(Obs_x,Obs_y,'rd','lineWidth', 2)

% plot(Init_x,Init_x,'bo','lineWidth', 2)

% grid on
% axis([1 max_x+1 1 max_y+1])
% xticks(1:1:max_x+1)
% yticks(1:1:max_y+1)
% legend('goal','obstacle','initial point')

%% Main Algo
% Open list: x | y | Parent x | Parent y | h | g | f
% Close list: x | y

Open_list = [];
Closed_list = [];

%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:max_x
    for j=1:max_y
        if(MAP(i,j) == -1)
            Closed_list(k,1)=i;
            Closed_list(k,2)=j;
            k=k+1;
        end
    end
end
Closed_count=size(Closed_list,1);

% Initialize with [Init_x,Init_y]
x = Init_x;
y = Init_y;
Open_count = 1;
hn = 0; % Cost at the state
gn = distance(x,y,Target_x,Target_y); % cost-to-go (cost to the goal)
fn = hn+gn;
Open_list(Open_count,:)=insert_open(x,y,x,y,hn,gn,fn); % for init state,
% its parent is itself
% After calculating the costs for init state, put it into Closed_list
Closed_count=Closed_count+1;
Closed_list(Closed_count,1)=x;
Closed_list(Closed_count,2)=y;
NoPath=1;

% START the main loop
while((x ~= Target_x || y ~= Target_y) && NoPath == 1)
    exp_array=expand_array(x,y,hn,Target_x,Target_y,Closed_list,max_x,max_y);
    exp_count=size(exp_array,1); % Step 4 of the algo
    
    for i=1:exp_count
        flag=0;
        for j=1:Open_count
            % For all nodes in Open_list, check whether its the same as the
            % i-th exp_array. Update the cost at that node with the smaller one
            if(exp_array(i,1) == Open_list(j,2) && exp_array(i,2) == Open_list(j,3) )
                Open_list(j,8)=min(Open_list(j,8),exp_array(i,5));
                % Compare the cost for the same state. If exp is smaller, do
                % the following
                if Open_list(j,8)== exp_array(i,5)
                    %UPDATE PARENTS,gn,hn
                    Open_list(j,4)=x;
                    Open_list(j,5)=y;
                    Open_list(j,6)=exp_array(i,3);
                    Open_list(j,7)=exp_array(i,4);
                end
                flag=1; % for all nodes in the Open_list, if one it the same as
                % the i-th exp_array, flag is assigned 1.
            end
        end
        if flag == 0 % if none of the nodes in the Open_list is the same as ith
            % exp_array, insert the i-th to the Open_list, with parent
            % node x and y.
            Open_count = Open_count+1;
            Open_list(Open_count,:)=insert_open(exp_array(i,1),exp_array(i,2),x,y,exp_array(i,3),exp_array(i,4),exp_array(i,5));
        end%End of insert new element into the OPEN list
    end
    %Find out the node with the smallest fn
    index_min_node = min_fn(Open_list,Open_count,Target_x,Target_y);
    if (index_min_node ~= -1)
        %Set xNode and yNode to the node with minimum fn
        x=Open_list(index_min_node,2);
        y=Open_list(index_min_node,3);
        path_cost=Open_list(index_min_node,6);%Update the cost of reaching the parent node
        %Move the Node to list CLOSED
        Closed_count=Closed_count+1;
        Closed_list(Closed_count,1)=x;
        Closed_list(Closed_count,2)=y;
        Open_list(index_min_node,1)=0;
    else
        %No path exists to the Target!!
        NoPath=0;%Exits the loop!
    end%End of index_min_node check
end

i=size(Closed_list,1);
Optimal_path=[];
x=Closed_list(i,1);
y=Closed_list(i,2);
i=1;
Optimal_path(i,1)=x;
Optimal_path(i,2)=y;
i=i+1;

if ( (x == Target_x) && (y == Target_y))
    inode=0;
    %Traverse OPEN and determine the parent nodes
    parent_x=Open_list(node_index(Open_list,x,y),4);%node_index returns the index of the node
    parent_y=Open_list(node_index(Open_list,x,y),5);
    
    while( parent_x ~= Init_x || parent_y ~= Init_y)
        Optimal_path(i,1) = parent_x;
        Optimal_path(i,2) = parent_y;
        %Get the grandparents:-)
        inode=node_index(Open_list,parent_x,parent_y);
        parent_x=Open_list(inode,4);%node_index returns the index of the node
        parent_y=Open_list(inode,5);
        i=i+1;
    end
%  j=size(Optimal_path,1);
%  %Plot the Optimal Path!
%  p=plot(Optimal_path(j,1),Optimal_path(j,2),'bo');
% %  j=j;
%  for i=j:-1:1
%   pause(.25);
%   set(p,'XData',Optimal_path(i,1),'YData',Optimal_path(i,2));
%  drawnow ;
%  end
%  h=plot(Optimal_path(:,1),Optimal_path(:,2));
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end
Optimal_path = flip(Optimal_path);
Optimal_path = [Init_x,Init_y;Optimal_path];
newStates = [];
steps = 81;
for i = 1:length(Optimal_path)-1
    if Optimal_path(i,1) == Optimal_path(i+1,1)
        temp_x = ones(1,steps)*Optimal_path(i,1);
    else
        %         temp_x = Optimal_path(i,1):0.025:Optimal_path(i+1,1);
        temp_x = linspace( Optimal_path(i,1),Optimal_path(i+1,1),steps);
    end
    if Optimal_path(i,2) == Optimal_path(i+1,2)
        temp_y = ones(1,steps)*Optimal_path(i,2);
    else
        %         temp_y = Optimal_path(i,2):0.025:Optimal_path(i+1,2);
        temp_y = linspace( Optimal_path(i,2),Optimal_path(i+1,2),steps);
    end
    if ~isequal(i,length(Optimal_path)-1)
        temp_x(end) = [];
        temp_y(end) = [];
    end
    newStates = [newStates; temp_x',temp_y'];
end
end
%%
% map.space = MAP;
% map.obs = [Obs_x;Obs_y];
% map.target = [Target_x;Target_y];
% map.init = [Init_x;Init_y];
% map.opttraj = Optimal_path;
% save('map.mat','map')

