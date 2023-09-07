classdef A_STAR_REFERENCE < handle
  properties
    self
    result
  end

  methods
    function obj = A_STAR_REFERENCE(self,varargin)
      obj.self = self;
      obj.result.state = STATE_CLASS(struct('state_list',["xd","p","v"],'num_list',[20,3,3]));
    end
    function  result= do(obj,varargin)
        env = varargin{4};

        route = a_star();
        %% Creation of RESULT variable
        result = obj.result; % Resultの初期定義
        result.route = route;
        result.state.p = route(2,:)';
        result.state.p(3) = 1 ;% 指定高度

        obj.result = result;
    end
        
  end
end

%% A* algorithm
function route = a_star(start,goal,env)
    %% Setting
    d = env.d;
    xp = env.xp;
    yp = env.yp;
    grid_size = [length(xp) length(yp)]; 
    grid = env.grid_in;
    poly = env.poly;
    move = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
    move_cost = [1 sqrt(2) 1 sqrt(2) 1 sqrt(2) 1 sqrt(2) ]*d;
    % move = [1 0;  0 1;  -1 0; 0 -1];
    % move_cost = [1  1  1  1 ]*d;
    
    %% Initialize
    start_grid = [find(xp+d/2 >= start(1),1) find(yp+d/2 >= start(2),1) ];
    goal_grid  = [find(xp+d/2 >= goal(1),1)  find(yp+d/2 >= goal(2),1) ];
    
    % ▼ Variable that stores the status of each node during traversal
    node_size         = 1; % 今保持しているノードの数
    node_grid         = start_grid ; % 各ノードの座標
    node_parent_index = 1; % 各ノードの親が保存されているインデックス
    node_real_cost    = 0; % 各ノードが持つ実コスト
    node_est_cost     = vecnorm(goal-start); % 各ノードが持つ推定コスト
    node_total_cost   = node_est_cost + node_real_cost; % 各ノードが持つ合計コスト
    node_status       = 1; % 各ノードの状態 0:探索済み 1:未探索 -1:無効
    
    move_len = length(move_cost);

    % close all
    % figure()
    % hold on
    % plot(env)
    
    attempt = 0;
    st = tic;

    %% Main algorithm
    loop_f = true;
    if all(start_grid==goal_grid,'all')
        loop_f=false;
        cand_grid = goal_grid;
        index = 1;
    end
    while loop_f
    
        % ▼ Decide where to explore
        index = find(node_status==1); % 未探索グリッドの検出
        index = index(node_total_cost(index) == min(node_total_cost(index))); % そのうちトータルコストが最小のindexを抽出
        index = index(node_real_cost(index) == min(node_real_cost(index)) ); % そのうち実コストが最小のindexを抽出
        % index = index(node_est_cost(index) == min(node_est_cost(index)) ); % そのうち推定コストが最小のindexを抽出
        index = index(end); % そのうちIndexが大きい方を抽出

        % ▼ Update parent node information
        node_status(index)    = 0;
        parent_grid  = node_grid(index,:);
        parent_realcost = node_real_cost(index);
        
        % ▼ Verify candidate searches from parent node in sequence
        for i = 1:move_len
            attempt = attempt +1;
            cand_pn =true;
            cand_grid = parent_grid + move(i,:);
            if ~ all(and([0 0]<cand_grid,cand_grid<=grid_size),2) % Gridが存在するかの判別
            elseif ~ grid(cand_grid(1),cand_grid(2))  % ENVのPolyshape内か判別
            else % 上記条件を満たす場合に実行
                cand_real_cost    = parent_realcost + move_cost(i);
                duplication_index = find(all(node_grid==cand_grid,2));
                
                if ~isempty(duplication_index)% 探索済と候補が重複している場合
                    if any(node_real_cost(duplication_index) <= cand_real_cost)
                        cand_pn=false; % 探索済みの実コストが小さい場合は候補を棄却
                    else 
                        node_status(duplication_index) = -1; % 候補の実コストが小さい場合は探索済みのステータスを無効に
                    end
                end
                if cand_pn
                    node_size = node_size + 1;
                    node_grid(node_size,:)       = cand_grid ;
                    node_parent_index(node_size) = index;
                    node_real_cost(node_size)    = cand_real_cost;
                    node_est_cost(node_size)     = vecnorm(goal-[xp(cand_grid(1)) yp(cand_grid(2))],1);
                    node_total_cost(node_size)   = node_est_cost(node_size) + cand_real_cost;
                    node_status(node_size)       = 1;
                    % 
                    if all(goal_grid == cand_grid)
                        loop_f=false; 
                        break;
                    end % 候補とゴールのグリッドが一致する場合に探索を終了
                end
            end
        end
    
    
        % cla;
        % hold on
        % plot(env.poly)
        % daspect([1 1 1])
        % 
        % scatter( xp(node_grid(node_status==0,1)) , yp(node_grid(node_status==0,2)) , "r.")
        % scatter( xp(node_grid(node_status==1,1)) , yp(node_grid(node_status==1,2)) , "b.")
        % scatter(start(1),start(2),'b*')
        % scatter(goal(1),goal(2),'r*')
        % drawnow
    end
    %% Routing

    grid_route = [ xp(cand_grid(1)) yp(cand_grid(2))];
    route = grid_route;
    env_buffer = polybuffer(poly , -d*sqrt(2)/2);
    while true
    
        grid_route = cat(1,[ xp(node_grid(index,1)) yp(node_grid(index,2))] , grid_route);
        [~,out_route] = intersect(env_buffer ,[ route(1,:) ;grid_route(1,:)]);
        if ~isempty(out_route)
           route = cat(1,grid_route(2,:),route);
        end
    
        % cla;
        % hold on
        % plot(env)
        % daspect([1 1 1])
        % plot(grid_route(:,1),grid_route(:,2),'g',LineWidth=2)
        % plot(route(:,1),route(:,2),'bo-',LineWidth=2)
        % plot(in_route(:,1),in_route(:,2),'b',out_route(:,1),out_route(:,2),'r')
        % drawnow
        if index == 1 
            route = cat(1,grid_route(1,:),route);
            break
        end
        index = node_parent_index(index);
    end
    %% Ploter
    % close all
    % figure()
    % cla;
    % hold on
    % plot(env.poly)
    % daspect([1 1 1])
    % scatter( xp(node_grid(node_status==0,1)) , yp(node_grid(node_status==0,2)) , "r.")
    % % scatter( xp(node_grid(node_status==-1,1)) , yp(node_grid(node_status==-1,2)) , "black.")
    % scatter( xp(node_grid(node_status==1,1)) , yp(node_grid(node_status==1,2)) , "r.")
    % scatter(start(1),start(2),'b*')
    % scatter(goal(1),goal(2),'r*')
    % 
    % plot(grid_route(:,1),grid_route(:,2),'g',LineWidth=1)
    % plot(route(:,1),route(:,2),'bo-',LineWidth=2)
    % % et = toc(st);
    % % attempt
    % % cand_real_cost
    % hold off
end