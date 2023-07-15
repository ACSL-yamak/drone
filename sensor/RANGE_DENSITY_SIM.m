classdef RANGE_DENSITY_SIM < handle
    % RangeDensityのsimulation用クラス：登録されたエージェントのうち半径内のエージェントの位置を返す
    %   rdensity = RANGE_DENSITY_SIM(Env)
    %   (optional) Env.r : 半径
    properties
        name = "";
        result
        target % 観測対象の環境
        self % センサーを積んでいる機体のhandle object

        ray_direction % LiDAR点群の照射方向 
        sensor_poly
    end
    properties (SetAccess = private) % construct したら変えない値．
        r = 10;
    end

    methods
        function obj = RANGE_DENSITY_SIM(self,Env)
            obj.self=self;
            if isfield(Env,'r'); obj.r= Env.r;end
            obj.ray_direction = -pi:0.01:pi;
            obj.sensor_poly = polyshape(obj.r*sin(obj.ray_direction), obj.r*cos(obj.ray_direction));
        end

        function result = do(obj,varargin)
            % result=rdensity.do(varargin) : obj.r 内のdensity mapを返す．
            %   result.state : State_obj,  p : position
            % 【入力】varargin = {{Env}}      agent : センサーを積んでいる機体obj,    Env：観測対象のEnv_obj
            Env=varargin{1}{4};
            pos=obj.self.plant.state.p; % Agentの現在位置
            env = Env.poly;


            %% センシング領域を定義
            sensor_range=translate(obj.sensor_poly , pos(1:2)'); % エージェントの位置を中心とした円

            %% 領域と環境のintersectionが測距領域
            region=intersect(sensor_range, env); % LiDARのサークルとENVを比較し切り出す
            circ = sensor_range.Vertices; % sensor_rangeの頂点

            result.angle =  obj.ray_direction;% NOTE 不要では？
            result.sensor_points = circ; % 配列の初期化
            for i = 1:length(circ)
                [in,~] = intersect(region, [pos(1:2)';circ(i, :) ]);
                if isempty(in)
                    % LiDARで測れる距離にいない
                    result.sensor_points(i,:) = pos(1:2)';% 測距距離が0とみなしAgent座標をLiDAR点群とする．
                else
                    [~,ii]= mink(vecnorm(in-pos(1:2)',2,2),2);
                    result.sensor_points(i,:) = in(ii(2),:);
                end
            end
            
            region = polyshape( result.sensor_points(:,1),result.sensor_points(:,2));

            %% 重み分布
            pmap_min = min(result.sensor_points,[],1);
            pmap_max = max(result.sensor_points,[],1);
            
            map_x_index = find(all([pmap_min(1) <= Env.xp ; Env.xp <= pmap_max(1) ],1)) ;
            map_y_index = find(all([pmap_min(2) <= Env.yp ; Env.yp <= pmap_max(2) ],1)) ;
            xp = Env.xp(map_x_index); yp = Env.yp(map_y_index);
            [xq,yq]= meshgrid(xp,yp);
            xq=xq'; yq=yq'; % cell indexは左上からだが，座標系は左下が基準なので座標系に合わせるように転置する．

            region_phi = Env.grid_density(map_x_index,map_y_index);
            in = reshape(isinterior(region,xq(:),yq(:)),size(xq));
            
            result.grid_density = region_phi.*in;
            result.xq=xq-pos(1);
            result.yq=yq-pos(2);
            result.map_min=pmap_min-pos(1:2)';
            result.map_max=pmap_max-pos(1:2)';
            result.region = translate(region,-pos(1:2)');
            obj.result = result;
        end
        function show(obj,varargin)
            if ~isempty(obj.result)
                contourf(obj.result.xq,obj.result.yq,obj.result.grid_density);
                %            surf(obj.result.xq,obj.result.yq,obj.result.grid_density);
                obj.draw_setting();
            else
                disp("do measure first.");
            end
        end
        function [] = draw_setting(obj)
            daspect([1 1 1])
            xlabel('x [m]');
            ylabel('y [m]');
            xlim([obj.result.map_min(1) obj.result.map_max(1)]);
            ylim([obj.result.map_min(2) obj.result.map_max(2)]);
            view(0, 90);
            cmap=[[1 1 1];parula];
            colormap(cmap)
            colorbar
            grid on;
        end

    end
end
