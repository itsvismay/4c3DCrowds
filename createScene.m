global agents info fig nodes p11 p12 p13 p14 p21 p22 p23 p24 sx sy sz
rev = 1;
nodes = 40;
theta = linspace(0, 2*pi*rev, nodes)';

t = linspace(0,10, nodes)';
x = cos(theta);
y = sin(theta);
z = 0*theta;

x1 = x;
y1 = z;
z1 = y;

%x,y,z,t, const agent idx, traj vert idx
agents = [x y z t 0*ones(size(t)) linspace(1, size(t,1), size(t,1))'; 
          x1 y1 z1 t ones(size(t)) linspace(1, size(t,1), size(t,1))'];

fig = figure;
t = agents(1:nodes, 4);
    x = agents(1:nodes, 1);
    y = agents(1:nodes, 2);
    z = agents(1:nodes, 3);
    
    x1 = agents(nodes+1:end,1);
    y1 = agents(nodes+1:end,2);
    z1 = agents(nodes+1:end,3);
    t1 = agents(nodes+1:end,4);
    tiledlayout(2,2);
    nexttile
    p11 = plot3(x,y,z,'r-o', 'buttondownfcn',{@Mouse_Callback,'down'}); hold on;
    p21 = plot3(x1,y1,z1,'b-o', 'buttondownfcn',{@Mouse_Callback,'down'});
    title("xyz");
    
    nexttile
    p12 = plot3(x,y,t, 'r-o', 'buttondownfcn',{@Mouse_Callback,'down'}); hold on;
    p22 = plot3(x1,y1,t1, 'b-o', 'buttondownfcn',{@Mouse_Callback,'down'});
    title("xyt");
    
    nexttile
    p13= plot3(x,z,t, 'r-o', 'buttondownfcn',{@Mouse_Callback,'down'}); hold on;
    p23 = plot3(x1,z1,t1, 'b-o', 'buttondownfcn',{@Mouse_Callback,'down'});
    title("xzt");
    
    nexttile
    p14 = plot3(y,z,t, 'r-o', 'buttondownfcn',{@Mouse_Callback,'down'}); hold on;
    p24 = plot3(y1,z1,t1, 'b-o', 'buttondownfcn',{@Mouse_Callback,'down'});
    title("yzt");


fig1 = uifigure;
sx = uislider(fig1, 'Position',[100 150 200 3], 'ValueChangingFcn',@(sx,event) sX(event));
sx.Limits = [-0.5 0.5];
sx.Value = 0;

sy = uislider(fig1, 'Position',[100 100 200 3], 'ValueChangingFcn',@(sy,event) sY(event));
sy.Limits = [-0.5 0.5];
sy.Value = 0;

sz = uislider(fig1, 'Position',[100 50 200 3], 'ValueChangingFcn',@(sz,event) sZ(event));
sz.Limits = [-0.5 0.5];
sz.Value = 0;

% Create a UI axes
ax = uiaxes('Parent',fig1,...
            'Units','pixels',...
            'Position', [10, 200, 300, 210]);  

set(ax,'DataAspectRatioMode','manual')
set(ax,'DataAspectRatio',[2 2 2])
set(ax,'PlotBoxAspectRatioMode','manual')
set(ax,'PlotBoxAspectRatio',[2 2 2])
ax.XLimMode = 'manual';
ax.YLimMode = 'manual';
ax.ZLimMode = 'manual';
ax.XLim = [-2 2];
ax.YLim = [-2 2];
ax.ZLim = [-2 2];

[V, F] = readOBJ("meshes\starfish.obj");
sld = uislider(fig1, 'Position',[350 10 350 3], 'Orientation', 'vertical', 'ValueChangingFcn',@(sld,event) SLD(event, ax, V, F));
sld.Limits = [t(1) t(end)];
sld.Value = t(1);

% Create the function for the ButtonPushedFcn callback
function SLD(event,ax, V, F)
        global agents nodes
        cla(ax);
        frame = event.Value;
        
        for a = 0:1
            curAgent = agents(nodes*a+1:nodes*a+nodes, :);
            [~, fidx] = min(abs(curAgent(:, 4) - frame));
            Va = V + curAgent(fidx, 1:3);
            color = [0 0 0];
            if a==0
                color = [1 0 0];
            else 
                color = [0 0 1];
            end
            tsurf(F, Va, 'Parent', ax, "FaceColor", color); hold on;
        end
end

function updateSubplots()
    global fig agents nodes  p11 p12 p13 p14 p21 p22 p23 p24
    t = agents(1:nodes, 4);
    x = agents(1:nodes, 1);
    y = agents(1:nodes, 2);
    z = agents(1:nodes, 3);
    
    x1 = agents(nodes+1:end,1);
    y1 = agents(nodes+1:end,2);
    z1 = agents(nodes+1:end,3);
    t1 = agents(nodes+1:end,4);

    p11.XData = x;
    p11.YData = y;
    p11.ZData = z;
    p21.XData = x1;
    p21.YData = y1;
    p21.ZData = z1;

    p12.XData = x;
    p12.YData = y;
    p12.ZData = t;
    p22.XData = x1;
    p22.YData = y1;
    p22.ZData = t1;
    
    p13.XData = x;
    p13.YData = z;
    p13.ZData = t;
    p23.XData = x1;
    p23.YData = z1;
    p23.ZData = t1;
    
    p14.XData = y;
    p14.YData = z;
    p14.ZData = t;
    p24.XData = y1;
    p24.YData = z1;
    p24.ZData = t1;
end


function sX(event)
    global agents info
    h = findobj(gca,'Tag','pt');
    h.XData = event.Value;
    idx = info(end);
    axidx = 1;
    if(get(get(gca, 'Title'), 'String') == 'xyt')
        axidx = 1;
    elseif(get(get(gca, 'Title'), 'String') == 'xzt')
        axidx = 1;
    elseif(get(get(gca, 'Title'), 'String') == 'yzt')
        axidx = 2;
    end
    agents(idx,1) = h.XData;
    updateSubplots()
end

function sY(event)
    global agents info
    h = findobj(gca,'Tag','pt');
    h.YData = event.Value;
    idx = info(end);
    axidx = 2;
    if(get(get(gca, 'Title'), 'String') == 'xyt')
        axidx = 2;
    elseif(get(get(gca, 'Title'), 'String') == 'xzt')
        axidx = 3;
    elseif(get(get(gca, 'Title'), 'String') == 'yzt')
        axidx = 3;
    end
    agents(idx,axidx) = h.YData;
    updateSubplots()
end

function sZ(event)
    global agents info nodes
    h = findobj(gca,'Tag','pt');
    h.ZData = event.Value;
    idx = info(end);
    axidx = 3;
    if(get(get(gca, 'Title'), 'String') == 'xyt')
        axidx = 4;
        agents(idx,axidx) = h.ZData;
        %make sure all previous times are smaller than h.ZData
        a = floor(idx/nodes);
        curAgent = agents(nodes*a+1:nodes*a+nodes, :);
        curIdx = rem(idx, nodes);
        curAgent(:, 4) = cleanTimesAroundIdx(curAgent(:,4), curIdx);
        agents(nodes*a+1:nodes*a+nodes, :) = curAgent;
    elseif(get(get(gca, 'Title'), 'String') == 'xzt')
        axidx = 4;
        agents(idx,axidx) = h.ZData;
        %make sure all previous times are smaller than h.ZData
        a = floor(idx/nodes);
        curAgent = agents(nodes*a+1:nodes*a+nodes, :);
        curIdx = rem(idx, nodes);
        curAgent(:, 4) = cleanTimesAroundIdx(curAgent(:,4), curIdx);
        agents(nodes*a+1:nodes*a+nodes, :) = curAgent;
    elseif(get(get(gca, 'Title'), 'String') == 'yzt')
        axidx = 4;
        agents(idx,axidx) = h.ZData;
        %make sure all previous times are smaller than h.ZData
        a = floor(idx/nodes);
        curAgent = agents(nodes*a+1:nodes*a+nodes, :);
        curIdx = rem(idx, nodes);
        curAgent(:, 4) = cleanTimesAroundIdx(curAgent(:,4), curIdx);
        agents(nodes*a+1:nodes*a+nodes, :) = curAgent;
    else
        agents(idx,axidx) = h.ZData;
    end
    updateSubplots()
end

function [messyTimes] = cleanTimesAroundIdx(messyTimes, idx)
    curTime = messyTimes(idx);
    if(curTime<0)
        curTime = 0;
    elseif (curTime>15)
        curTime = 15;
    end
    
    %% First fix lower times
    %first index of time thats higher than curTime
    [~,fIdx] = max((curTime - messyTimes(1:idx-1)) <= 0);
    lowerval =0;
    if(fIdx>1)
        lowerval = messyTimes(fIdx -1);
        upperval = curTime;
        messyTimes(fIdx:idx-1) = linspace(lowerval,  upperval, idx - fIdx);
    end
    
    
    %% Next fix higher times
    %first index of time thats lower than curTime
    [~,fIdx] = min((curTime - messyTimes(1:end)) >= 0);
    lowerval = curTime;
    upperval = curTime;
    if(fIdx>1)
        upperval = messyTimes(fIdx);
    else
        %set to last idx
        %if curTime is larger than all other times
        fIdx = size(messyTimes, 1);
    end
    messyTimes(idx+1 : fIdx) = linspace(lowerval,  upperval, fIdx - idx);

end

 function Mouse_Callback(hObj,~,action)
    global agents info sx sy sz
    persistent curobj
    action
    switch action
        case 'down'
            info = whereDidIClick(agents); %x,y,z,t,agent idx, traj point idx
            curobj = hObj;
            delta = 2;
            sx.Limits = [info(1)- delta info(1)+ delta]; sx.Value = info(1);
            sy.Limits = [info(2)- delta info(2)+ delta]; sy.Value = info(2);
            sz.Limits = [info(3)- delta info(3)+ delta]; sz.Value = info(3);
            if(get(get(gca, 'Title'), 'String') == 'xyt')
                sx.Limits = [info(1)- delta info(1)+ delta]; sx.Value = info(1);
                sy.Limits = [info(2)- delta info(2)+ delta]; sy.Value = info(2);
                sz.Limits = [info(4)- delta info(4)+ delta]; sz.Value = info(4);
            elseif(get(get(gca, 'Title'), 'String') == 'xzt')
                sx.Limits = [info(1)- delta info(1)+ delta]; sx.Value = info(1);
                sy.Limits = [info(3)- delta info(3)+ delta]; sy.Value = info(3);
                sz.Limits = [info(4)- delta info(4)+ delta]; sz.Value = info(4);
            elseif(get(get(gca, 'Title'), 'String') == 'yzt')
                sx.Limits = [info(2)- delta info(2)+ delta]; sx.Value = info(2);
                sy.Limits = [info(3)- delta info(3)+ delta]; sy.Value = info(3);
                sz.Limits = [info(4)- delta info(4)+ delta]; sz.Value = info(4);
            end
%         case 'move'
%             whereDidIMove(info, curobj);
%         case 'up'
%             set(gcf,...
%               'WindowButtonMotionFcn',  '',...
%               'WindowButtonUpFcn',      '');
    end

 end

 function [info] = whereDidIClick(agents)
    point = get(gca, 'CurrentPoint'); % mouse click position
    camPos = get(gca, 'CameraPosition'); % camera position
    camTgt = get(gca, 'CameraTarget'); % where the camera is pointing to
    camDir = camPos - camTgt; % camera direction
    camUpVect = get(gca, 'CameraUpVector'); % camera 'up' vector
    % build an orthonormal frame based on the viewing direction and the 
    % up vector (the "view frame")
    zAxis = camDir/norm(camDir);    
    upAxis = camUpVect/norm(camUpVect); 
    xAxis = cross(upAxis, zAxis);
    yAxis = cross(zAxis, xAxis);
    rot = [xAxis; yAxis; zAxis]; % view rotation 
    % the clicked point represented in the view frame
    rotatedPointFront = rot * point' ;
    % the point cloud represented in the view frame
    pointCloud = agents(:, 1:3)';
    if(get(get(gca, 'Title'), 'String') == 'xyt')
        pointCloud(3,:) = agents(:,4)';
    elseif(get(get(gca, 'Title'), 'String') == 'xzt')
        pointCloud(2:3,:) = agents(:,3:4)';
    elseif(get(get(gca, 'Title'), 'String') == 'yzt')
        pointCloud = agents(:,2:4)';
    end
    rotatedPointCloud = rot * pointCloud; 
    % find the nearest neighbour to the clicked point 
    pointCloudIndex = dsearchn(rotatedPointCloud(1:2,:)', ... 
        rotatedPointFront(1:2));
    h = findobj(gca,'Tag','pt'); % try to find the old point
    selectedPoint = pointCloud(:, pointCloudIndex); 
    if isempty(h) % if it's the first click (i.e. no previous point to delete)
        
        % highlight the selected point
        h = plot3(selectedPoint(1,:), selectedPoint(2,:), ...
            selectedPoint(3,:), 'r.', 'MarkerSize', 20); 
        set(h,'Tag','pt'); % set its Tag property for later use  
        
    else % if it is not the first click
        delete(h); % delete the previously selected point
        
        % highlight the newly selected point
        h = plot3(selectedPoint(1,:), selectedPoint(2,:), ...
            selectedPoint(3,:), 'r.', 'MarkerSize', 20);  
        set(h,'Tag','pt');  % set its Tag property for later use
    end
    info = [agents(pointCloudIndex,:) pointCloudIndex];
 end


 function whereDidIMove(info, curobj)
    point = get(gca, 'CurrentPoint'); % mouse click position
    camPos = get(gca, 'CameraPosition'); % camera position
    camTgt = get(gca, 'CameraTarget'); % where the camera is pointing to
    camDir = camPos - camTgt; % camera direction
    camUpVect = get(gca, 'CameraUpVector'); % camera 'up' vector
    % build an orthonormal frame based on the viewing direction and the 
    % up vector (the "view frame")
    zAxis = camDir/norm(camDir);    
    upAxis = camUpVect/norm(camUpVect); 
    xAxis = cross(upAxis, zAxis);
    yAxis = cross(zAxis, xAxis);
    rot = [xAxis; yAxis; zAxis]; % view rotation 
    % the clicked point represented in the view frame
    rotatedPointFront = rot * point' ;
    xdata = get(curobj, 'xdata');
    ydata = get(curobj, 'xdata');
    zdata = get(curobj, 'xdata');
    xdata(info(6)) = rotatedPointFront(1);
    ydata(info(6)) = rotatedPointFront(2);
    zdata(info(6)) = rotatedPointFront(3);
    set(curobj,'xdata',zdata)
    set(curobj,'ydata',ydata)
    set(curobj,'zdata',zdata)
 end

 % Callback function for each point
% function Mouse_Callback(hObj,~,action)
%     persistent curobj xdata ydata ind
%     pos = get(gca,'CurrentPoint');
%     switch action
%       case 'down'
%           curobj = hObj;
%           xdata = get(hObj,'xdata');
%           ydata = get(hObj,'ydata');
%           [~,ind] = min(sum((xdata-pos(1)).^2+(ydata-pos(3)).^2,1));
%           disp(pos)
%           disp(ind)
% %           set(gcf,...
% %               'WindowButtonMotionFcn',  {@Mouse_Callback,'move'},...
% %               'WindowButtonUpFcn',      {@Mouse_Callback,'up'});
% %       case 'move'
% %           % vertical move
% %           ydata(ind) = pos(3);
% %           set(curobj,'ydata',ydata)
% %       case 'up'
% %           set(gcf,...
% %               'WindowButtonMotionFcn',  '',...
% %               'WindowButtonUpFcn',      '');
%     end
% end