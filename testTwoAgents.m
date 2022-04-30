%% Testing One Agent
% 1. Construct agent
% 2. Create energy function
% 3. Setconstraints
% 4. Pass into fmincon
% 5. Render

%% Construct agent curves
revolutions = 0.5;
nodes = 10;
theta = linspace(0, 2*pi*revolutions, nodes)';

t = linspace(0,10, nodes)';
x = cos(theta)  + 0*rand(numel(t),1);
y = sin(theta) +  0*rand(numel(t),1);
z = 0*theta +  0*rand(numel(t),1);

x1 = x;
y1 = z;
z1 = y;

A = [x';y';z';t'];
fA = reshape(A, [1, numel(A)])';
B = [x1';y1';z1';t'];
fB = reshape(B, [1, numel(B)])';

%flattened agent cols
Q0 = [fA fB]; 
q = [fA; fB];

plottings(Q0,1);

%% Set Constraints
% time is monotonic inequality constraints 
% t_i+1 - t_i > 0 --> in this format --> A1*q <= 0
% And max time <= MT constraint
A1L = sparse([1:size(t,1)-1], [4:4:numel(fA)-4], ones(1,size(t,1)-1), size(t,1), numel(fA));
A1R = sparse([1:size(t,1)-1], [8:4:numel(fA)], -ones(1,size(t,1)-1), size(t,1), numel(fA));
A1MaxTime = sparse(size(t,1), numel(fA), 1, size(t,1), numel(fA));
A1 = A1L + A1R + A1MaxTime; % pick t_i+1, pick t_i, pick max time
b1 = [zeros(size(t,1)-1,1); 11]; % size of num segments and max time

A2L = sparse([1:size(t,1)-1], [4:4:numel(fB)-4], ones(1,size(t,1)-1), size(t,1), numel(fB));
A2R = sparse([1:size(t,1)-1], [8:4:numel(fB)], -ones(1,size(t,1)-1), size(t,1), numel(fB));
A2MaxTime = sparse(size(t,1), numel(fB), 1, size(t,1), numel(fB));
A2 = A2L + A2R + A2MaxTime; % pick t_i+1, pick t_i, pick max time
b2 = [zeros(size(t,1)-1,1); 11]; % size of num segments and max time

%Put A1 and A2 together
A = blkdiag(A1, A2);
b = [b1; b2];

% fix end points and waypoints equality constraints
endpoints = [1; 2; 3; 4;... %start xyzt of agent A
            size(fA,1)-3; size(fA,1)-2; size(fA,1)-1;... %end xyz of agent A
            size(fA,1)+1; size(fA,1)+2; size(fA,1)+3; size(fA,1)+4;... %start xyzt of Agent B
            size(fA,1)+size(fB,1)-3; size(fA,1)+size(fB,1)-2; size(fA,1)+size(fB,1)-1];
beq = q(endpoints(:));
Aeq = speye(size(q,1)); %set at I initially
Aeq = Aeq(endpoints(:), :); % set Aeq st. Aeq*q = beq

%% Optimizing
%minimize here
options = optimoptions('fmincon', ...
                        'Display', 'iter',...
                        'UseParallel', false);
                    
options.MaxFunctionEvaluations = 1e6;

[qn, fval, exitflag, output] = fmincon(@(q) energy(q,Q0),... 
                            q, ...
                            A,b,Aeq,beq,[],[], ...
                            [], options);
Qn = reshape(qn, size(Q0));
plottings(Qn,1);

%% Energy function
function [e] = energy(q, Q0)
    Q = reshape(q, size(Q0));
    E = 0;
    for i=1:2
        [F, Fblocks] = makeF(Q,Q0,i);
        [R, Rblocks, Sf, Sfblocks] = makeRS(Fblocks);
        e = Psi(Sf, Q, i);
        E = e + E;
    end
    
%     [F, Fblocks] = makeF(Q,Q0,1);
%     [R, Rblocks, Sf, Sfblocks] = makeRS(Fblocks);
%     e = Psi(Sf, Q, 1);
end

%% Compute Psi(KE) from strains
function [e] = Psi(S, Q, i)
    m = 1; % constant mass
    q_i = Q(:, i); %4*nodes
    dX = q_i(5:end) - q_i(1:end -4);
    dy = S*dX;
    dy = reshape(dy, 4, numel(q_i)/4-1)';
    e = 0.5*m*sum(sum(dy(:, 1:3).*dy(:,1:3),2)./dy(:,4)); %kinetic energy
end

% 4x4 matrix F = dx*dX' / l0 for each segment
function[F, Fblocks] = makeF(Qn, Q0,i)
    q0_i = Q0(:, i); %4*nodes
    qn_i = Qn(:, i); %4*nodes
    dX = reshape(q0_i(5:end) - q0_i(1:end -4), 4, numel(q0_i)/4-1)';
    dx = reshape(qn_i(5:end) - qn_i(1:end -4), 4, numel(qn_i)/4-1)';
    l0 = sum(dX.*dX, 2);
    Fblocks = {};
    for i=1:size(dX,1)
        Fblocks{i} = (dx(i,:)'*dX(i,:))/l0(i);
    end
    F = blkdiag(Fblocks{:});
    
    %A =  reshape(dX', 1, numel(dX))';
    %B = reshape(dx', 1, numel(dx))';
    %B - F*A should be 0 since dx = F*dX
end

%% Solve procrustes for 4x4 R
function [R, Rblocks,S, Sblocks] = makeRS(Fblocks)
    Rblocks = {};
    Sblocks = {};
    for i=1:size(Fblocks,2)
        [u,s,v] = svd(Fblocks{i});
        % compute polar decomposition of F
        % into rot/strain components
        rot = u*v';
        strain = v*s*v'; %symmetric straing
        Rblocks{i} = rot;
        Sblocks{i} = strain;
    end
    R = blkdiag(Rblocks{:});
    S = blkdiag(Sblocks{:});

end

%% Plottings
function [] = plottings(Q, i)
    Qi = reshape(Q(:,i), 4, numel(Q(:,i))/4)';

    x = Qi(:,1);
    y = Qi(:,2);
    z = Qi(:,3);
    t = Qi(:,4);
    
    figure;
    tiledlayout(2,2)
    nexttile
    p11 = plot3(x,y,z,'r-o');
    title("xyz");
    axis equal;
    
    nexttile
    p12 = plot3(x,y,t, 'r-o');
    title("xyt");
    axis equal;
    
    nexttile
    p13= plot3(x,z,t, 'r-o');
    title("xzt");
    axis equal;
    
    nexttile
    p14 = plot3(y,z,t, 'r-o');
    title("yzt");
    axis equal;
    
end