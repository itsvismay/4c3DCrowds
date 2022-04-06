%% Testing Energies

%% Construct agent curves
revolutions = 0.5;
nodes = 40;
theta = linspace(0, 2*pi*revolutions, nodes)';

t = linspace(0,10, nodes)';
x = cos(theta)  + rand(numel(t),1);
y = sin(theta) +  rand(numel(t),1);
z = 0*theta +  rand(numel(t),1);

x1 = x;
y1 = z;
z1 = y;
 
plot3(x,y,z,'ro-');

A = [x';y';z';t'];
fA = reshape(A, [1, numel(A)])';
B = [x1';y1';z1';t'];
fB = reshape(B, [1, numel(B)])';
Q = [fA fB]; %flattened agent cols

%% Get the traditional KE
a1KE = kineticEnergy(Q, 1);
a1KE

%% Get new KE
%% psi(S) - L:(RS - F)
[F, Fblocks] = makeF(Q,Q,1);
[R, Rblocks, Sf, Sfblocks] = makeRS(Fblocks);
psi1 = Psi(Sf, Q, 1);
psi1


%% Make def Grad F
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
    % A - F*A should be 0
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

%% Make Diag Strain matrix from Q
function [S, Sblocks] = makeS(Q, i)
    q_i = Q(:, i); %4*nodes
    dx = reshape(q_i(5:end) - q_i(1:end -4), 4, numel(q_i)/4-1)';
    for i=1:size(dx,1)
        Sblocks{i} = diag(dx(i,1:4));
    end
    S = blkdiag(Sblocks{:});
end
%% Traditional KE Function
function [e] = kineticEnergy(Q, i)
    %various fun path energies, I'll use principle of least action becase I like it
    %kinetic energy of curve integrated along piecewise linear segment is 
    % 0.5*dx'*dx./dt
    m = 1; % constant mass
    q_i = Q(:, i); %4*nodes
    dx = reshape(q_i(5:end) - q_i(1:end -4), 4, numel(q_i)/4-1)';

    e = 0.5*m*sum(sum(dx(:, 1:3).*dx(:,1:3),2)./dx(:,4)); %kinetic energy
end