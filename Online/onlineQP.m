function u=onlineQP(g, V_iter, deriv_x, x0, gamma, dt)
%% Grid

t = 0;

% x = nan(4,length(sim_t));
% u = nan(2,length(sim_t));
% d = nan(2,length(sim_t));
%
% u_delta = nan(3,length(sim_t));
% x(:,1) = x0;


H_delta = [ 0 , 0  ,0 ; 0,0,0 ; 0,  0 , 1];
f_delta = [0,0,0];
lb_delta = [-pi/2; -2; 0];
ub_delta = [pi/2; 2; inf];

H = eye(2);
f = zeros(2,1);
lb = [-pi/2;-2];
ub = [pi/2;2];

options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');


%% Setup without slack variable
LgV1 = deriv_x(3);
LgV2 = deriv_x(4);
d = [0;0];

LfV = deriv_x(1)*x0(4)*cos(x0(3)) + deriv_x(2)*x0(4)*sin(x0(3)) + ...
    + deriv_x(1)*d(1) +  deriv_x(2)*d(2);

A_delta = [ LgV1 , LgV2 , -1 ; 0, 0, -1 ];
b_delta = [ -LfV - gamma*V_iter ; 0];
[u_delta,~,flag] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);

A = [LgV1 , LgV2];
b = -LfV - gamma*V_iter + u_delta(3); % The 0.01 here accounts for the
%                                               convergence threshold of
%                                               CLVF
[u,~,flag] = quadprog(H,f,A,b,[],[],lb,ub);
% [ts_temp1, xs_temp1] = ode45(@(t,y) Dcar(t,y,u,d), [t t+dt], x0);
% x = xs_temp1(end,:);
% 
% function dydt = Dcar(t,s,u,d)
% dydt = [s(4)*cos(s(3)) + d(1);s(4)*sin(s(3)) + d(2);u(1);u(2)];
% end
end