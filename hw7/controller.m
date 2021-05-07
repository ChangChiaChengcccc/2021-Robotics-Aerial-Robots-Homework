function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
xd    = u(1:3);
b1d   = u(4:6);

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);


%parameter 
e3      = [0 0 1];
Rd      = [1 0 0;
           0 1 0;
           0 0 1];
       
Omega_d = [0 0.1 1];

eR    = 0.5 * vee(Rd'*R - R'*Rd);


eOmega  = Omega - R' * Rd * Omega_d'; 

J = [P.Jxx  0      0;
     0      P.Jyy  0;
     0      0      P.Jzz] ;



f =  (  P.kx * ( x(3)-xd(3) ) +...
        P.kv * ( v(3)-0 ) +...
        P.mass*P.gravity -...
        P.mass*0) /...
        dot(e3,(R * e3'));
%tmp = J * Omega ;
%tmp =  cross(Omega,(J * Omega);
M =  (  -P.kR * eR - P.kOmega * eOmega + cross(Omega,(J * Omega)));

%{     
f = 0;
%}
%M = [0; 0; 0];
    
out = [f;M];
end