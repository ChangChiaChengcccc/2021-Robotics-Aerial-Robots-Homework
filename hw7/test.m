Rd = [1 0 0; 0 1 0; 0 0 1];
R  = [1 0 0; 0 1 0; 0 0 1];
eR = 0.5 * vee(Rd' * R - R' * Rd);

constant = 4+5+ ...
    +6;

e3 = [0 0 1];
ee = [0 5 6];
e2 = [0 1 0];
a  = cross(e2,e3');
m  = [1 0 0;
      0 1 0;
      0 0 1];
r1 = [0;0;0];
r2 = [0;0;0];
r3 = cross(r1,r2);