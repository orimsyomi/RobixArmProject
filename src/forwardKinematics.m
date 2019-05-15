function f = ForwardKinematics(theta1, theta2, theta3, theta4, theta5)

f{1} = Q_Forward(theta1, theta2, theta3, theta4, theta5);
f{2} = LinearConfig1(-theta1, theta2, theta3, theta4, -theta5);
f{3} = LinearConfig2(theta1, theta2, theta3, theta4, theta5);
f{4} = Q_Forward_eq(theta1, theta2, theta3, theta4, theta5);
end

function f = Q_Forward(theta1, theta2, theta3, theta4, theta5)

% Generate A matrices and Q_Forward = Q_effector_base (matrix defining how
% to get the base to the end effector)
A1 = [cosd(theta1) sind(theta1) 0 (10*cosd(theta1)); sind(theta1) -cosd(theta1) 0 (10*sind(theta1)); 0 0 -1 12.5; 0 0 0 1];
A2 = [cosd(theta2) 0 sind(theta2) (10*cosd(theta2)); sind(theta2) 0 -cosd(theta2) (10*sind(theta2)); 0 1 0 0; 0 0 0 1];
A3 = [cosd(theta3) sind(theta3) 0 (6*cosd(theta3)); sind(theta3) -cosd(theta3) 0 (6*sind(theta3)); 0 0 -1 0; 0 0 0 1];
A4 = [cosd(90+theta4) 0 sind(90+theta4) 0; sind(90+theta4) 0 -cosd(90+theta4) 0; 0 1 0 0; 0 0 0 1];
A5 = [cosd(theta5) -sind(theta5) 0 0; sind(theta5) cosd(theta5) 0 0; 0 0 1 0; 0 0 0 1];

f = A1*A2*A3*A4*A5;

end

function f = Q_Forward_eq(theta1, theta2, theta3, theta4, theta5)

% Generate A matrices and Q_Forward = Q_effector_base (matrix defining how
% to get the base to the end effector)
ta = (cosd(theta1-theta2)*cosd(theta3-270-theta4)*cosd(theta5) + sind(theta1-theta2)*sind(theta5));
tb = (-cosd(theta1-theta2)*cosd(theta3-270-theta4)*sind(theta5) + sind(theta1-theta2)*cosd(theta5));
tc = (cosd(theta1-theta2)*sind(270+theta4-theta3));
td = ((6*cosd(theta1-theta2)*cosd(theta3))+(10*cosd(theta1-theta2))+(10*cosd(theta1)));
te = (sind(theta1-theta2)*cosd(theta3-270-theta4)*cosd(theta5) + cosd(theta1-theta2)*sind(theta5));
tf = (-sind(theta1-theta2)*cosd(theta3-270-theta4)*sind(theta5) + cosd(theta1-theta2)*cosd(theta5));
g = sind(theta1-theta2)*sind(270+theta4-theta3);
th = ((6*sind(theta1-theta2)*cosd(theta3))+(10*sind(theta1-theta2))+(10*sind(theta1)));
ti = sind(270+theta4-theta3)*cosd(theta5);
tj = -sind(270+theta4-theta3)*sind(theta5);
tk = -cos(theta3-270-theta4);
tl = (-6*sind(theta3))+12.5;

f = [ta tb tc td; te tf g th; ti tj tk tl; 0 0 0 1];

end

% Functions to manipulate servos
% x1 is the amount of steps required to rotate servo 1 by y1 degrees

function f = LinearConfig1(y1, y2, y3, y4, y5)

% First configuration [Preferred]. Setting x = 0 to represent y = 0 degrees.
% [Taking the home configuration as the refernce].
x1 = y1/(0.0554);
x2 = y2/(0.0429);
x3 = y3/(0.0554);
x4 = y4/(0.0589);
x5 = y5/(0.0536);

f = [x1, x2, x3, x4, x5];

end

function f = LinearConfig2(y1, y2, y3, y4, y5)

% Second configuration. Setting x = 1400 to represent y = 0 degrees.
x1 = ((-y1+76.6)-76.6)/(-0.0554);
x2 = ((-y2+59.94)-59.94)/(-0.0429);
x3 = ((-y3+76.6)-76.6)/(-0.0554);
x4 = ((-y4+82.54)-82.54)/(-0.0589);
x5 = ((-y5+74.96)-74.96)/(-0.0536);

f = [x1, x2, x3, x4, x5];

end