function g = InverseKinematics(Q_Spec)
    
    g{1} = Q_Spec;
    g{2} = getInverseParams(Q_Spec);
  
    if g{2} == 0
        warning('Out Of Bounds. dimensions not attainable!');
    else
        g{2} = checkThetaVal(g{2});
        g{3} = round(getMotorStepValues(g{2}), 0);
    end

end

function thetaParams = getInverseParams(Q_Spec)
K = 8.5;    l_1 = 8.5;   l_2 = 8.5;   l_3 = 6;    d_1 = 12.5;

a_x = Q_Spec(1,1); b_x = Q_Spec(1,2); c_x = Q_Spec(1,3); p_x = Q_Spec(1,4);

a_y = Q_Spec(2,1); b_y = Q_Spec(2,2); c_y = Q_Spec(2,3); p_y = Q_Spec(2,4);

a_z = Q_Spec(3,1); b_z = Q_Spec(3,2); c_z = Q_Spec(3,3); p_z = Q_Spec(3,4);

theta3 = asind(((K*c_z)+d_1-p_z)/l_3);
theta3 = [theta3 180-theta3];
newtheta3 = unique(checkThetaRange_and_Complex(theta3, 3));  %check theta3 values
placeHolder = newtheta3;
for i=1:length(placeHolder)
    p_znew = K*c_z - l_3*sind(placeHolder(i)) + d_1;
    if round(p_znew,1) ~= round(p_z,1)
        newtheta3 = newtheta3(newtheta3 ~= placeHolder(i));
    end
end

% for theta4
theta4 = [];
for i = 1:length(newtheta3)
    r = [1 -1] * sqrt(c_x^2 + c_y^2);
    temp = [acosd(r) -acosd(r)];
    for j = 1:length(temp)
        theta4 = [theta4 temp(j)+newtheta3(i)];
    end
end
newtheta4 = unique(round(checkThetaRange_and_Complex(theta4,4),2));  %check theta4 values
placeHolder = newtheta4;
for i=1:length(placeHolder)
    for j=1:length(newtheta3)
        c_znew = -cosd(newtheta3(j)-90-placeHolder(i));
        if round(c_znew,1) ~= round(c_z,1)
            newtheta4 = newtheta4(newtheta4 ~= placeHolder(i));
        end
    end
end

% for theta2
theta2 = [];
for i = 1:length(newtheta3)
    for j = 1:length(newtheta4)
        alpha = p_x^2 + p_y^2 - (K^2*((cosd(newtheta4(j)-newtheta3(i)))^2)) - ((2*l_3)*K*cosd(newtheta4(j)-newtheta3(i))*cosd(newtheta3(i))) - ((2*l_2)*K*cosd(newtheta4(j)-newtheta3(i))) - ((l_3^2)*((cosd(newtheta3(i)))^2)) - ((2*l_3*l_2)*cosd(newtheta3(i))) - (l_1^2) - (l_2^2);
        beta = ((2*l_1*K)*cosd(newtheta4(j)-newtheta3(i))) + ((2*l_3*l_1)*cosd(newtheta3(i))) + (2*l_1*l_2);
        result = [acosd(alpha/beta) -acosd(alpha/beta)];
        theta2 = [theta2 result];
    end
end
theta2 = checkThetaRange_and_Complex(theta2,2);  %check theta2 values

% for theta1
theta1 = [];
theta1_2 = [];
for i=1:length(theta2)
    for j=1:length(newtheta3)
        for k=1:length(newtheta4)
            first_val = acosd(c_x/(sind(90+newtheta4(k)-newtheta3(j))));
            second_val = asind(c_y/(sind(90+newtheta4(k)-newtheta3(j))));
            result = [(first_val + theta2(i)) ((-first_val) + theta2(i)) (second_val + theta2(i)) ((180 - second_val) + theta2(i))];
            theta1 = [theta1 result];
        end
        result = atan2d(p_y,p_x) + atan2d(sind(theta2(i))*((l_3*cosd(newtheta3(j))) + l_2),(cosd(theta2(i))*((l_3*cosd(newtheta3(j))) + l_2))+l_1);
        theta1_2 = [theta1_2 result];
    end
end
theta1 = checkThetaRange_and_Complex(theta1,1);  %check theta2 values
theta1_2 = checkThetaRange_and_Complex(theta1_2,1);  %check theta2 values

placeHolder1 = [theta1 theta1_2];
placeHolder2 = theta2;
newthetavalues = getNewThetaValues(placeHolder1, placeHolder2,newtheta3, newtheta4, K, l_1, l_2, l_3, p_x, p_y);
newtheta1 = newthetavalues{1};
newtheta2 = newthetavalues{2};

% for theta5
theta5 = [];
for i = 1:length(newtheta1)
    for j = 1:length(newtheta2)
        result = a_x*cosd(newtheta1(i)-newtheta2(j)) + a_y*sind(newtheta1(i)-newtheta2(j));
        theta5 = [theta5 acosd(result) -acosd(result)];
    end
end
newtheta5 = unique(round(checkThetaRange_and_Complex(theta5,5),2));  %check theta5 values

theta_vals = 0;
for i=1:length(newtheta1)
    for j=1:length(newtheta2)
        for k=1:length(newtheta3)
            for l=1:length(newtheta4)
                for m=1:length(newtheta5)
                    a_xnew = cosd(newtheta1(i)-newtheta2(j))*cosd(newtheta3(k)-90-newtheta4(l))*cosd(newtheta5(m)) + sind(newtheta1(i)-newtheta2(j))*sind(newtheta5(m));
                    b_xnew = -cosd(newtheta1(i)-newtheta2(j))*cosd(newtheta3(k)-90-newtheta4(l))*sind(newtheta5(m)) + sind(newtheta1(i)-newtheta2(j))*cosd(newtheta5(m));
                    c_xnew = cosd(newtheta1(i)-newtheta2(j))*sind(90+newtheta4(l)-newtheta3(k));
                    p_xnew = K*cosd(newtheta1(i)-newtheta2(j))*sind(90+newtheta4(l)-newtheta3(k)) + l_3*cosd(newtheta1(i)-newtheta2(j))*cosd(newtheta3(k)) + l_2*cosd(newtheta1(i)-newtheta2(j)) + l_1*cosd(newtheta1(i));
                    a_ynew = sind(newtheta1(i)-newtheta2(j))*cosd(newtheta3(k)-90-newtheta4(l))*cosd(newtheta5(m)) - cosd(newtheta1(i)-newtheta2(j))*sind(newtheta5(m));
                    b_ynew = -sind(newtheta1(i)-newtheta2(j))*cosd(newtheta3(k)-90-newtheta4(l))*sind(newtheta5(m)) - cosd(newtheta1(i)-newtheta2(j))*cosd(newtheta5(m));
                    c_ynew = sind(newtheta1(i)-newtheta2(j))*sind(90+newtheta4(l)-newtheta3(k));
                    p_ynew = K*sind(newtheta1(i)-newtheta2(j))*sind(90+newtheta4(l)-newtheta3(k)) + l_3*sind(newtheta1(i)-newtheta2(j))*cosd(newtheta3(k)) + l_2*sind(newtheta1(i)-newtheta2(j)) + l_1*sind(newtheta1(i));
                    a_znew = sind(90+newtheta4(l)-newtheta3(k))*cosd(newtheta5(m));
                    b_znew = -sind(90+newtheta4(l)-newtheta3(k))*sind(newtheta5(m));
                    c_znew = -cosd(newtheta3(k)-90-newtheta4(l));
                    p_znew = -K*cosd(newtheta3(k)-90-newtheta4(l)) + -l_3*sind(newtheta3(k)) + d_1;
                    Q_new = [a_xnew b_xnew c_xnew p_xnew; a_ynew b_ynew c_ynew p_ynew; a_znew b_znew c_znew p_znew; 0 0 0 1];
             
                    criteria1 = isalmost(Q_new(1:3,1:3),Q_Spec(1:3,1:3),0.1);
                    criteria2 = isalmost(Q_new(:,4),Q_Spec(:,4),3);
                    if all(criteria1(:)) && all(criteria2')
                        theta_vals = [newtheta1(i) newtheta2(j) newtheta3(k) newtheta4(l) newtheta5(m)];
                        if isequal(round(Q_new,1),round(Q_Spec,1))
                            thetaParams = [newtheta1(i) newtheta2(j) newtheta3(k) newtheta4(l) newtheta5(m)];
                            return
                        end  
                    end
                end
            end
        end
    end
end
 thetaParams = theta_vals;

end


function newThetaValues = checkThetaVal(thetaValues)

    for i = 1:length(thetaValues)
        if (i == 1 || i == 3) && (thetaValues(i) > 77.5 || thetaValues(i) < -77.5)
            if (thetaValues(i)-360 > 77.5 || thetaValues(i)-360 < -77.5)
                thetaValues(i) = sign(thetaValues(i))*77.5;
            else
                thetaValues(i) = thetaValues(i)-360;
            end
        end

        if (i == 2) && (thetaValues(i) > 60 || thetaValues(i) < -60)
            if (thetaValues(i)-360 > 60 || thetaValues(i)-360 < -60)
                thetaValues(i) = sign(thetaValues(i))*60;
            else
                thetaValues(i) = thetaValues(i)-360;
            end
        end

        if (i == 4) && (thetaValues(i) > 82.5 || thetaValues(i) < -82.5)
            if (thetaValues(i)-360 > 82.5 || thetaValues(i)-360 < -82.5)
                thetaValues(i) = sign(thetaValues(i))*82.5;
            else
                thetaValues(i) = thetaValues(i)-360;
            end
        end

        if (i == 5) && (thetaValues(i) > 75 || thetaValues(i) < -75)
            if (thetaValues(i)-360 > 75 || thetaValues(i)-360 < -75)
                thetaValues(i) = sign(thetaValues(i))*75;
            else
                thetaValues(i) = thetaValues(i)-360;
            end
        end
    end
    newThetaValues = thetaValues;
end

function newThetaValues = getNewThetaValues(placeHolder1, placeHolder2, newtheta3, newtheta4, K, l_1, l_2, l_3, p_x, p_y)
newtheta1 = [];
newtheta2 = [];
newThetaValues = {[] []};

    for i=1:length(placeHolder1)
        for j=1:length(placeHolder2)
            for k=1:length(newtheta3)
                for l=1:length(newtheta4)
                    p_xnew = K*cosd(placeHolder1(i)-placeHolder2(j))*sind(90+newtheta4(l)-newtheta3(k)) + l_3*cosd(placeHolder1(i)-placeHolder2(j))*cosd(newtheta3(k)) + l_2*cosd(placeHolder1(i)-placeHolder2(j)) + l_1*cosd(placeHolder1(i));
                    p_ynew = K*sind(placeHolder1(i)-placeHolder2(j))*sind(90+newtheta4(l)-newtheta3(k)) + l_3*sind(placeHolder1(i)-placeHolder2(j))*cosd(newtheta3(k)) + l_2*sind(placeHolder1(i)-placeHolder2(j)) + l_1*sind(placeHolder1(i));
                    if (round(p_xnew,0) == round(p_x,0)) && (round(p_ynew,0) == round(p_y,0))
                        newtheta1 = [newtheta1 placeHolder1(i)];
                        newtheta2 = [newtheta2 placeHolder2(j)];
                        newThetaValues = {newtheta1 newtheta2};
                        return
                    end
                end
            end
        end
    end
end

function forwardMatrix = getForwardMatrix(theta1, theta2, theta3, theta4, theta5, K, l_1, l_2, l_3, d_1)
% Generate A matrices and Q_Forward = Q_effector_base (matrix defining how
% to get the base to the end effector)
A1 = [cosd(theta1) sind(theta1) 0 (10*cosd(theta1)); sind(theta1) -cosd(theta1) 0 (l_1*sind(theta1)); 0 0 -1 d_1; 0 0 0 1];
A2 = [cosd(theta2) 0 sind(theta2) (10*cosd(theta2)); sind(theta2) 0 -cosd(theta2) (l_2*sind(theta2)); 0 1 0 0; 0 0 0 1];
A3 = [cosd(theta3) sind(theta3) 0 (6*cosd(theta3)); sind(theta3) -cosd(theta3) 0 (l_3*sind(theta3)); 0 0 -1 0; 0 0 0 1];
A4 = [cosd(90+theta4) 0 sind(90+theta4) 0; sind(90+theta4) 0 -cosd(90+theta4) 0; 0 1 0 0; 0 0 0 1];
A5 = [cosd(theta5) -sind(theta5) 0 0; sind(theta5) cosd(theta5) 0 0; 0 0 1 K; 0 0 0 1];

forwardMatrix = A1*A2*A3*A4*A5;

end

function QTRPY = getQTRPY(Trans_x,Trans_y,Trans_z,roll,pitch,yaw)

    Q_T = [1 0 0 Trans_x; 0 1 0 Trans_y; 0 0 1 Trans_z; 0 0 0 1];
    Q_R = [cosd(roll) -sind(roll) 0 0; sind(roll) cosd(roll) 0 0; 0 0 1 0; 0 0 0 1];
    Q_P = [cosd(pitch) 0 sind(pitch) 0; 0 1 0 0; -sind(pitch) 0 cosd(pitch) 0; 0 0 0 1];
    Q_Y = [1 0 0 0; 0 cosd(yaw) -sind(yaw) 0; 0 sind(yaw) cosd(yaw) 0; 0 0 0 1];
    QTRPY = Q_T*Q_R*Q_P*Q_Y;

end

% Functions to manipulate servos
% x1 is the amount of steps required to rotate servo 1 by y1 degrees
function motorValues = getMotorStepValues(y)

% First configuration [Preferred]. Setting x = 0 to represent y = 0 degrees.
% [Taking the home configuration as the refernce].
x1 = -y(1)/(0.0554);
x2 = y(2)/(0.0429);
x3 = y(3)/(0.0554);
x4 = y(4)/(0.0589);
x5 = -y(5)/(0.0536);

motorValues = [x1, x2, x3, x4, x5];

end

function newThetaValues = checkThetaRange_and_Complex(thetaValues, whichTheta)
newThetaValues = thetaValues;
% return;
    for i = 1:length(thetaValues)
        if (whichTheta == 1) && (isreal(thetaValues(i)) == 0)
            newThetaValues(i) = abs(newThetaValues(i));
        end

        if (whichTheta == 2) && (isreal(thetaValues(i)) == 0)
            newThetaValues(i) = abs(newThetaValues(i));
        end
        
        if (whichTheta == 3) && (thetaValues(i) > 77.5 || thetaValues(i) < -77.5 || isreal(thetaValues(i)) == 0) && (thetaValues(i)-360 > 77.5 || thetaValues(i)-360 < -77.5)
            newThetaValues(i) = sign(newThetaValues(i))*77.5;
        end

        if (whichTheta == 4) && (thetaValues(i) > 82.5 || thetaValues(i) < -82.5 || isreal(thetaValues(i)) == 0) && (thetaValues(i)-360 > 82.5 || thetaValues(i)-360 < -82.5)
            newThetaValues(i) = sign(newThetaValues(i))*82.5;
        end

        if (whichTheta == 5) && (isreal(thetaValues(i)) == 0)
            newThetaValues(i) = sign(newThetaValues(i))*75;
        end
    end
    newThetaValues = newThetaValues(newThetaValues == real(newThetaValues));
end
