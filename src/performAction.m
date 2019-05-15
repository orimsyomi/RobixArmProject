function ActionFlag = performAction(O_T_Location, referenceLocation_wrt_Image, O_T_Type, whichObject)
    referenceLocation_wrt_Robot = [24,-24];

    if strcmpi(O_T_Type,'object')
        ActionFlag = PickObject(O_T_Location, referenceLocation_wrt_Image, referenceLocation_wrt_Robot, whichObject);
    end
    if strcmpi(O_T_Type,'target')
        DropObject(O_T_Location, referenceLocation_wrt_Image, referenceLocation_wrt_Robot, whichObject);
        ActionFlag = 0;
    end
    
end

function ActionFlag = PickObject(objectLocation, referenceLocation_wrt_Image, referenceLocation_wrt_Robot, whichObject)
Q_OI_TX     = objectLocation(1);
Q_OI_TY     = objectLocation(2);
roll_OI     = -objectLocation(3);

Q_WI_TX     = referenceLocation_wrt_Image(1);
Q_WI_TY     = referenceLocation_wrt_Image(2);

Q_WR_TX     = referenceLocation_wrt_Robot(1);
Q_WR_TY     = referenceLocation_wrt_Robot(2);

% GET Q_OI MATRIX object wrt Image
Q_OI_T = [1 0 0 Q_OI_TX; 0 1 0 Q_OI_TY; 0 0 1 0; 0 0 0 1];
Q_OI_R = [cosd(roll_OI) -sind(roll_OI) 0 0; sind(roll_OI) cosd(roll_OI) 0 0; 0 0 1 0; 0 0 0 1];
Q_OI_P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_OI_Y = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_OI   = Q_OI_T*Q_OI_R*Q_OI_P*Q_OI_Y;

% GET Q_WI MATRIX workspace wrt Image
Q_WI_T = [1 0 0 Q_WI_TX; 0 1 0 Q_WI_TY; 0 0 1 0; 0 0 0 1];
Q_WI_R = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WI_P = [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1];
Q_WI_Y = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WI   = Q_WI_T*Q_WI_R*Q_WI_P*Q_WI_Y;


% GET Q_WR MATRIX workspace wrt Robot base
Q_WR_T = [1 0 0 Q_WR_TX; 0 1 0 Q_WR_TY; 0 0 1 0; 0 0 0 1];
Q_WR_R = [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1];
Q_WR_P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WR_Y = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WR   = Q_WR_T*Q_WR_R*Q_WR_P*Q_WR_Y;

Q_OR = Q_WR * 1/Q_WI * Q_OI;

getStepValues = InverseKinematics(Q_OR);

if length(getStepValues) == 3
    stepValues = getStepValues{3};
    fileID = fopen('../docs/Robix_Instructions.txt','a');
    fprintf(fileID,'\n#Instructions to pick up %s\n', whichObject);
    fprintf(fileID,'Invert all off;\n');
    fprintf(fileID,'move 6 to minpos;\n');
    fprintf(fileID,'move 1 to %d;\n', stepValues(1));
    fprintf(fileID,'move 2 to %d;\n', stepValues(2));
    fprintf(fileID,'move 3 to 0;\n');
    fprintf(fileID,'move 5 to %d;\n', stepValues(5));
    fprintf(fileID,'move 4 to %d;\n', stepValues(4));
    fprintf(fileID,'maxspd 3,4 6;\n');
    fprintf(fileID,'move 3 to %d;\n', stepValues(3));
    fprintf(fileID,'move 6 to 0;\n');
    fprintf(fileID,'move 3 to 0;\n');
    fclose(fileID);
    ActionFlag = 1;
else
    fileID = fopen('../docs/Robix_Instructions.txt','a');
    fprintf(fileID,'\n#object %s out of range\n', whichObject);
    fclose(fileID);
    ActionFlag = 0;
end



end

function DropObject(objectLocation, referenceLocation_wrt_Image, referenceLocation_wrt_Robot, whichObject)
Q_TI_TX     = objectLocation(1);
Q_TI_TY     = objectLocation(2);
roll_OI     = -objectLocation(3);

Q_WI_TX     = referenceLocation_wrt_Image(1);
Q_WI_TY     = referenceLocation_wrt_Image(2);

Q_WR_TX     = referenceLocation_wrt_Robot(1);
Q_WR_TY     = referenceLocation_wrt_Robot(2);

% GET Q_OI MATRIX object wrt Image
Q_TI_T = [1 0 0 Q_TI_TX; 0 1 0 Q_TI_TY; 0 0 1 0; 0 0 0 1];
Q_TI_R = [cosd(roll_OI) -sind(roll_OI) 0 0; sind(roll_OI) cosd(roll_OI) 0 0; 0 0 1 0; 0 0 0 1];
Q_TI_P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_TI_Y = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_TI   = Q_TI_T*Q_TI_R*Q_TI_P*Q_TI_Y;

% GET Q_WI MATRIX workspace wrt Image
Q_WI_T = [1 0 0 Q_WI_TX; 0 1 0 Q_WI_TY; 0 0 1 0; 0 0 0 1];
Q_WI_R = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WI_P = [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1];
Q_WI_Y = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WI   = Q_WI_T*Q_WI_R*Q_WI_P*Q_WI_Y;


% GET Q_WR MATRIX workspace wrt Robot base
Q_WR_T = [1 0 0 Q_WR_TX; 0 1 0 Q_WR_TY; 0 0 1 0; 0 0 0 1];
Q_WR_R = [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1];
Q_WR_P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WR_Y = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_WR   = Q_WR_T*Q_WR_R*Q_WR_P*Q_WR_Y;

Q_TR = Q_WR * 1/Q_WI * Q_TI;

getStepValues = InverseKinematics(Q_TR);

if length(getStepValues) == 3
        stepValues = getStepValues{3};
        fileID = fopen('../docs/Robix_Instructions.txt','a');
        fprintf(fileID,'\n#Instructions to drop %s\n', whichObject);
        fprintf(fileID,'move 1 to %d;\n', stepValues(1));
        fprintf(fileID,'move 2 to %d;\n', stepValues(2));
        fprintf(fileID,'move 4 to %d;\n', stepValues(4));
        fprintf(fileID,'move 5 to %d;\n', stepValues(5));
        fprintf(fileID,'move 3 to %d;\n', stepValues(3));
        fprintf(fileID,'move 6 to minpos;\n');
        fprintf(fileID,'move 3 to 0;\n');
        fclose(fileID);
else
    fileID = fopen('../docs/Robix_Instructions.txt','a');
    fprintf(fileID,'\n#Target out of range\n');
    fprintf(fileID,'move 4 to -818;\n');
    fprintf(fileID,'move 3 to 755;\n');
    fprintf(fileID,'move 6 to minpos;\n');
    fprintf(fileID,'move 3 to 0;\n');
    fclose(fileID);
end
end
