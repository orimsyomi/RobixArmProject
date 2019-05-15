% Read Image
% Workspace_image = imread('Test1.png');
% Workspace_image = imread('Test2.png');
% Workspace_image = imread('Test3.png');
% Workspace_image = imread('Test4.png');
% Workspace_image = imread('Test5.png');
% Workspace_image = imread('Test6.png');
% Workspace_image = imread('Workspace.png');
% Workspace_image = imread('testimg.jpg');
Workspace_image = imread('all outside the workspace.jpg');
% Workspace_image = imread('rectangle.jpg');
% Workspace_image = imread('square.jpg');
% Workspace_image = imread('triangle.jpg');
% Workspace_image = imread('circle.jpg');

% Change to grayscale
Workspace_image_gray = rgb2gray(Workspace_image);

%plot Histogram for observation
% [pixelCount, grayLevels] = imhist(Workspace_image_gray);
% figure(1);
% bar(pixelCount);
% title('Histogram of original image');

%set threshold and convert image to binary image using threshold
thresholdValue = 80;
Workspace_image_binary = Workspace_image_gray < thresholdValue;
% imshow(Workspace_image_binary)

% Label each blob so we can make measurements of it
Workspace_image_binary_labelled = bwlabel(Workspace_image_binary, 8);
coloredLabels = label2rgb (Workspace_image_binary_labelled, 'hsv', 'k', 'shuffle');
% imshow(Workspace_image_binary_labelled, []);hold on;imshow(coloredLabels);

% get all measurement parameters for each blob(shape) in the binary image
blob_Measurements = regionprops(Workspace_image_binary_labelled, Workspace_image_gray, 'all');
number_of_blobs = size(blob_Measurements, 1);
imshow(Workspace_image_gray);
title('Outlines, from bwboundaries()'); 
axis image; % Make sure image is not artificially stretched because of screen's aspect ratio.
hold on;
boundaries = bwboundaries(Workspace_image_binary);
numberOfBoundaries = size(boundaries, 1);
for k = 1 : numberOfBoundaries
    thisBoundary = boundaries{k};
    plot(thisBoundary(:,2), thisBoundary(:,1), 'g', 'LineWidth', 2);
end
hold off;


blobECD = zeros(1, number_of_blobs);
% Print header line in the command window.
fprintf(1,'Blob #      Orientation  Area   Perimeter    Centroid       Diameter   EulerN   MajorAxisLength    MinorAxisLength    FilledArea     BoundingBox3     BoundingBox4\n');
% Loop over all blobs printing their measurements to the command window.
for k = 1 : number_of_blobs           % Loop through all blobs.
    % Find the mean of each blob.  (R2008a has a better way where you can pass the original image
    % directly into regionprops.  The way below works for all versions including earlier versions.)
    thisBlobsPixels = blob_Measurements(k).PixelIdxList;  % Get list of pixels in current blob.
    meanGL = mean(Workspace_image_gray(thisBlobsPixels)); % Find mean intensity (in original image!)
    meanGL2008a = blob_Measurements(k).MeanIntensity; % Mean again, but only for version >= R2008a
    
    blobArea = blob_Measurements(k).Area;       % Get area.
    blobPerimeter = blob_Measurements(k).Perimeter;     % Get perimeter.
    blobCentroid = blob_Measurements(k).Centroid;       % Get centroid one at a time
    blobECD(k) = sqrt(4 * blobArea / pi);                   % Compute ECD - Equivalent Circular Diameter.
    EulerNumber = blob_Measurements(k).EulerNumber;
    MajorAxisLength = blob_Measurements(k).MajorAxisLength;
    MinorAxisLength = blob_Measurements(k).MinorAxisLength;
    FilledArea = blob_Measurements(k).FilledArea;
    BoundingBox3 = blob_Measurements(k).BoundingBox(3);
    BoundingBox4 = blob_Measurements(k).BoundingBox(4);
    Orientation = blob_Measurements(k).Orientation;
    
    fprintf(1,'#%2d %17.1f %11.1f %8.1f %8.1f %8.1f %8.1f    %d      %8.1f         %8.1f        %8.1f          %8.1f        %8.1f\n', k, Orientation, blobArea, blobPerimeter, blobCentroid, blobECD(k), EulerNumber, MajorAxisLength, MinorAxisLength, FilledArea, BoundingBox3, BoundingBox4);
    % Put the "blob number" labels on the "boundaries" grayscale image.
    text(blobCentroid(1), blobCentroid(2), num2str(k), 'FontWeight', 'Bold', 'Color', 'w');
end

pixel_to_cm = 10/72;
% centers = blob_Measurements.Centroid;
figure(2);imshow(Workspace_image_gray); hold on;
% 
% a = length(centers);


% Initialize object cells
rectangle_Objects = [];
rectangle_Targets = [];

square_Objects = [];
square_Targets = [];

circle_Objects = [];
circle_Targets = [];

triangle_Objects = [];
triangle_Targets = [];

classified = [];
objectIds = [];

filledImage = imfill(Workspace_image_binary_labelled, 'holes');
circleMeasurements = regionprops(filledImage, 'perimeter', 'area','centroid');
circles = [];
for i = 1 : length(circleMeasurements)
    tempcentroid = circleMeasurements(i).Centroid;
   
    circularity = circleMeasurements(i).Perimeter^2/(4*pi*circleMeasurements(i).Area);
    if(round(circularity,1) <=1)
        circles = [circles; tempcentroid];
    end
end
numberOfCircles = size(circles);
numberOfCircles = numberOfCircles(1);

% Detect and classify Objects
for i = 1 : length(blob_Measurements)
    objectIds = [objectIds i];
    
    % Classify regionprops parameters for better readability
    Area = blob_Measurements(i).Area;
    BoundingBox3 = blob_Measurements(i).BoundingBox(3);
    BoundingBox4 = blob_Measurements(i).BoundingBox(4);
    Centroid = blob_Measurements(i).Centroid;
    Diameter = blob_Measurements(i).EquivDiameter;
    EulerNumber = blob_Measurements(i).EulerNumber;
    FilledArea = blob_Measurements(i).FilledArea;
    Perimeter = blob_Measurements(i).Perimeter;
    MajorAxisLength = blob_Measurements(i).MajorAxisLength;
    MinorAxisLength = blob_Measurements(i).MinorAxisLength;
    circularity = circleMeasurements(i).Perimeter^2/(4*pi*circleMeasurements(i).Area);
    
    %Extrema Params
    TopLeft         = blob_Measurements(i).Extrema(1,:);
    TopRight        = blob_Measurements(i).Extrema(2,:);
    RightTop        = blob_Measurements(i).Extrema(3,:);
    RightBottom     = blob_Measurements(i).Extrema(4,:);
    BottomRight     = blob_Measurements(i).Extrema(5,:);
    BottomLeft      = blob_Measurements(i).Extrema(6,:);
    LeftBottom      = blob_Measurements(i).Extrema(7,:);
    LeftTop         = blob_Measurements(i).Extrema(8,:);
    
    
%     wh = round(blob_Measurements(i).BoundingBox(3) - blob_Measurements(i).BoundingBox(4),0); %wh represents the x width - y width
%     dw = round(blob_Measurements(i).BoundingBox(3) - blob_Measurements(i).EquivDiameter, 0); % x width - diameter of object
%     tri = round(blob_Measurements(i).BoundingBox(3)*blob_Measurements(i).BoundingBox(4)*(1/2),0); %Base * Hieght /2
    
    %Detect Reference Frame
    if(Area > 680)
        plot(TopRight(1),TopRight(2),'ro');
        text(Centroid(1),Centroid(2),num2str(i),'Color','y');
        x_Reference = TopRight(1)*pixel_to_cm;
        y_Reference = TopRight(2)*pixel_to_cm;
        position_Reference = [x_Reference y_Reference];
        classified = [classified i];
    end
    
    %Detect Rectangle
    if((MajorAxisLength > (MinorAxisLength+10)) && (Area < 680 && Area > 120) && (Perimeter < 200 && Perimeter > 80))
        plot(Centroid(1),Centroid(2),'ro');
        text(Centroid(1),Centroid(2),num2str(i),'Color','y');
        
        %if Object then store position and orientation
        if EulerNumber == 1
            x = Centroid(1)*pixel_to_cm;
            y = Centroid(2)*pixel_to_cm;
            orientation = blob_Measurements(i).Orientation;
            rectangle_Objects = [rectangle_Objects; [x y orientation]];
            classified = [classified i];
            continue
        end
        if EulerNumber == 0
            x = Centroid(1)*pixel_to_cm;
            y = Centroid(2)*pixel_to_cm;
            orientation = blob_Measurements(i).Orientation;
            rectangle_Targets = [rectangle_Targets; [x y orientation]];
            classified = [classified i];
            continue
        end
    end
    
    % DETECT CIRCLE
    for j = 1:numberOfCircles
        circle = circles(j,:);
        criteria = isalmost(circle,Centroid,3);
        if all(criteria) && Perimeter > 30 && FilledArea < 310 && FilledArea > 240
            if EulerNumber == 1
                x = Centroid(1)*pixel_to_cm;
                y = Centroid(2)*pixel_to_cm;
                orientation = blob_Measurements(i).Orientation;
                circle_Objects = [circle_Objects; [x y orientation]];
                classified = [classified i];
                break
            end
            if EulerNumber == 0
                x = Centroid(1)*pixel_to_cm;
                y = Centroid(2)*pixel_to_cm;
                orientation = blob_Measurements(i).Orientation;
                circle_Targets = [circle_Targets; [x y orientation]];
                classified = [classified i];
                break
            end
        end
    end
    
%         % DETECT TRIANGLE
    ratio = (BoundingBox3*BoundingBox4)/FilledArea;
    if (ratio > 1.5) && FilledArea < 300 && FilledArea > 120 && circularity > 1.06
        if EulerNumber == 1
                x = Centroid(1)*pixel_to_cm;
                y = Centroid(2)*pixel_to_cm;
                orientation = blob_Measurements(i).Orientation;
                triangle_Objects = [triangle_Objects; [x y orientation]];
                classified = [classified i];
                continue
            end
            if EulerNumber == 0
                x = Centroid(1)*pixel_to_cm;
                y = Centroid(2)*pixel_to_cm;
                orientation = blob_Measurements(i).Orientation;
                triangle_Targets = [triangle_Targets; [x y orientation]];
                classified = [classified i];
                continue
            end
    end
end
unclassified = setdiff(objectIds, classified);

for i=1:length(unclassified)
    blobId = unclassified(i);
    EulerNumber = blob_Measurements(blobId).EulerNumber;
    Area = blob_Measurements(blobId).Area;
    Centroid = blob_Measurements(blobId).Centroid;
    orientation = blob_Measurements(blobId).Orientation;
    
    %if Object then store position and orientation
        if EulerNumber == 1 && Area > 100
            x = Centroid(1)*pixel_to_cm;
            y = Centroid(2)*pixel_to_cm;
            square_Objects = [square_Objects; [x y orientation]];
            continue
        end
        if EulerNumber == 0 && Area > 100
            x = Centroid(1)*pixel_to_cm;
            y = Centroid(2)*pixel_to_cm;
            square_Targets = [square_Targets; [x y orientation]];
            continue
        end
end

% Move Rectangles If any
numberOfObjects = size(rectangle_Objects);
numberOfObjects = numberOfObjects(1);
numberOfTargets = size(rectangle_Targets);
numberOfTargets = numberOfTargets(1);
if numberOfObjects ~= 0
    for i=1:numberOfObjects
        if  numberOfTargets >= i
            objectPicked = performAction(rectangle_Objects(i,:), position_Reference, 'object', 'rectangle');
            if objectPicked == 1
                performAction(rectangle_Targets(i,:), position_Reference, 'target', 'rectangle');
            end
        end
    end
end

% Move Circles If any
numberOfObjects = size(circle_Objects);
numberOfObjects = numberOfObjects(1);
numberOfTargets = size(circle_Targets);
numberOfTargets = numberOfTargets(1);
if numberOfObjects ~= 0
    for i=1:numberOfObjects
        if  numberOfTargets >= i
            objectPicked = performAction(circle_Objects(i,:), position_Reference, 'object', 'circle');
            if objectPicked == 1
                performAction(circle_Targets(i,:), position_Reference, 'target', 'circle');
            end
        end
    end
end

% Move Triangles If any
numberOfObjects = size(triangle_Objects);
numberOfObjects = numberOfObjects(1);
numberOfTargets = size(triangle_Targets);
numberOfTargets = numberOfTargets(1);
if numberOfObjects ~= 0
    for i=1:numberOfObjects
        if  numberOfTargets >= i
            objectPicked = performAction(triangle_Objects(i,:), position_Reference, 'object', 'triangle');
            if objectPicked == 1
                performAction(triangle_Targets(i,:), position_Reference, 'target', 'triangle');
            end
        end
    end
end

% Move Squares If any
numberOfObjects = size(square_Objects);
numberOfObjects = numberOfObjects(1);
numberOfTargets = size(square_Targets);
numberOfTargets = numberOfTargets(1);
if numberOfObjects ~= 0
    for i=1:numberOfObjects
        if  numberOfTargets >= i
            objectPicked = performAction(square_Objects(i,:), position_Reference, 'object', 'square');
            if objectPicked == 1
                performAction(square_Targets(i,:), position_Reference, 'target', 'square');
            end
        end
    end
end

fileID = fopen('Robix Instructions.txt','a');
fprintf(fileID,'\n#End of action sequence\n');
fprintf(fileID,'move all to 0;\n');
fclose(fileID);