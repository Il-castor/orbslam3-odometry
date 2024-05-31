
[intrinsicMatrix1,distortionCoefficients1,intrinsicMatrix2, ...
distortionCoefficients2,rotationOfCamera2,translationOfCamera2] =...
stereoParametersToOpenCV(stereoParams);


% Open a file for writing
fileID = fopen('orbslam_parameters.txt', 'w');

fprintf(fileID, 'Camera.type: "Rectified"\n\n');


imageSize = stereoParams.CameraParameters1.ImageSize;
imageWidth = imageSize(2); % Width is the second element
imageHeight = imageSize(1); % Height is the first element

fprintf(fileID, 'Camera.width: %d\n', imageWidth);
fprintf(fileID, 'Camera.height: %d\n\n', imageHeight);

fprintf(fileID, 'Camera1.fx: %.12f\n', intrinsicMatrix1(1));
fprintf(fileID, 'Camera1.fy: %.12f\n', intrinsicMatrix1(5));
fprintf(fileID, 'Camera1.cx: %.12f\n', intrinsicMatrix1(7));
fprintf(fileID, 'Camera1.cy: %.12f\n', intrinsicMatrix1(8));

fprintf(fileID, '\n');

fprintf(fileID, 'Camera1.k1: %.12f\n', distortionCoefficients1(1));
fprintf(fileID, 'Camera1.k2: %.12f\n', distortionCoefficients1(2));
fprintf(fileID, 'Camera1.p1: %.12f\n', distortionCoefficients1(3));
fprintf(fileID, 'Camera1.p2: %.12f\n', distortionCoefficients1(4));

fprintf(fileID, '\n');


fprintf(fileID, 'Camera2.fx: %.12f\n', intrinsicMatrix2(1));
fprintf(fileID, 'Camera2.fy: %.12f\n', intrinsicMatrix2(5));
fprintf(fileID, 'Camera2.cx: %.12f\n', intrinsicMatrix2(7));
fprintf(fileID, 'Camera2.cy: %.12f\n', intrinsicMatrix2(8));

fprintf(fileID, '\n');

fprintf(fileID, 'Camera2.k1: %.12f\n', distortionCoefficients2(1));
fprintf(fileID, 'Camera2.k2: %.12f\n', distortionCoefficients2(2));
fprintf(fileID, 'Camera2.p1: %.12f\n', distortionCoefficients2(3));
fprintf(fileID, 'Camera2.p2: %.12f\n', distortionCoefficients2(4));

fprintf(fileID, '\n');
baseline = norm(stereoParams.TranslationOfCamera2);
baseline = baseline / 1000; 
fprintf(fileID, 'Stereo.b: %.5f\n', baseline);

% Create the transformation matrix
T_c1_c2 = eye(4); % Initialize as identity matrix
T_c1_c2(1:3, 1:3) = rotationOfCamera2; % Assign rotation
T_c1_c2(1:3, 4) = translationOfCamera2; % Assign translation

% Print the transformation matrix to the file
fprintf(fileID, 'Stereo.T_c1_c2: !!opencv-matrix\n');
fprintf(fileID, '  rows: 4\n');
fprintf(fileID, '  cols: 4\n');
fprintf(fileID, '  dt: f\n');
fprintf(fileID, '  data: [');
for i = 1:size(T_c1_c2, 1)
    for j = 1:size(T_c1_c2, 2)
        fprintf(fileID, '%.12f', T_c1_c2(i, j));
        if j < size(T_c1_c2, 2)
            fprintf(fileID, ', ');
        end
    end
    if i < size(T_c1_c2, 1)
        fprintf(fileID, ',\n         ');
    else
        fprintf(fileID, ']\n');
    end
end

% Close the file
fclose(fileID);
