%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--Point Tracking and 3D coordinate Calculation From Stereo Cameras---%
%-------------Input: Stereo images & intrinsic parameters-------------%
%-------------Output: 3D point coordinates----------------------------%
%-------------Creator: Billy ZHONG------------------------------------%
%-------------Last Updated: 08/01/2018--------------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function: The algorithm aims to use the user-defined tracking points
%in stereo images to calculate the 3D corrdinates of the world feature
%points in real time.

%Preparation: Calibrate your stereo camera using MATLAB stereo calibrator,
%then save the calibration result to .mat file. If you want to use existing
%videos, please prepare two videos with same video size and framerate.
%There are two .m files, the StereoTracker.m class file and
%StereoTracking.m main file. Put all these files together in one folder.

%Starting program: Two windows are created with video images. Use
%left-clicking of your mouse to select the projected world feature points
%you want to track on respective images, left image first and then right.
%Avoid clicking the same figure twice or there will be errors. 3D 
%coordinates of these pair of selected points are then computed.
%please refer to the member "POINTS3D" in your declared "StereoTracker"
%class to obtain the computed 3D coordinates.

%Ending Program: Please press q when either window is on top to terminate
%the program normally.

clc;
clear all;

warning('off');

%Create two figure window to show content
fig1 = figure('Position', [100, 200, 640, 480]);
fig2 = figure('Position', [1000, 200, 640, 480]);

%Enable following when using live USB camera
% vid1 = videoinput('winvideo', 1, 'YUY2_640x480');
% triggerconfig(vid1, 'manual');
% set(vid1, 'ReturnedColorspace', 'rgb');
% vid2 = videoinput('winvideo', 2, 'YUY2_640x480');
% triggerconfig(vid2, 'manual');
% set(vid2, 'ReturnedColorspace', 'rgb');
% start([vid1, vid2]);
%Enable following when using existing video pairs
mov1 = VideoReader('StereoL.mp4');
mov2 = VideoReader('StereoR.mp4');

%To track the 3D features observed by stereo cameras
%Input: number of 3D tracking points, two figure handles, to track/untrack
T = StereoTracker(fig1, fig2, 'StereoParameters');

while true
        
    %Enable mouse left-clicking callback in both images
    set(fig1, 'WindowButtonDownFcn', @T.StereoMouseCallbackLC);
    set(fig2, 'WindowButtonDownFcn', @T.StereoMouseCallbackLC);
    
    %% Frame grabbing
    %Enable following when using live USB camera
    % img_l = getsnapshot(vid1);
    % img_r = getsnapshot(vid2);
    %Enable following when using existing video pairs
    vid1 = readFrame(mov1);
    vid2 = readFrame(mov2);
    img_l = vid1;
    img_r = vid2;
  
    %% Point tracking and 3D coordinate calculation
    %To executive image-based point tracking in respective images
    T.StereoFeatureTracking(img_l,img_r);
    %Store the computed 3D point coordinates to your variables as follows:
    %Suppose there are n pairs of tracking points selected from stereo images:
    Computed3DPointsCoordinates = T.POINTS3D; %should return a nx3 matrix
    Computed3DPointsSize = T.POINTS3D_SIZE; %should return a scalar n
    
    %% Processing and displaying left and right image
    set(0, 'CurrentFigure', fig1);
    imshow(img_l);
    hold on;
    textt = ['Pair of points: ', num2str(Computed3DPointsSize)];
    text(50,50,textt,'Color','w','FontSize',30);
    %To plot the tracking points on figure
    T.StereoTrackingPlottingLImage();
    
    set(0, 'CurrentFigure', fig2);
    imshow(img_r);
    hold on;
    %To plot the tracking points on figure
    T.StereoTrackingPlottingRImage();
    
    %% Keyboard input callback for quitting program
    if strcmpi(get(gcf,'CurrentKey'),'q')
        %press q to exit the program normally
        close all;
        break;
    end
    
    %% needed for updating figure content
    pause(.01); 

end