%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%      Feature Tracking Using Stereo Vision   %%%%%%%%%%%%%%%
%%%%%%%%
%%%%%%%%      Input: number of 3D tracking points,
%%%%%%%%             two figure handles for display,
%%%%%%%%             flag to indicate whether to track the points or not
%%%%%%%%
%%%%%%%%      Output: feature point coord. in 3D camera frame
%%%%%%%%
%%%%%%%%
%%%%%%%%

classdef StereoTracker < handle
    
    properties (SetAccess = public)
        %These are variables that needed to be called in main program
        TRACK
        POINTS_L_OK
        POINTS_R_OK
        POINTS_L
        POINTS_R
        POINTS_L_SIZE
        POINTS_R_SIZE
        NUM_OF_CLICKED_L
        NUM_OF_CLICKED_R
        POINTS3D
        POINTS3D_SIZE
        StereoIntrinsic
    end
    
    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
    end
    
    % only this class methods can view/modify
    properties (SetAccess = private)
        %If set private, they can't be seen from class object
        pointTracker_l
        pointTracker_r
        POINTS_INIT_LALL
        POINTS_INIT_RALL
        fig1
        fig2
        i_O
        i_O_x
        i_O_y
        i_O_z
    end
    
    methods
        
        %The constructing function only contains data initialisation
        function self = StereoTracker(fig1_, fig2_, stereoParamFilename)
            %read stereo-cam parameters from .mat file
            self.StereoIntrinsic = load(stereoParamFilename);
            self.fig1 = fig1_;
            self.fig2 = fig2_;
            %Do variable initialisation
            self.init();

        end
        
        
        function delete(self)
        end
        
        
        %Initialisation for variables and mouse callback when newly defined
        function init(self)
            
            %Internal storage and flags for selecting and tracking points
            self.POINTS_L = [];
            self.POINTS_R = [];
            self.POINTS_L_OK = false;
            self.POINTS_R_OK = false;
            self.POINTS_L_SIZE = 0;
            self.POINTS_R_SIZE = 0;
            self.NUM_OF_CLICKED_L = 0;
            self.NUM_OF_CLICKED_R = 0;
            self.POINTS_INIT_LALL = false;
            self.POINTS_INIT_RALL = false;
            
            %To store the computed 3D coordinates
            self.POINTS3D = [];
            self.POINTS3D_SIZE = 0;                       
        end
        
        
        %% Mouse left-clicking callback handle and triggered event
        function StereoMouseCallbackLC(self,h,~)
            %If clicked on left figure, then store the points in POINTS_L
            if gcf == self.fig1
                self.NUM_OF_CLICKED_L = self.NUM_OF_CLICKED_L + 1;
                clicked = get(self.fig1.CurrentAxes, 'CurrentPoint');
                self.POINTS_L(end+1,:) = [clicked(1,1), clicked(1,2)];
                self.POINTS_L_OK = true;
                self.POINTS_INIT_LALL = true;
            %If clicked on right figure, then store the points in POINTS_R
            elseif gcf == self.fig2
                self.NUM_OF_CLICKED_R = self.NUM_OF_CLICKED_R + 1;
                clicked = get(self.fig2.CurrentAxes, 'CurrentPoint');
                self.POINTS_R(end+1,:) = [clicked(1,1), clicked(1,2)];
                self.POINTS_R_OK = true;
                self.POINTS_INIT_RALL = true;
            end
        end
        
        
        %% Initialise PointTracker if tracking point initialised
        function [pointTrackerObject] = StereoPointTrackingInit(self, points_, img)
            % Create a point tracker and enable the bidirectional error constraint to
            % make it more robust in the presence of noise and clutter.
            pointTrackerObject = vision.PointTracker('MaxBidirectionalError', 20); %the larger the easier to track
            % Initialize the tracker with the initial point locations and the initial
            % video frame.
            initialize(pointTrackerObject, points_, img);
        end
        
        
        %% Track the point in consecutive frame after tracking intialisation
        function [tracking_point, tracking_point_size] = StereoPointTracking(self, pointTracker_, img)
            % Track the points. Note that some points may be lost.
            % Tracking points stored in Soft_point
            [Soft_point, isFound] = step(pointTracker_, img);
            visiblePoints = Soft_point(isFound, :);
            if size(visiblePoints,1) > 0
                tracking_point = visiblePoints;
                tracking_point_size = size(visiblePoints,1);
            else
                tracking_point(1,:) = [1,1];
                tracking_point_size = 0;
            end
        end
        
        %% Feature point initialisation and tracking
        function StereoFeatureTracking(self,img_ll,img_rr)
            %check if the tracking point on left image is initialised
            %if yes, then store the mouse-clicked init point to tracker
            if self.POINTS_L_OK == true
                if self.POINTS_INIT_LALL == true
                    %If point initialised by mouse clicking, then store it in tracker class for tracking
                    self.pointTracker_l = self.StereoPointTrackingInit(self.POINTS_L, img_ll);
                    self.POINTS_INIT_LALL = false;
                end
                %If tracking point already initialised and enable tracking, then just track it
                [self.POINTS_L, self.POINTS_L_SIZE] = self.StereoPointTracking(self.pointTracker_l, img_ll);
                %should be still be given values for triangulation
                self.POINTS_L_SIZE = size(self.POINTS_L, 1);
            end
            
            %check if the tracking point on right image is initialised
            %if yes, then store the mouse-clicked init point to tracker
            if self.POINTS_R_OK == true
                if self.POINTS_INIT_RALL == true
                    %If point initialised by mouse clicking, then store it in tracker class for tracking
                    self.pointTracker_r = self.StereoPointTrackingInit(self.POINTS_R, img_rr);
                    self.POINTS_INIT_RALL = false;
                end
                %If tracking point already initialised and enable tracking, then just track it
                [self.POINTS_R, self.POINTS_R_SIZE] = self.StereoPointTracking(self.pointTracker_r, img_rr);
                %should be still be given values for triangulation
                self.POINTS_R_SIZE = size(self.POINTS_R, 1);
            end
            
            %calculate 3D coordinate if point detected from both left and
            %right image
            if self.POINTS_L_OK == true && self.POINTS_R_OK == true
            %Calculate the 3D coordinates if pairs of features selected
                self.POINTS3D = [];
                for i=1:self.POINTS_R_SIZE
                    self.POINTS3D(end+1,:) = triangulate(self.POINTS_L(i,:), self.POINTS_R(i,:), self.StereoIntrinsic.stereoParams);
                end
                self.POINTS3D_SIZE = size(self.POINTS3D,1);
            end
            
        end
            
        
        %% Plotting tracking points and other stuff (left image)
        function StereoTrackingPlottingLImage(self)
            if self.NUM_OF_CLICKED_L > 0
                plot(self.POINTS_L(:,1), self.POINTS_L(:,2), 'ok','MarkerSize',10,'MarkerFaceColor', [1, 0, 0]);
            end
        end
        
        
        %% Plotting tracking point and other stuff (right image)
        function StereoTrackingPlottingRImage(self)
            if self.NUM_OF_CLICKED_R > 0
                plot(self.POINTS_R(:,1), self.POINTS_R(:,2), 'ok','MarkerSize',10,'MarkerFaceColor', [1,0,0]);
            end
        end
        
    end
end