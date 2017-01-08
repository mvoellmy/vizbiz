function varargout = gui_simple(varargin)
% GUI_SIMPLE MATLAB code for gui_simple.fig
%      GUI_SIMPLE, by itself, creates a new GUI_SIMPLE or raises the existing
%      singleton*.
%
%      H = GUI_SIMPLE returns the handle to a new GUI_SIMPLE or the handle to
%      the existing singleton*.
%
%      GUI_SIMPLE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_SIMPLE.M with the given input arguments.
%
%      GUI_SIMPLE('Property','Value',...) creates a new GUI_SIMPLE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_simple_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_simple_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_simple

% Last Modified by GUIDE v2.5 17-Dec-2016 16:07:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_simple_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_simple_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before gui_simple is made visible.
function gui_simple_OpeningFcn(hObject, eventdata, handles, varargin)

% close figures except gui
set(handles.main_figure, 'HandleVisibility', 'off');
close all;
set(handles.main_figure, 'HandleVisibility', 'on');

% clear variables
clearvars -except handles hObject eventdata;

% clear command line
clc;

% prepare axes
iptsetpref('ImshowInitialMagnification', 'fit')
clearAxes(hObject, eventdata, handles);

% add search paths
addpath(genpath('./helpers/'));
addpath(genpath('./testing/'));
addpath(genpath('./visualization/'));

% place gui
movegui(hObject, 'onscreen');

% load parameter struct
handles = guidata(hObject);  % Care for the newest version explicitly!
push_reset_Callback(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

handles.output = hObject;
guidata(hObject, handles);

% UIWAIT makes gui_simple wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function varargout = gui_simple_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function push_run_Callback(hObject, eventdata, handles)

clc;
clearAxes(hObject, eventdata, handles);

handles = guidata(hObject);
update_parameters(hObject, eventdata, handles);
handles = guidata(hObject);

runVOPipeline(handles.params, handles);

function radio_use_bootstrapping_Callback(hObject, eventdata, handles)

handles.params.auto_bootstrap = get(hObject,'Value');
guidata(hObject, handles);

function radio_use_BA_Callback(hObject, eventdata, handles)

handles.params.cont.use_BA = get(hObject,'Value');
guidata(hObject, handles);

function radio_run_continuous_Callback(hObject, eventdata, handles)

handles.params.run_continous = get(hObject,'Value');
guidata(hObject, handles);

function radio_all_features_Callback(hObject, eventdata, handles)

handles.params.gui.show_all_features = get(hObject,'Value');
guidata(hObject, handles);

function radio_inlier_features_Callback(hObject, eventdata, handles)

handles.params.gui.show_inlier_features = get(hObject,'Value');
guidata(hObject, handles);

function radio_triang_features_Callback(hObject, eventdata, handles)

handles.params.gui.show_triang_features = get(hObject,'Value');
guidata(hObject, handles);

function popup_dataset_Callback(hObject, eventdata, handles)

val = get(handles.popup_dataset,'Value');
str = get(handles.popup_dataset,'String');
switch str{val}
    case 'Kitti'
        handles.params.ds = 0;
    case 'Malaga'
        handles.params.ds = 1;
    case 'Parking'
        handles.params.ds = 2;
end

guidata(hObject, handles);

function popup_dataset_CreateFcn(hObject, eventdata, handles)

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function push_about_Callback(hObject, eventdata, handles)

file = dir('gui_simple.m');
msgbox({sprintf(['Authors: Pascal Buholzer, Fabio Dubois, Miro Voellmy, Milan Schilling\n\n',...
                 'Last modified: ', file.date, '\n\n'])});

function push_reset_Callback(hObject, eventdata, handles)
% hObject    handle to push_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clearvars -except handles hObject eventdata;

% reset parameters
handles = guidata(hObject);  % Care for the newest version explicitly!
reset_parameters(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

% clear console
set(handles.listbox_console,'String','');

% reset performance metrics
set(handles.text_RT_value,'String','00.00');
set(handles.text_value_tracked,'String','0');

% reset track bar
handles = guidata(hObject);
reset_bar(hObject, eventdata, handles);
handles = guidata(hObject);

guidata(hObject,handles);

function clearAxes(hObject, eventdata, handles)

% clear tracked axes
axes(handles.ax_tracked);
cla(handles.ax_tracked);
handles.plot_bar = plot(0,0,'-','Color',[0.5 0.5 0.5]);
reset_bar(hObject, eventdata, handles);

% clear image axes
axes(handles.ax_current_frame);
cla(handles.ax_current_frame);
hold off;
axis off;

% clear trajectory axes
axes(handles.ax_trajectory);
cla(handles.ax_trajectory);
handles.plot_gt = plot(0, 0, 'k-');
hold on;
handles.plot_trajectory = plot(0, 0, '.-');
hold on;
handles.plot_local_cloud = plot(0, 0, 'k.');
hold off;

axis equal;
axis square;
axis off;

guidata(hObject, handles);

function reset_parameters(hObject, eventdata, handles)

axesHandlesToChildObjects = findobj(handles.ax_current_frame, 'Type', 'image');
if ~isempty(axesHandlesToChildObjects)
	delete(axesHandlesToChildObjects);
end

rng(1); % fix random seed

% prepare console output
handles.console_string = ' ';

handles.params = loadParameters();
% set general parameters
handles.params.perf.profiling = false;
handles.params.compare_against_groundthruth = true;
handles.params.run_on_first_x_images = 0; % 0 for all images
handles.params.show_map_and_cams = true;
handles.params.through_gui = true;
% disable debug figures
handles.params.boot.figures = false;
handles.params.init.figures = false;
handles.params.cont.figures = false;

guidata(hObject, handles);

function reset_bar(hObject, eventdata, handles)

axes(handles.ax_tracked);
handles.plot_bar.XData = 0;
handles.plot_bar.YData = 0;
handles.plot_bar.LineWidth = 2;

xlim([0 200]); % todo: parametrize?
axis equal;
axis off;

function update_parameters(hObject, eventdata, handles)

handles = guidata(hObject);  % Care for the newest version explicitly!
reset_parameters(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

% update dropdown input
popup_dataset_Callback(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

% update radio inputs
handles.params.auto_bootstrap = get(handles.radio_use_bootstrapping,'Value');
handles.params.cont.use_BA = get(handles.radio_use_BA,'Value');
handles.params.run_continous = get(handles.radio_run_continuous,'Value');
handles.params.gui.show_all_features = get(handles.radio_all_features,'Value');
handles.params.gui.show_inlier_features = get(handles.radio_inlier_features,'Value');
handles.params.gui.show_triang_features = get(handles.radio_triang_features,'Value');

guidata(hObject, handles);
