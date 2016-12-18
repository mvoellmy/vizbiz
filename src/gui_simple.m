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
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_simple (see VARARGIN)

% close figures except gui
set(handles.main_figure, 'HandleVisibility', 'off');
close all;
set(handles.main_figure, 'HandleVisibility', 'on');

% clear variables
clearvars -except handles hObject eventdata;

% clear command line
clc;

% prepare console output
handles.sTringToDisplay = '>>';

% prepare axes
iptsetpref('ImshowInitialMagnification', 'fit')
axes(handles.ax_current_frame);
hold off;
axis off;
axes(handles.ax_trajectory);
handles.plot_trajectory = plot(0,0,'.-');
axis off;
guidata(hObject, handles);

fprintf('starting VO pipeline...\n');

addpath(genpath('./helpers/'));
addpath(genpath('./testing/'));
addpath(genpath('./visualization/'));
rng(1); % fix random seed

% place gui
movegui(hObject, 'north');

% load parameter struct
handles = guidata(hObject);  % Care for the newest version explicitly!
push_reset_Callback(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

handles.output = hObject;
guidata(hObject, handles);

% UIWAIT makes gui_simple wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_simple_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in push_run.
function push_run_Callback(hObject, eventdata, handles)
% hObject    handle to push_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc;

handles = guidata(hObject);  % Care for the newest version explicitly!
update_parameters(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

runVOPipeline(handles.params, handles);

% --- Executes on button press in radio_use_bootstrapping.
function radio_use_bootstrapping_Callback(hObject, eventdata, handles)
% hObject    handle to radio_use_bootstrapping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.params.auto_bootstrap = get(hObject,'Value');
guidata(hObject, handles);

% --- Executes on button press in radio_use_BA.
function radio_use_BA_Callback(hObject, eventdata, handles)
% hObject    handle to radio_use_BA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.params.init.use_BA = get(hObject,'Value');
guidata(hObject, handles);

% --- Executes on button press in radio_run_continuous.
function radio_run_continuous_Callback(hObject, eventdata, handles)
% hObject    handle to radio_run_continuous (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.params.run_continous = get(hObject,'Value');
guidata(hObject, handles);

% --- Executes on selection change in popup_dataset.
function popup_dataset_Callback(hObject, eventdata, handles)
% hObject    handle to popup_dataset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popup_dataset contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popup_dataset

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

% --- Executes during object creation, after setting all properties.
function popup_dataset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popup_dataset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in push_about.
function push_about_Callback(hObject, eventdata, handles)
% hObject    handle to push_about (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

msgbox({sprintf(['About\n',...
                 '...',...
                 'Authors: Pascal Buholzer, Fabio Dubois, Miro Voellmy, Milan Irokese\n'])});


% --- Executes on button press in push_about.
function push_abort_Callback(hObject, eventdata, handles)
% hObject    handle to push_about (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

error('User requested abortion.');


% --- Executes on button press in push_clear.
function push_reset_Callback(hObject, eventdata, handles)
% hObject    handle to push_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clearvars -except handles hObject eventdata;

% reset parameters
fprintf('load default parameters...\n');
handles = guidata(hObject);  % Care for the newest version explicitly!
reset_parameters(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

% reset dataset dropdown

% reset buttons to false
set(handles.radio_use_bootstrapping,'Value',0);
set(handles.radio_use_BA,'Value',0);
set(handles.radio_run_continuous,'Value',0);

guidata(hObject,handles);

function reset_parameters(hObject, eventdata, handles)

axesHandlesToChildObjects = findobj(handles.ax_current_frame, 'Type', 'image');
if ~isempty(axesHandlesToChildObjects)
	delete(axesHandlesToChildObjects);
end

% prepare console output
handles.sTringToDisplay = '>>';

handles.params = loadParameters(true);
guidata(hObject, handles);

function update_parameters(hObject, eventdata, handles)
fprintf('update parameters...\n');

handles = guidata(hObject);  % Care for the newest version explicitly!
reset_parameters(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

% update dropdown input
popup_dataset_Callback(hObject, eventdata, handles);
handles = guidata(hObject);  % Get the version updated!

% update radio inputs
handles.params.auto_bootstrap = get(handles.radio_use_bootstrapping,'Value');
handles.params.init.use_BA = get(handles.radio_use_BA,'Value');
handles.params.run_continous = get(handles.radio_run_continuous,'Value');

guidata(hObject, handles);
