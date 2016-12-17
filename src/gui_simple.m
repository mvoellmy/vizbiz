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

clc;
addpath(genpath('../../src/helpers'));
addpath(genpath('../../src/testing'));
addpath(genpath('../../src/visualization'));
rng(1); % fix random seed

% load parameter struct
fprintf('load parameter struct...\n');
params = loadParameters(true);
handles.params = params;

% Choose default command line output for gui_simple
handles.output = hObject;

% Update handles structure
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

runVOPipeline(handles.params,handles);



% --- Executes on button press in radio_use_bootstrapping.
function radio_use_bootstrapping_Callback(hObject, eventdata, handles)
% hObject    handle to radio_use_bootstrapping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.params.auto_bootstrap = get(hObject,'Value');


% --- Executes on button press in radio_use_BA.
function radio_use_BA_Callback(hObject, eventdata, handles)
% hObject    handle to radio_use_BA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.params.init.use_BA = get(hObject,'Value');


% --- Executes on button press in radio_run_continuous.
function radio_run_continuous_Callback(hObject, eventdata, handles)
% hObject    handle to radio_run_continuous (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.params.run_continous = get(hObject,'Value');


% --- Executes on selection change in popup_dataset.
function popup_dataset_Callback(hObject, eventdata, handles)
% hObject    handle to popup_dataset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popup_dataset contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popup_dataset



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


% --- Executes on button press in push_clear.
function push_clear_Callback(hObject, eventdata, handles)
% hObject    handle to push_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc;
clearvars -except handles hObject eventdata;

fprintf('reload parameter struct...\n');
params = loadParameters(true);
handles.params = params;

% Choose default command line output for gui_simple
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
