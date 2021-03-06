function varargout = GUI_ver1(varargin)
% GUI_VER1 MATLAB code for GUI_ver1.fig
%      GUI_VER1, by itself, creates a new GUI_VER1 or raises the existing
%      singleton*.
%
%      H = GUI_VER1 returns the handle to a new GUI_VER1 or the handle to
%      the existing singleton*.
%
%      GUI_VER1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_VER1.M with the given input arguments.
%
%      GUI_VER1('Property','Value',...) creates a new GUI_VER1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_ver1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_ver1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_ver1

% Last Modified by GUIDE v2.5 25-Aug-2020 10:59:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @GUI_ver1_OpeningFcn, ...
    'gui_OutputFcn',  @GUI_ver1_OutputFcn, ...
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


% --- Executes just before GUI_ver1 is made visible.
function GUI_ver1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_ver1 (see VARARGIN)

% Choose default command line output for GUI_ver1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% BUNLARI BIZ YAZDIK
newobjs = instrfind;
%fclose(newobjs);
% UIWAIT makes GUI_ver1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_ver1_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)

handles.output = hObject;
global vidDevice
handles.vidDevice = imaq.VideoDevice('gige',1 ,'Mono8','ROI', [1 1 640 480], ...
    'ReturnedColorSpace', 'rgb');
guidata(hObject, handles);
vidDevice =handles.vidDevice;
vidInfo = imaqhwinfo(vidDevice);
hVideoIn = vision.VideoPlayer('Name', 'Final Video', ...
    'Position', [100 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);
nFrame = 0;
global flag
flag = true;
global foto
foto = false;
tstart = tic;

ax = handles.axes1;
while(flag)
    rgbFrame = step(vidDevice); % Acquire single frame
    image(rgbFrame, 'Parent', ax);
    set(ax,'Visible', 'off');
    pause(1/1000);
    if foto
        x = get(handles.uibuttongroup2,'SelectedObject');
        text1=sprintf('%.1f volt', 5);
        if strcmp(x.String,'ON')
            rgbFrame = insertText(rgbFrame, [5 5], text1,'FontSize',6);
        end
        desktop=winqueryreg('HKEY_CURRENT_USER', 'Software\Microsoft\Windows\CurrentVersion\Explorer\Shell Folders', 'Desktop');
        [file,path] = uiputfile([desktop '\image1.png']);
        imwrite(rgbFrame,[path,file])
        pause(2)
        foto=false;
        
    end
    
    nFrame = nFrame+1;
end
tend = toc(tstart);

% handles.text2.String = [num2str(tend) '/n' num2str(nFrame/tend)];
release(hVideoIn);
release(vidDevice);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global flag
flag = false;


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
global foto
foto = true;


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)

vidDevice = handles.vidDevice;
user_text = get(handles.edit2,'string');
[nwname, desktp] = check_adres();
[file,path] = uiputfile([desktp '\' nwname]);
selpath =[path,file];
path2 = selpath(1:end-7);
mkdir(path2);

user_text=[user_text,';'];
ind = strfind(user_text,';');
nwmatrix=[];
if any(ind)
    son =1;
    for i = 1:length(ind)
        out = sscanf(user_text(son:ind(i)-1), '%f:%f:%f');
        nwmatrix = [nwmatrix out(1):out(2):out(3)];
        son = ind(i)+1;
    end
end
ax = handles.axes1;

for j = 1:length(nwmatrix)

    ser=handles.ser;
    pause(0.05)
    disp(nwmatrix(j))
    fwrite(ser,nwmatrix(j)*10);
    
    [display_voltage,display_current] = serial_read(handles);
    while isnan(display_voltage)||abs(display_voltage-nwmatrix(j))>0.1
       [display_voltage,display_current] = serial_read(handles);
    end
    text1=sprintf('%.3f volt',display_voltage);
    x = get(handles.uibuttongroup2,'SelectedObject');
    rgbFrame = step(vidDevice);
    if strcmp(x.String,'ON')
        rgbFrame = insertText(rgbFrame, [5 5], text1,'FontSize',6);
    end
    image(rgbFrame, 'Parent', ax);
    set(ax,'Visible', 'off');
    
    baseFileName = sprintf('image %.1f volt.png',nwmatrix(j)); % Whatever....
    fullFileName = [path2,'\', baseFileName]; % to constitute the full path for saving                        imwrite(frame, fullFileName);
    imwrite(rgbFrame, fullFileName);
    pause(1) % wait more for the next step
    
    disp(display_voltage);
    disp(display_current);
    %/////////arduino
end


function edit1_KeyPressFcn(hObject, eventdata, handles)
set(handles.edit1,'String','')


function [newfoldername, desktop]= check_adres()
desktop=winqueryreg('HKEY_CURRENT_USER', 'Software\Microsoft\Windows\CurrentVersion\Explorer\Shell Folders', 'Desktop');
dir(desktop);
a = dir(fullfile([desktop '\mynewfolder', '*']));
if ~isempty(a)
    fStr     = lower(sprintf('%s*', a.name));
    fNum     = sscanf(fStr,['mynewfolder', '%d', '*']);
    if isempty(fNum), fStr = fStr(13:length(fStr));
        if isempty(fStr), fNum = 0;
        else, fNum = sscanf(fStr,['mynewfolder', '%d', '*']);
        end
    end
    newNum   = max(fNum) + 1;
else
    newNum = [];
end
newfoldername = ['mynewfolder',num2str(newNum)];


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
vidDevice = handles.vidDevice;
global frame1
frame1=step(vidDevice);


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
vidDevice = handles.vidDevice;
global frame1
global frame2
frame2=step(vidDevice);
format long
GPP=(rgb2gray(frame1));
GS=(rgb2gray(frame2));
result=(double(GS)./double(GPP));
[file,path] = uiputfile('image1.png');
imwrite(result,[path,file])


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
contents=cellstr(get(hObject,'string'));
pop_choice=contents{get(hObject,'Value')};
global ser
handles.ser=serial(pop_choice,'baudrate',9600);
ser=handles.ser;
guidata(hObject, handles);
try
    fopen(ser);
catch e
    msgbox('CommunicationERROR!!!')
    newobjs = instrfind;
    fclose(newobjs);
end
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
freeports = serialportlist("available");
freeports=['---' freeports];
set(hObject,'String',freeports);
function figure1_CloseRequestFcn(hObject, eventdata, handles)
vidDevice=handles.vidDevice;
newobjs = instrfind;
fclose(newobjs);
delete(vidDevice);
delete(hObject);


function [display_voltage,display_current] = serial_read(handles)
ser = handles.ser;
pause(0.3); % bu demi ? evet :D
flushinput(ser);
pause(0.1);
read_voltage=(char(fread(ser)))';
flushinput(ser);
pause(0.1);
akim=strfind(read_voltage,'-');
voltaj=strfind(read_voltage,'+');
akim((diff(akim)==1))=[];
voltaj((diff(voltaj)==1))=[];
[value,~]=min([length(akim)-1,length(voltaj)-1]);
funct= @(x,y) y(voltaj(x):akim(x));
funct2= @(x,y) y(akim(x):voltaj(x+1));
voltaj_values = arrayfun(@(x) funct(x,read_voltage),1:value,'UniformOutput',false);
akim_values = arrayfun(@(x) funct2(x,read_voltage),1:value,'UniformOutput',false);

all_akim = regexp(akim_values,'\d+\.?\d*','Match');
all_voltaj = regexp(voltaj_values,'\d+\.?\d*','Match');
akim_values = str2double(string([all_akim{:}]));
voltaj_values = str2double(string([all_voltaj{:}]));
%alttakiler ekrana bastirilacak
display_voltage = mean((voltaj_values));
display_current = mean((akim_values));



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
voltage_string = get(handles.edit3,'string');
voltage_value=sscanf(voltage_string,'%f');
ser=handles.ser;
pause(0.35)
disp(voltage_value)
fwrite(ser,voltage_value*10);

