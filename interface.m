function varargout = interface(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interface_OpeningFcn, ...
                   'gui_OutputFcn',  @interface_OutputFcn, ...
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


% --- Executes just before interface is made visible.
function interface_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interface (see VARARGIN)

% Choose default command line output for interface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interface_OutputFcn(hObject , eventdata , handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
  %handles.ejex=0:pi/360:2*pi; 
  %y1=sin(2*pi*1*handles.ejex); 
  %plot(handles.ejex,y1,'LineWidth',2);grid on; 
  %axis([0 2*pi -1 1]);
  ImgFlor = imread('definitiva.jpg');
  subplot(1,2,2),
  imshow(ImgFlor);
varargout{1} = handles.output;


% --- Executes on selection change in buttonColor.
function buttonColor_Callback(hObject, ~, handles)
    v = get(handles.buttonColor, 'Value');
    ImgFlor = imread('flor.jpg');
    switch v   
        case 1
            subplot(1,2,2),
            imshow(ImgFlor);          
        case 2
           %Leemos el plano azul y lo mostramos
           planoAzul = imread('planoAzul.jpg');
           subplot(2,2,2),imshow(ImgFlor),subplot(2,2,4),imshow(planoAzul)
                      
        case 3
           %Leemos el plano verde y lo mostramos
           planoVerde = imread('planoVerde.jpg');
           subplot(2,2,2),imshow(ImgFlor),subplot(2,2,4),imshow(planoVerde)          
        case 4
           %Leemos el plano rojo y lo mostramos
           planoRojo = imread('planoRojo.jpg');
           subplot(2,2,2),imshow(ImgFlor),subplot(2,2,4),imshow(planoRojo)                     
        otherwise 
            subplot(1,2,2),
            imshow(ImgFlor);
          
    end
    guidata(hObject, handles); 
   
% --- Executes on selection change in buttonBordes.
function buttonBordes_Callback(hObject, eventdata, handles)
% hObject    handle to buttonBordes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns buttonBordes contents as cell array
%        contents{get(hObject,'Value')} returns selected item from buttonBordes

    v = get(handles.buttonBordes, 'Value');
    
    switch v       
        case 1
            ImgFlor = imread('flor.jpg');
            planoAzul = ImgFlor(:,:,3);
            ImgBordes= edge(planoAzul, 'canny');
            filter=[1 2 1;0 0 0 ;-1 -2 -1];
            Im_gris=rgb2gray(ImgFlor);           
            ImBordes4=filter2(filter,Im_gris);
            subplot(2,2,2),imshow(ImgBordes),
            title('Extracción canny'),
            subplot(2,2,4),  imshow(ImBordes4),
            title('Extracción por filtrado');

            
        case 2
           
            ImgFlor = imread('flor.jpg');
            planoRojo = ImgFlor(:,:,1);
            planoVerde = ImgFlor(:,:,2);          
            ImgBordes2= edge(planoRojo, 'sobel');
            ImgBordes3= edge(planoVerde, 'prewitt');
            subplot(2,2,2),imshow(ImgBordes2),
            title('Extracción sobel'),
            subplot(2,2,4),imshow(ImgBordes3),
            title('Extracción prewitt');  
         
        case 3
            %Segmentación por umbral rgb
            ImgMariposa= imread('mariposa.jpg');
            ImBin=im2bw(ImgMariposa,0.8);
            subplot(2,2,2),imshow(ImgMariposa),
            subplot(2,2,4), imshow(ImBin), 
            title('Segmentación por umbral RGB');
            
        case 4
            %Segmentación por umbral escala grises
            lobo=imread('lobo.jpg');
            loboR = lobo(:,:,1);
            loboG = lobo(:,:,2);
            loboB = lobo(:,:,3);
            ImBin2=loboR<=30;
            ImBin3=loboG<=90;
            ImBin4=loboB<=150;
            subplot(2,3,2),imshow(lobo), 
            subplot(2,3,3),imshow(ImBin2),title('Umbral: 30'),
            subplot(2,3,5),imshow(ImBin3),title('Umbral: 90') ,
            subplot(2,3,6),imshow(ImBin4), title('Umbral: 150');
            
        otherwise 
            ImgFlor = imread('flor.jpg');
            subplot(1,2,2),
            imshow(ImgFlor);
          
    end
    guidata(hObject, handles);
    
% --- Executes on selection change in buttonFiltros.
function popupmenu1_Callback(hObject, ~, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

 v = get(handles.popupmenu1, 'Value');
  switch v
      case 1
          foto= imread('flor.jpg');
          subplot(1,2,2),
          imshow(foto);
      case 2
            %paso bajo
            foto2= imread('flor.jpg');
            foto = imnoise(foto2, 'gaussian',0.2,0.01);

            foto_R = foto(:,:,1);
            foto_G = foto(:,:,2);
            foto_B = foto(:,:,3);

            filtro = 1/10 *[0 1 0;1 6 1;0 1 0];

            foto2_R = filter2(filtro, double(foto_R));
            foto2_G = filter2(filtro, double(foto_G));
            foto2_B = filter2(filtro, double(foto_B));

            foto3(:,:,1) = foto2_R;
            foto3(:,:,2) = foto2_G;
            foto3(:,:,3) = foto2_B;
            
            subplot(2,2,2),imshow(foto),title('Original con ruido gaussiano'),
            subplot(2,2,4),imshow(uint8(foto3)),title('Filtrada paso bajo');
           
      case 3
            %smooth
            foto= imread('flor.jpg');
            foto2 = imnoise(foto, 'gaussian',0.2,0.01);
            foto_R = foto2(:,:,1);
            foto_G = foto2(:,:,2);
            foto_B = foto2(:,:,3);

            filtro_smooth = 1/16 *[1 2 1;2 4 2;1 2 1];

            foto3_R = filter2(filtro_smooth, double(foto_R));
            foto3_G = filter2(filtro_smooth, double(foto_G));
            foto3_B = filter2(filtro_smooth, double(foto_B));

            foto3(:,:,1) = foto3_R;
            foto3(:,:,2) = foto3_G;
            foto3(:,:,3) = foto3_B;

            subplot(2,2,2),imshow(uint8(foto2)),title('Original con ruido gaussiano'),
            subplot(2,2,4),imshow(uint8(foto3)),title('Filtrada Smooth');
      otherwise 
            subplot(1,2,2),
            imshow(ImgFlor);
  end
guidata(hObject, handles);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over buttonColor.
function buttonColor_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to buttonColor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function buttonFiltros_Callback(hObject, eventdata, handles)
% hObject    handle to buttonFiltros (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns buttonFiltros contents as cell array
%        contents{get(hObject,'Value')} returns selected item from buttonFiltros
  v = get(handles.buttonFiltros, 'Value');
  switch v     
      case 1
         %filtro mediana
            imc=imread('rectangulos.jpg'); 
            im=rgb2gray(imc); 
            fs = imnoise(im,'salt & pepper',0.1);
            mediana2=medfilt2(fs); 
            %Representaciones de las imágenes 
            subplot(2,2,2),subimage(fs),
            title('Imagen con ruido aleatorio'); 
            subplot(2,2,4),subimage(mediana2),
            title('Filtro mediana');
            
      case 2
         %filtro media
         imc=imread('rectangulos.jpg'); 
         im=rgb2gray(imc); 
         fg = imnoise(im,'gaussian'); 
         h1=fspecial('average'); 
         media1=imfilter(fg,h1); 
         %Representaciones de las imágenes 
         subplot(2,2,2),subimage(fg),title('Imagen con ruido gaussiano'); 
         subplot(2,2,4),subimage(media1),title('Filtro media');
         
      case 3
        %paso alto    
          foto= imread('flor.jpg');

          foto_R = foto(:,:,1);
          foto_G = foto(:,:,2);
          foto_B = foto(:,:,3);

          filtroPasoAlto = [-1 -1 -1;-1 8 -1;-1 -1 1];

          foto3_R = filter2(filtroPasoAlto, double(foto_R));
          foto3_G = filter2(filtroPasoAlto, double(foto_G));
          foto3_B = filter2(filtroPasoAlto, double(foto_B));

          foto3(:,:,1) = foto3_R;
          foto3(:,:,2) = foto3_G;
          foto3(:,:,3) = foto3_B;

          subplot(2,2,2),imshow(foto),title('Original'),
          subplot(2,2,4),imshow(uint8(foto3)),title('Filtrada paso alto');
      otherwise
          
  end
  
          

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2
v = get(handles.popupmenu2, 'Value');

switch v 
      case 1
        Rectangulos=imread('rectangulos.jpg');
        subplot(1,2,2),
        imshow(Rectangulos);
      case 2
        % Erosión
        Rectangulos=imread('rectangulos.jpg');
        SE=strel('square',15);
        
        rectGris=rgb2gray(Rectangulos);
        rectBordes=edge(rectGris,'sobel');
        rectDilatada=imdilate(rectBordes,SE);
        rectErode=imerode(rectDilatada,SE);
        
        subplot(2,3,2),imshow(uint8(Rectangulos)),
        subplot(2,3,3),imshow(rectDilatada),
        subplot(2,3,5),imshow(rectErode);
        
      case 3
        %  Dilatacion
        SE=strel('square',30);
        
        Rectangulos=imread('rectangulos.jpg');       
        rectGris=rgb2gray(Rectangulos);
        rectBordes=edge(rectGris,'sobel');
        rectDilatada=imdilate(rectBordes,SE);
        
        subplot(2,3,2),imshow(Rectangulos),
        subplot(2,3,3),imshow(rectBordes),
        subplot(2,3,5),imshow(rectDilatada);
    otherwise
end

  

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


% --- Executes on selection change in buttonIntensidad.
function buttonIntensidad_Callback(hObject, eventdata, handles)
% hObject    handle to buttonIntensidad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns buttonIntensidad contents as cell array
%        contents{get(hObject,'Value')} returns selected item from buttonIntensidad
v = get(handles.buttonIntensidad,'Value');
switch v 
      case 1        
        ImgFlor = imread('flor.jpg');
        subplot(1,2,2),
        imshow(ImgFlor);
          
      case 2
        %Ampliamos la intensidad. 
        ImgFlor = imread('flor.jpg');
        
        planoVerde = ImgFlor(:,:,2);
        planoVerdeD= double(planoVerde);
        planoVerdeD=planoVerdeD*1.4;
        planoVerde2= uint8(planoVerdeD);
        
        subplot(2,2,2), imshow(planoVerde),  title('Imagen original'),
        subplot(2,2,4),  imshow(planoVerde2), title('Mayor intensidad'),
          
      case 3
        % Reducimos la intensidad.
         ImgFlor = imread('flor.jpg');
         
         planoRojo = ImgFlor(:,:,1);
         planoRojoD= double(planoRojo);
         planoRojoD=planoRojoD*0.4;
         planoRojo2= uint8(planoRojoD);
         subplot(2,2,2), imshow(planoRojo), title('Imagen original'),
         subplot(2,2,4),  imshow(planoRojo2), title('Menor intensidad'),
          
  end
  

% --- Executes during object creation, after setting all properties.
function buttonIntensidad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to buttonIntensidad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in buttonTransformar.
function buttonTransformar_Callback(hObject, eventdata, handles)
% hObject    handle to buttonTransformar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns buttonTransformar contents as cell array
%        contents{get(hObject,'Value')} returns selected item from buttonTransformar


% --- Executes during object creation, after setting all properties.
function buttonTransformar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to buttonTransformar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
