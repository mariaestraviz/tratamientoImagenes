%% TRATAMIENTO DE IMÁGENES EN MATLAB
% Ltds 2013
% Autores: Beatriz Fuster González y María Estraviz Pardo

%% RGB

%Leer la imagen
ImgFlor = imread('flor.jpg');

%% Calcular el tamaño
[m,n]=size(ImgFlor);

%% Mostramos la imagen

imshow(ImgFlor);
pixel=impixel;

%% Grabar contenido de una imagen en un archivo
imwrite(ImgFlor,'datos.jpg')

%% Leemos la nueva imagen y la mostramos
ImgFlor2 = imread('datos.jpg');
imshow(ImgFlor2)

%% Obtención de planos
planoR = ImgFlor(:,:,1);
planoG = ImgFlor(:,:,2);
planoB = ImgFlor(:,:,3);

%% Leemos el plano rojo y lo mostramos
imwrite(planoR,'planoRojo.jpg');

planoRojo = imread('planoRojo.jpg');
subplot(1,2,1),imshow(ImgFlor),
subplot(1,2,2),imshow(planoRojo);

%% Leemos el plano verde y lo mostramos

imwrite(planoG,'planoVerde.jpg');

planoVerde = imread('planoVerde.jpg');
subplot(1,2,1),imshow(ImgFlor),
subplot(1,2,2),imshow(planoVerde);


%% Leemos el plano azul y lo mostramos

imwrite(planoB,'planoAzul.jpg');

planoAzul = imread('planoAzul.jpg');
subplot(1,2,1),imshow(ImgFlor),subplot(1,2,2),imshow(planoAzul)

%% Valor de un pixel
imshow(ImgFlor);


%% Manipulación de pixeles

ImgFlor(1200,1200,1);
ImgFlor(1200,1200,2);
ImgFlor(1200,1200,3);


%% Cambiamos el valor del pixel a negro:

ImgFlor(1200,1200,1)=0;
ImgFlor(1200,1200,2)=0;
ImgFlor(1200,1200,3)=0;
imshow(ImgFlor)

%% Cambiamos el valor del pixel a blanco:

valorR=255;
valorG=255;
valorB=255;

imshow(ImgFlor)

%% Función improfile RGB

imshow(ImgFlor)
improfile;

%% Función improfile escala de grises
planoAzul = imread('planoAzul.jpg');
imshow(planoAzul)
improfile;


 %% Vamos a muestrear la imagen. Primero la convertimos a tipo double  

Imagen_sub = ImgFlor(1:4:end,1:4:end,1:1:end);
imshow(ImgFlor);
figure,imshow(Imagen_sub);
  
 %% Ampliamos la intensidad.
 
planoVerdeD= double(planoVerde);
planoVerdeD=planoVerdeD*1.4;
planoVerde2= uint8(planoVerdeD);
subplot(1,2,1), imshow(planoVerde), subplot(1,2,2),  imshow(planoVerde2);


 %% Reducimos la intensidad.
  
 planoRojoD= double(planoRojo);
 planoRojoD=planoRojoD*0.4;
 planoRojo2= uint8(planoRojoD);
 subplot(1,2,1), imshow(planoRojo), 
 subplot(1,2,2),  imshow(planoRojo2);
 
 %% REDUCCIÓN DE RUIDO EN UNA IMAGEN DIGITAL. 

%Filtros no lineales: de media y mediana

imc=imread('rectangulos.jpg'); 
im=rgb2gray(imc); 
fg = imnoise(im,'gaussian'); 
fs = imnoise(im,'salt & pepper',0.1);
h1=fspecial('average'); 

media1=imfilter(fg,h1); 
media2=imfilter(fs,h1); 

mediana1=medfilt2(fg); 
mediana2=medfilt2(fs); 

%Representación de las imágenes 

subplot(2,3,1),subimage(fg),title('Imagen con ruido gaussiano'); 
subplot(2,3,4),subimage(fs),title('Imagen con ruido aleatorio'); 
subplot(2,3,2),subimage(media1),title('Filtro media'); 
subplot(2,3,5),subimage(media2),title('Filtro media'); 
subplot(2,3,3),subimage(mediana1),title('Filtro mediana'); 
subplot(2,3,6),subimage(mediana2),title('Filtro mediana');


%% Filtros de suavizado
% Filtro paso bajo y smooth

foto = imnoise(ImgFlor, 'salt & pepper', 0.4);

foto_R = foto(:,:,1);
foto_G = foto(:,:,2);
foto_B = foto(:,:,3);

filtro = 1/10 *[0 1 0;1 6 1;0 1 0];

foto2_R = filter2(filtro, double(foto_R));
foto2_G = filter2(filtro, double(foto_G));
foto2_B = filter2(filtro, double(foto_B));

foto2(:,:,1) = foto2_R;
foto2(:,:,2) = foto2_G;
foto2(:,:,3) = foto2_B;

filtro_smooth = 1/16 *[1 2 1;2 4 2;1 2 1];

foto3_R = filter2(filtro_smooth, double(foto_R));
foto3_G = filter2(filtro_smooth, double(foto_G));
foto3_B = filter2(filtro_smooth, double(foto_B));

foto3(:,:,1) = foto3_R;
foto3(:,:,2) = foto3_G;
foto3(:,:,3) = foto3_B;

subplot(2,2,1),imshow(ImgFlor),title('original'),
subplot(2,2,2),imshow(foto),title('original con ruido')
subplot(2,2,3),imshow(uint8(foto2)),title('filtrada paso bajo')
subplot(2,2,4),imshow(uint8(foto3)),title('filtrada Smooth');

 %% Filtrado paso bajo

foto2 = imnoise(ImgFlor, 'salt & pepper', 0.4);

foto_R = foto2(:,:,1);
foto_G = foto2(:,:,2);
foto_B = foto2(:,:,3);

filtro = 1/9 *[1 1 1;1 1 1;1 1 1];

foto2_R = filter2(filtro, double(foto_R));
foto2_G = filter2(filtro, double(foto_G));
foto2_B = filter2(filtro, double(foto_B));

foto2(:,:,1) = foto2_R;
foto2(:,:,2) = foto2_G;
foto2(:,:,3) = foto2_B;

foto3 = imnoise(ImgFlor, 'speckle', 0.9);
foto_R = foto3(:,:,1);
foto_G = foto3(:,:,2);
foto_B = foto3(:,:,3);

foto3_R = filter2(filtro, double(foto_R));
foto3_G = filter2(filtro, double(foto_G));
foto3_B = filter2(filtro, double(foto_B));

foto3(:,:,1) = foto3_R;
foto3(:,:,2) = foto3_G;
foto3(:,:,3) = foto3_B;


foto4 = imnoise(ImgFlor, 'gaussian',0.2,0.01);
foto_R = foto4(:,:,1);
foto_G = foto4(:,:,2);
foto_B = foto4(:,:,3);


foto4_R = filter2(filtro, double(foto_R));
foto4_G = filter2(filtro, double(foto_G));
foto4_B = filter2(filtro, double(foto_B));

foto4(:,:,1) = foto4_R;
foto4(:,:,2) = foto4_G;
foto4(:,:,3) = foto4_B;

subplot(2,2,1),imshow(ImgFlor),title('Imagen de partida'),
subplot(2,2,2),imshow(uint8(foto2)),title('Filtrado de la imagen con ruido impulsivo'),
subplot(2,2,3),imshow(uint8(foto3)),title('Filtrado de la imagen con ruido speckle'),
subplot(2,2,4),imshow(uint8(foto4)), title('Filtrado de la imagen con ruido gaussiano');

%% Filtro paso alto

% Los coeficientes serán positivos y negativos y normalmente
% de suma nula

foto = imnoise(ImgFlor, 'salt & pepper', 0.4);

foto_R = foto(:,:,1);
foto_G = foto(:,:,2);
foto_B = foto(:,:,3);

filtroPasoAlto = [-1 -1 -1;-1 8 -1;-1 -1 1];

foto2_R = filter2(filtroPasoAlto, double(foto_R));
foto2_G = filter2(filtroPasoAlto, double(foto_G));
foto2_B = filter2(filtroPasoAlto, double(foto_B));

foto2(:,:,1) = foto2_R;
foto2(:,:,2) = foto2_G;
foto2(:,:,3) = foto2_B;

imwrite(foto, 'filtroRuidoPasoAlto.jpg');
imwrite(foto2, 'filtradaPasoAlto.jpg');

subplot(2,2,1),imshow(ImgFlor),title('original'),
subplot(2,2,2),imshow(foto),title('original con ruido'),
subplot(2,2,3),imshow(uint8(foto2)),title('filtrada paso alto');


%% Extraccion de bordes mediante la función  edge y filtrado.

% algoritmos de edge
ImgBordes= edge(planoAzul, 'canny');
ImgBordes2= edge(planoRojo, 'sobel');
ImgBordes3= edge(planoVerde, 'prewitt');

%filtrado
filter=[1 2 1;0 0 0 ;-1 -2 -1];
Im_gris=rgb2gray(ImgFlor);
ImBordes4=filter2(filter,Im_gris);

subplot(2,2,1),imshow(ImgBordes2), title('Extracción con el algoritmo sobel'),
subplot(2,2,2),imshow(ImgBordes), title('Extracción con el algoritmo canny'),
subplot(2,2,3), imshow(ImgBordes3), title('Extracción con el algoritmo prewitt'),
subplot(2,2,4),imshow(ImBordes4), title('Extracción por filtrado');


%% Segmentación por umbral rgb

ImgMariposa= imread('mariposa.jpg');
ImBin=im2bw(ImgMariposa,0.8);
subplot(1,2,1), imshow(ImgMariposa), 
subplot(1,2,2),  imshow(ImBin);


%% Segmentación por umbral escala grises

lobo=imread('lobo.jpg');
loboR = lobo(:,:,1);
loboG = lobo(:,:,2);
loboB = lobo(:,:,3);
ImBin2=loboR<=30;
ImBin3=loboG<=90;
ImBin4=loboB<=150;
subplot(2,2,1), imshow(lobo), title('Imagen original'),
subplot(2,2,2),  imshow(ImBin2), title('Umbral: 30'),
subplot(2,2,3),  imshow(ImBin3),  title('Umbral: 90')
subplot(2,2,4) ,imshow(ImBin4),  title('Umbral: 150');
 
 
%% OPERACIONES MORFOLÓGICAS

%%  Dilatacion

SE=strel('square',30);
Rectangulos=imread('rectangulos.jpg');
rectGris=rgb2gray(Rectangulos);
rectBordes=edge(rectGris,'sobel');
rectDilatada=imdilate(rectBordes,SE);

subplot(2,2,1),imshow(Rectangulos),
subplot(2,2,2),imshow(rectBordes),
subplot(2,2,3),imshow(rectDilatada);

%% Erosión

SE=strel('square',15);
rectErode=imerode(rectDilatada,SE);
subplot(2,2,1),imshow(Rectangulos),
subplot(2,2,2),imshow(rectDilatada),
subplot(2,2,3),imshow(rectErode);


%% Obtención del número de objetos

FigColores=imread('figcolores.jpg');
FigBin=im2bw(FigColores,0.5);
FigBin2=not(FigBin);
subplot(1,2,1),imshow(FigBin),subplot(1,2,2),imshow(FigBin2)
ImEtiq=bwlabel(FigBin2,8);
n=max(max(ImEtiq))

%% Selección de un objeto

imshow(FigBin2);
ImSeleccionada=bwselect(8);
imshow(ImSeleccionada);

%% TRANSFORMACIONES

close all
[X,mp] = imread('cameraman.tif');

figure(1)
imshow(X,mp)
axis on
title('imagen original')

[N,M] = size(X);

%% Rotacion

Y = zeros(N,M);
th = 30*pi/180;

for i=1:N
    for j=1:M
       m = [cos(th) sin(th);-sin(th) cos(th)]*[i j]';
       ip = fix(m(1)+0.5);
       jp = fix(m(2)+0.5);
       if (jp>=1) & (jp<=M) & (ip>=1) & (ip<=N)
           Y(i,j) = X(ip,jp);
       end
   end
end

figure(3)
imshow(Y,mp)
axis on
title(sprintf('imagen con rotacion en (%f)',th*180/pi))

%% Rotacion y traslacion

Y = zeros(N,M);
th = 30*pi/180;
i0 = 128;
j0 = 128;

for i=1:N
    for j=1:M

        m = [cos(th) sin(th);-sin(th) cos(th)]*[i-i0 j-j0]'+[i0 j0]';
       ip = fix(m(1)+0.5);
       jp = fix(m(2)+0.5);
       if (jp>=1) & (jp<=M) & (ip>=1) & (ip<=N)
           Y(i,j) = X(ip,jp);
       end
   end
end

figure(3)
imshow(Y,mp)
axis on
title(sprintf('imagen con rotacion en %f y tras lac (%d,%d)',th*180/pi,i0,j0))

%% Deformacion radial

Y = zeros(N,M);

i0 = 128;
j0 = 128;

for i=1:N
    for j=1:M
       [th,r] = cart2pol(i-i0,j-j0);
       rp  = sqrt(r*r+r^3)/10;
       thp = th ;
       ip = fix(rp*cos(thp)+i0+0.5);
       jp = fix(rp*sin(thp)+j0+0.5);
       if (jp>=1) & (jp<=M) & (ip>=1) & (ip<=N)
           Y(i,j) = X(ip,jp);
       end
   end
end

figure(6)
imshow(Y,mp)
axis on
title(sprintf('imagen con distorsion radial'))


