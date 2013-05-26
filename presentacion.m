%% Improfile
% Función improfile RGB
ImgFlor = imread('flor.jpg');

imshow(ImgFlor)
improfile;

%% Función improfile escala de grises
planoAzul = imread('planoAzul.jpg');
imshow(planoAzul)
improfile;

%% Ruido

ImgFlor = imread('flor.jpg');
foto = imnoise(ImgFlor, 'salt & pepper', 0.4);
foto2 = imnoise(ImgFlor, 'gaussian',0.2,0.01);
foto3 = imnoise(ImgFlor, 'speckle', 0.9);

subplot(2,2,1),imshow(ImgFlor),title('Imagen de partida'),
subplot(2,2,2),imshow(uint8(foto)),title('Imagen con ruido impulsivo'),
subplot(2,2,3),imshow(uint8(foto2)),title('Imagen con ruido gaussiano'),
subplot(2,2,4),imshow(uint8(foto3)), title('Filtrado de la imagen con ruido uniforme');

%% Número de objetos

FigColores=imread('figcolores.jpg');
FigBin=im2bw(FigColores,0.5);
FigBin2=not(FigBin);

ImEtiq=bwlabel(FigBin2,8);
n=max(max(ImEtiq))

subplot(1,2,1),imshow(FigBin),
subplot(1,2,2),imshow(FigBin2);




%% Selección de un objeto

imshow(FigBin2);
ImSeleccionada=bwselect(8);
imshow(ImSeleccionada);

