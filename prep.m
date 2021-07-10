
% Vorbereiten von der Aufnahmen: Konvertierung und Bildoptimierung

function [I1, I2] = prep 

I1_raw = rgb2gray(imread(fullfile('Images','Wiesn','2018_04.jpg')));
I2_raw = rgb2gray(imread(fullfile('Images','Wiesn','2019_09.jpg')));

I1 = adapthisteq(I1_raw,'NumTiles',[10 10]);
I2 = adapthisteq(I2_raw,'NumTiles',[10 10]);

I2 = imhistmatch(I2,I1)

I1 = single(I1);
I2 = single(I2);


end