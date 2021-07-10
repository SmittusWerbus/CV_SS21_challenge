
% Vorbereiten von der Aufnahmen: Konvertierung und Bildoptimierung

function [I1, I2] = prep 

I1_raw = rgb2gray(imread(fullfile('Images','Dubai','1990_12.jpg')));
I2_raw = rgb2gray(imread(fullfile('Images','Dubai','2003_12.jpg')));

I1 = adapthisteq(I1_raw,'NumTiles',[10 10]);
I2 = adapthisteq(I2_raw,'NumTiles',[10 10]);

I2 = imhistmatch(I2,I1)

I1 = single(I1);
I2 = single(I2);


end