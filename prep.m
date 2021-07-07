
% Vorbereiten von der Aufnahmen: Konvertierung und Bildoptimierung

function [I1, I2] = prep 

I1_raw = rgb2gray(imread(fullfile('images','Frauenkirche','2012_08.jpg')));
I2_raw = rgb2gray(imread(fullfile('images','Frauenkirche','2015_08.jpg')));

I1 = adapthisteq(I1_raw,'NumTiles',[10 10]);
I2 = adapthisteq(I2_raw,'NumTiles',[10 10]);

I1 = single(I1);
I2 = single(I2);

end