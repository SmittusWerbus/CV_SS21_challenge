
% Vorbereiten von der Aufnahmen: Konvertierung und Bildoptimierung

function [I1, I2] = prep 

I1_raw = rgb2gray(imread('./Images/Frauenkirche/2012_08.jpg'));
I2_raw = rgb2gray(imread('./Images/Frauenkirche/2015_07.jpg'));

I1 = adapthisteq(I1_raw,'NumTiles',[10 10]);
I2 = adapthisteq(I2_raw,'NumTiles',[10 10]);

end