1. MyAppInstaller_web.exe auführen - Installiert benötigte Toolboxen

2. SatelliteImageChangeRecognition.exe ausführen - Startet das Hauptprogramm
2.1 Bilder hinzufügen über den "Upload" Button links oben oder direkt in der Mitte (mindestens zwei Bilder)
2.2 Beschriftung der Bilder: yyyy_mm
2.3 Im Reiter "Loaded Pictures" können die hinzugefügten Bilder betrachtet werden.
2.4 Mit dem "Löschen" Button links oben kann das Programm zurückgesetzt werden

3. Starten der Detektion
3.1 "Density Range" unten mittig aus- oder einschalten (schnellere Berechnung, wenn ausgeschaltet)
3.2 Starten über den Button "RUN"

4. Ergebnis
4.1 "Feature Detection": Keypoints und Matched Keypoints pro Bild anzeigen lassen (mit dem Schalter zwischen den zwei Funktionen umschalten)
4.2 Prepared Pictures: Rotierte und überlappende Bilder anzeigen lassen (mittels Schalter umschalten)
4.3 Change Detection: Überlappende Änderungen und Änderungen einzelner Bilder anzeigen lassen (mittels Schalter umschalten), Histogramm zeigt die Verteilung der Änderungen über alle Bilder
4.4 Density of Change: Die Änderungspixel werden abhängig von ihrer Distanz zu mittels Clustering einer Bin zugewiesen mit welcher via Interpolation eine Heatmap erzeugt wird 
		       die eine qualitative Darstellung von der Änderung pro Fläche ermöglicht. Dabei handelt es sich immer um eine Darstellung der relativen Änderungsdichte.
		       Die Funktion ist jedoch sehr rechen- und zeitintensiv (min 16 Gb RAM werden empfohlen) abhängig von detektieren Veränderung und der Anzahl der untersuchten Satellitenaufnahmen.
		       Die Darstellung hat durch die vorgenommene Interpolation und die laufzeitverbessernde dezimierung des Änderungsvektor den Nachteil dass diese unter Umständen sehr fragil sein kann.
		       
		       Die Canopy Darstellung stellt eine perspektivische Darstellung der Änderungshotspots (Berge des 3D Mesh) und der gleichgebliebenen Areale dar. 
		       Die Gegenüberstellung der Heatmap vs. des Bildes ist das 2D Equivalent der Canopy Darstellung.  
		       Bei beiden Plots wird der Bezug der Änderung vom aktuellen Bild zum vorherigen Bild hergestellt, dazu einfach durchklicken.
