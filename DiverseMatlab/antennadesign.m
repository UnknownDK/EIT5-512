%Size of PCB
pcbThickness = 3e-3;  %1.6mm
pcbLength = 152.4e-3;   %152.4mm
pcbWidth = 101.6e-3;    %101.6mm
%Specifying Material of PCB
pcbMaterial = 'polystyrene';
pcbEpsilonR = 2.2;
%Creating dielectic Material
d = dielectric(pcbMaterial); 
d.EpsilonR = pcbEpsilonR;
d.Thickness = pcbThickness;
AntennaPlane=antenna.Rectangle('Length',0.5e-2,'Width',5e-2,'Center',[0, 0.020]); %Creating Feed Plane of Antenna 
GndPlane = antenna.Rectangle('Length',pcbLength,'Width',pcbWidth); %Creating Ground Plane of Antenna
%Creating Different Shapes of antenna
Rec = antenna.Rectangle('Length',0.050,'Width',0.040,'Center',[0,-0.020]);
%%Creating PCB Stack
p = pcbStack;
p.Name = 'Strip-fed slot';
p.BoardShape = GndPlane;
p.BoardThickness = pcbThickness;
p.Layers = {AntennaPlane,d,GndPlane}; %[x Cordinate,y Cordinate,startLayer stopLayer]
p.FeedLocations = [0,(pcbWidth/2)-0.030,1,3];
 
%Adding all different shapes of antenna
AntennaPlane = AntennaPlane + Rec;
p.Layers = {AntennaPlane,d,GndPlane};
%Plotting Different patterns and graphs
figure(1);
show(p); %Display Antenna 
figure(2);
pattern(p,2.441e9); %Display Radiation Pattern
figure(3);
impedance(p,2.4e9:2e7:2.483e9); %Display Impedance Graph from 1.6GHz to 2.2GHz
freq = linspace(2.4e9, 2.483e9, 50);  % Creating Frequency Vector
s = sparameters(p,freq,50);     % Calalculate S11 for all frequencys
figure(4);
rfplot(s);%Diplay S11 Plot
%Generating Gerber Files for Fabrication 
C = PCBConnectors.SMA_Cinch;
W = PCBServices.PCBWayWriter;
W.Filename = 'antenna_design_file';
gerberWrite(p,W,C);
%This will genrate a ZIP file in your project folder with Name "antenna_design_file.zip"
%Now just Upload the Gerber file to any PCB Service online and you are ready to go
%Else if you want to make PCB Yourself then upload the files to https://www.gerber-viewer.com/
%from there you can conver the gerber into PDF and take print of all individual layers