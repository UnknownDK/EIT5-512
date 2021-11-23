%% Test af adc oploesning
clearvars
tic

%signaler
f1 = 300e6;
theta_a = pi; 
f2 = 300e6;
theta_b = pi/2;

Clambda = 0.1228658;
vOensket = (Clambda * 90)/(360 * 0.75 *Clambda);
droneVinkelOensket = (asin(vOensket)) * 180 / pi



antalTests = 3000; %antal tests til stoej

fs = 42e6;
sample_range = [256 256 2048]; %antal samples vi tester?? 
snr_range = [46 1 49]; %hvad tester vi af SNR f
b_range = [12 2 12];      %hvad bits tester vi med

antalSample = round((sample_range(3) - sample_range(1)) / sample_range(2));
antalB = round((b_range(3) - b_range(1)) / b_range(2));
antalSNR = round((snr_range(3) - snr_range(1)) / snr_range(2));


filterDC = 10;
samples = 42e6;
Tmax = samples/fs;
t = 0:1/fs:Tmax-1/fs;


%for fftB = 9:1:10
N = samples;           % oploesning af FFT
f = (-N/2:N/2-1)/N*fs;


difResult = zeros(antalB,antalSNR);
faseResult = zeros(antalB,antalSNR);
droneResult = zeros(antalB,antalSNR);

%for i_b = b_range(1):b_range(2):b_range(3)
%for i_e = snr_range(1):snr_range(2):snr_range(3)
faseDif = zeros(antalTests,1);
v = zeros(antalTests,1);
droneVinkel = zeros(antalTests,1);
difference = zeros(antalTests,1);

    
Bits = 14;          % Bit number.    
eInput = 0.9;       % % af input range vi bruger
snr = 0;          %i dB i forhold til signal der udfylder 100% input range
    
    
%Signaler
a = eInput*cos((2*pi*f1*t)+theta_a);
b = eInput*cos((2*pi*f2*t)+theta_b);


% ADC
% Sampling
aSampled = a;
bSampled = b;    

% ADC: Quantizer
% Quantizing (abs of input value not over 1)
partition = [-1:(2)/(2^Bits):1-(2)/(2^Bits)];
codebook = [-1:(2)/(2^Bits):1-(2)/(2^Bits)];
aDigital = quantiz(aSampled,partition,codebook); 
bDigital = quantiz(bSampled,partition,codebook);    


%fft stuff
A = fft(aDigital,N);
A = fftshift(A);
B = fft(bDigital,N);
B = fftshift(B);

%sorterer DC tillaegget fra ADC'erne fra:
for i = (N/2)-(N/filterDC):1:(N/2)+(N/filterDC)
    A(i) = 0;
    B(i) = 0;
end
        
%stem(f,abs(A))        
%thetaA = angle(A);
%thetaB = angle(B);

[maxAmpValB, indexB] = max(abs(B));
%[maxAmpValA, indexA]= max(abs(A));
faseB = angle(B(indexB)) * 180 / pi;
faseA = angle(A(indexB)) * 180 / pi;            


faseDif = mod(abs(faseB - faseA),180);
v = (Clambda * faseDif)/(360 * 0.75 *Clambda);
droneVinkel = (asin(v)) * 180 / pi
difference = droneVinkelOensket - droneVinkel
    

% faseDif(runI) = mod(abs(faseB - faseA),180);
% v(runI) = (Clambda * faseDif(runI))/(360 * 0.75 *Clambda);
% droneVinkel(runI) = (asin(v(runI))) * 180 / pi;
% difference(runI) = droneVinkelOensket - droneVinkel(runI);
%     
% difference = abs(difference);
% difResult(taeller_b,taellerSamp) = mean(difference,'all');
% faseResult(taeller_b,taellerSamp) = mean(faseDif,'all');
% droneResult(taeller_b,taellerSamp) = mean(droneVinkel,'all');
                
                
%filename = "C:\Users\nikol\Desktop\resultater\" + N + "N.mat";
%mySave(filename,difResult,faseResult,droneResult);


toc

function mySave(filenm, WT1, WT2, WT3)
    save(filenm, 'WT1', 'WT2', 'WT3');
end