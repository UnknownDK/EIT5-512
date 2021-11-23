%% Test af adc oploesning
clearvars
tic

fs = 42e6;
filterDC = 10;
samples = 42e6;
Tmax = samples/fs;
t = 0:1/fs:Tmax-1/fs;

%signaler
f1 = 300e6;
theta_a = pi; 
f2 = 300e6;
theta_b = pi/2;

eInput = 0.9;       % % af input range vi bruger

a_pre = eInput*cos((2*pi*f1*t)+theta_a);
b_pre = eInput*cos((2*pi*f2*t)+theta_b);

Clambda = 0.1228658;
vOensket = (Clambda * 90)/(360 * 0.75 *Clambda);
droneVinkelOensket = (asin(vOensket)) * 180 / pi;



antalTests = 1; %antal tests til stoej
snr_range = [20 10 50]; %hvad tester vi af SNR f
b_range = [10 2 14];      %hvad bits tester vi med


antalB = round((b_range(3) - b_range(1)) / b_range(2));
antalSNR = round((snr_range(3) - snr_range(1)) / snr_range(2));

N = samples;           % oploesning af FFT
f = (-N/2:N/2-1)/N*fs;


difResult = zeros(antalB,antalSNR);
faseResult = zeros(antalB,antalSNR);
droneResult = zeros(antalB,antalSNR);

b_taeller = 1;
for i_b = b_range(1):b_range(2):b_range(3)
    snr_taeller = 1;
    for i_snr = snr_range(1):snr_range(2):snr_range(3)
        faseDif = zeros(antalTests,1);
        v = zeros(antalTests,1);
        droneVinkel = zeros(antalTests,1);
        difference = zeros(antalTests,1);
        
            
        Bits = i_b;          % Bit number.    
        snr = i_snr;          %i dB i forhold til signal der udfylder 100% input range
        
        

        parfor runI = 1:1:antalTests
            a = awgn(a_pre,snr,'measured');
            b = awgn(b_pre,snr,'measured');
            
            
            % ADC: Quantizer
            % Quantizing (abs of input value not over 1)
            partition = [-1:(2)/(2^Bits):1-((2)/(2^Bits))];
            codebook = [-1:(2)/(2^Bits):1];
            aDigital = quantiz(a,partition,codebook); 
            bDigital = quantiz(b,partition,codebook);    
    
    
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

            faseDif(runI) = mod(abs(faseB - faseA),180);
            v(runI) = (Clambda * faseDif(runI))/(360 * 0.75 *Clambda);
            droneVinkel(runI) = (asin(v(runI))) * 180 / pi;
            difference(runI) = droneVinkelOensket - droneVinkel(runI);
        end
        difference = abs(difference);
        difResult(b_taeller,snr_taeller) = mean(difference,'all');
        faseResult(b_taeller,snr_taeller) = mean(faseDif,'all');
        droneResult(b_taeller,snr_taeller) = mean(droneVinkel,'all');
        snr_taeller = snr_taeller + 1;
    end
    b_taeller = b_taeller + 1;
end

                        
filename = "C:\Users\nikol\Desktop\result.mat";
mySave(filename,difResult,faseResult,droneResult);


toc

function mySave(filenm, WT1, WT2, WT3)
    save(filenm, 'WT1', 'WT2', 'WT3');
end