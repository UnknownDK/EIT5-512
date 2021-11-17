%% Test af adc oploesning
clearvars
tic

antalTests = 300;

fs = 42e6;
Tmax = 0.0005;
t = 0:1/fs:Tmax-1/fs;



%signaler
f1 = 300e6;
theta_a = pi; 
f2 = 300e6;
theta_b = pi/2;
X_m = 1;            % Range i volt er: +- X_m
Clambda = 0.1228658;
vOensket = (Clambda * 90)/(360 * 0.75 *Clambda);
droneVinkelOensket = (asin(vOensket)) * 180 / pi;

for fftB = 8:1:10

    N = 2^fftB;           % oploesning af FFT
    f = (-N/2:N/2-1)/N*fs;


    e_range = [20 10 90]; %hvad tester vi af SNR f
    b_range = [10 1 16];      %hvad bits tester vi med
    antalB = round((b_range(3) - b_range(1)) / b_range(2));
    antalE = round((e_range(3) - e_range(1)) / e_range(2));

    difResult = zeros(antalB,antalE);
    faseResult = zeros(antalB,antalE);
    droneResult = zeros(antalB,antalE);

    

    taeller_b = 1;
    for i_b = b_range(1):b_range(2):b_range(3)

        taeller_e = 1;
        for i_e = e_range(1):e_range(2):e_range(3)
            
            faseDif = zeros(antalTests,1);
            v = zeros(antalTests,1);
            droneVinkel = zeros(antalTests,1);
            difference = zeros(antalTests,1);


            Bits = i_b;            % Bit number.
            eInput = 0.9;       % % af input range vi bruger
            snr = i_e; %i dB i forhold til signal der udfylder 100% input range


            %Signaler
            a_pre = eInput*X_m*cos((2*pi*f1*t)+theta_a);
            b_pre = eInput*X_m*cos((2*pi*f2*t)+theta_b);

            parfor runI = 1:1:antalTests

                a = awgn(a_pre,snr,'measured');
                b = awgn(b_pre,snr,'measured');
                
                
                % ADC
                % Sampling
                aSampled = a;
                bSampled = b;
    
                % ADC: Quantizer
                % Quantizing (abs of input value not over 1)
                partition = [-X_m:(2*X_m)/(2^Bits):X_m-(2*X_m)/(2^Bits)];
                codebook = [-X_m:(2*X_m)/(2^Bits):X_m-(2*X_m)/(2^Bits)];
                aDigital = quantiz(aSampled,partition,codebook); 
                bDigital = quantiz(bSampled,partition,codebook);
                
                
    
    
    
    
                %fft stuff
                A = fft(aDigital,N);
                A = fftshift(A);
                B = fft(bDigital,N);
                B = fftshift(B);
    
                %sorterer DC tillaegget fra ADC'erne fra:
                for i = (N/2)-(N/16):1:(N/2)+(N/16)
                    A(i) = 0;
                    B(i) = 0;
                end
                

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
            difResult(taeller_b,taeller_e) = mean(difference,'all');
            faseResult(taeller_b,taeller_e) = mean(faseDif,'all');
            droneResult(taeller_b,taeller_e) = mean(droneVinkel,'all');


            taeller_e = taeller_e +1;
        end
        taeller_b = taeller_b + 1;
    end

    filename = "C:\Users\Nikolaj\Desktop\resultater\" + N + "N.mat";
    mySave(filename,difResult,faseResult,droneResult);

end
toc

function mySave(filenm, WT1, WT2, WT3)
    save(filenm, 'WT1', 'WT2', 'WT3');
end