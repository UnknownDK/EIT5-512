fs = 1000; 
fc = 200;
N = 4096;
t = 0:1/fs:512/fs-1/fs;
f = (-N/2:N/2-1)/N*fs;

x = sin(2*pi*30*t);%+2*sin(2*pi*60*t);

fDev = 100;

y = fmmod(x,fc,fs,fDev);

plot(t,x,'c',t,y,'b--')
xlabel('Time (s)')
ylabel('Amplitude')
legend('Original Signal','Modulated Signal')

A = fft(x,N);
A = fftshift(A);
B = fft(y,N);
B = fftshift(B);

stem(f,abs(B))