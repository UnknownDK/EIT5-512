
channel1 = freqFinder(0)
%channel2 = freqFinder(20)
%channel3 = freqFinder(40)
%channel4 = freqFinder(60)




function result = freqFinder(i)
    fl = 297.23 + i
    fh = fl + 5;

    x = floor(fh / (fh-fl));
    result = zeros(2,x);
    test = zeros(1,x);

    for n = 1:x
        if (2*fh/n) < (2*fl / (n-1)) 
            result(1,n) = 2*fh/n;
            result(2,n) = 2*fl/(n-1);
        end
    end 
end

function result = freqFinder2(i)
    fl = 290 + i
    fh = fl + 20;

    x = floor(fh / (fh-fl));
    result = zeros(2,x);
    test = zeros(1,x);

    for n = 1:x
        if (2*fh/n) < (2*fl / (n-1)) 
            result(1,n) = 2*fh/n;
            result(2,n) = 2*fl/(n-1);
        end
    end 
end
