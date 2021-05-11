
x = zeros(4,4,100);
for i=1:100
%     x(:,:,i) = transl(i/100, 0.25 - i/200, 0.1);
    x(:,:,i) = transl(i/100, 0.15 + 0.1*sin(i*2*pi*0.05), 0.15 +  0.1*cos(i*2*pi*0.05));
end