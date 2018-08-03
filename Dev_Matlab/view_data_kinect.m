clear
close all 
%data = load('data106_sd.txt');
data = load('data100.txt');
x = data(:,1);
y= data(:,2);
z= data(:,3);
r= data(:,4);
g=data(:,5);
b=data(:,6);
%{
figure(1)
    plot3(x,y,z,'.')
    grid on
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
%}

    
