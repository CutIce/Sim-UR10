function T_lim= setAngleLimits(T)
%SETANGLELIMITS 此处显示有关此函数的摘要
%   此处显示详细说明
[col,row]=size(T);
for i=1:col
    for j=1:row
        while(T(i,j)<-pi)
            T(i,j)=T(i,j)+2*pi;
        end
        while(T(i,j)>pi)
            T(i,j)=T(i,j)-2*pi;
        end
    end
end
T_lim=T;
end

