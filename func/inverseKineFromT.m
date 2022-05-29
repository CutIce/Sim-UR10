function q = inverseKineFromT(T)
    [robot, para, axis] = model();

%     d1 = 127.3;
%     a2 = -612;
%     a3 = -572.3;
%     d4 = 163.941;
%     d5 = 115.7;
%     d6 = 92.2 + 3.5;
    d1 = para(1, 2);
    a2 = para(2, 3);
    a3 = para(3, 3);
    d4 = para(4, 2);
    d5 = para(5, 2);
    d6 = para(6, 2);


%    [robot, DHpara, axis] = model();
%    [d1; d2; d3; d4; d5; d6] = DHpara(:, 2);
%    [a1; a2; a3; a4; a5; a6] = DHpara(:, 3);
    T=double(T);
    n=T(1:3,1);
    o=T(1:3,2);
    a=T(1:3,3);
    p=T(1:3,4);
    
    %% calc theta1
    m = d6 * a(2) - p(2);
    w = a(1) * d6 - p(1);
    if m^2 + w^2 - d4^2 < 0
        fprintf("Solve q1 Error\n")
        return
    end
    theta1(1) = atan2(m, w) - atan2(d4, sqrt(m^2 + w^2 - d4^2));
    theta1(2) = atan2(m, w) - atan2(d4, -sqrt(m^2 + w^2 - d4^2));
    %% calc theta5
    num = 1;
    for i = 1:length(theta1)
        th1 = theta1(i);
        if abs(a(1) * sin(th1) - a(2) * cos(th1)) <= 1
            theta5(num) =  acos(a(1) * sin(th1) - a(2) * cos(th1));
            num = num + 1;
            theta5(num) = -acos(a(1) * sin(th1) - a(2) * cos(th1));
            num = num + 1;
        end
    end
    
    %% calc theta6
    num = 1;
    for i=1:length(theta5)
        th5=theta5(i);
        th1=theta1(i/2+0.25+0.25*(-1)^(i+1));
        m=n(1)*sin(th1)-n(2)*cos(th1);
        w=o(1)*sin(th1)-o(2)*cos(th1);
        if(sin(th5)==0)
            fprintf("Solve q5 Error\n");
            return
        else
            theta6(num)=atan2(m,w)-atan2(sin(th5),0);       
            num=num+1;
        end
    end
    %% calc theta2,3,4
    num=1;
    for i=1:length(theta6)
        th6=theta6(i);
        th1=theta1(i/2+0.25+0.25*(-1)^(i+1));
        c1=cos(th1);s1=sin(th1);
        c6=cos(th6);s6=sin(th6);

        m=d5*(s6*(n(1)*c1+n(2)*s1)+c6*(o(1)*c1+o(2)*s1))-d6*(a(1)*c1+a(2)*s1)+p(1)*c1+p(2)*s1;
        w=p(3)-d1-a(3)*d6+d5*(o(3)*c6+n(3)*s6);
        if(m^2+w^2<(a2+a3)^2)
            theta3(num)=acos((m^2+w^2-a2^2-a3^2)/2/a2/a3);
            % calc theta2
            c3=cos(theta3(num));s3=sin(theta3(num));
            s2=((a3*c3+a2)*w-a3*s3*m)/(a2^2+a3^2+2*a2*a3*c3);
            c2=(m+a3*s3*s2)/(a3*c3+a2);
            theta2(num)=atan2(s2,c2);
            % calc theta4
            theta4(num)=atan2(-s6*(n(1)*c1+n(2)*s1)-c6*(o(1)*c1+o(2)*s1),o(3)*c6+n(3)*s6)-theta2(num)-theta3(num);
            num=num+1;
            
            theta3(num)=-acos((m^2+w^2-a2^2-a3^2)/2/a2/a3);
            c3=cos(theta3(num));s3=sin(theta3(num));
            s2=((a3*c3+a2)*w-a3*s3*m)/(a2^2+a3^2+2*a2*a3*c3);
            c2=(m+a3*s3*s2)/(a3*c3+a2);
            theta2(num)=atan2(s2,c2);
            theta4(num)=atan2(-s6*(n(1)*c1+n(2)*s1)-c6*(o(1)*c1+o(2)*s1),o(3)*c6+n(3)*s6)-theta2(num)-theta3(num);
            num=num+1;
        else
            fprintf("Solve q2 Error\n")
            return
        end
    end
    %% sorting answers
    theta=zeros(length(3),6);
    for i=0:1
        for j=0:1
                for k=1:2
                    theta(4*i+2*j+k,1)=theta1(i+1);
                    theta(4*i+2*j+k,2)=theta2(4*i+2*j+k);
                    theta(4*i+2*j+k,3)=theta3(4*i+2*j+k);
                    theta(4*i+2*j+k,4)=theta4(4*i+2*j+k);
                    theta(4*i+2*j+k,5)=theta5(2*i+j+1);
                    theta(4*i+2*j+k,6)=theta6(2*i+j+1);
                end
        end
    end
    theta=setAngleLimits(theta);
    flag=zeros(1,length(theta3));
    counter=0;
    for i=1:length(theta3)-1
        for j=i+1:length(theta3)
            if(norm(theta(i,:)-theta(j,:))<1e-9)
                flag(j)=1;
                counter=counter+1;
            end
        end
    end
    num=1;
    for i=1:length(theta3)-counter
        if(flag(i)==0)
            q(i,:)=theta(num,:);
            num=num+1;
        end
    end
end