% clc 
% clear
% m = 1;
% T = 1;
% t = 0:0.01:250;
% k0 = 1;
% k = k0 + (1+m)/T;
% 
% figure(1);%
% for i = 1:length(t)
%     u(i) = T^(1+m)/(T-t(i))^(1+m);
% end 
% x = ones(1,length(t));
% 
% ui = -k*u.*x;
% 
% 
% plot(t,ui,'b','LineWidth',0.1);
% hold on;              %����ͼ��
% plot(t,x,'r','LineWidth',0.1);
% hold on;              %����ͼ��
% axis([0 250 -2 2]);

% 
% syms  t s;
% 
% m = 1;
% T = 4;
% k0 = 1;
% k = k0 + (1+m)/T;
% u = T^(1+m)/(T-t)^(1+m);
% x = ones(1,length(t));
% ui = -k*u.*x
% 
% s = tf('s');  
% 
% G0 = 1/(s*(s+1)*(0.5*s+1));  %ԭϵͳ��������
% figure(1);%���ƽ�Ծ��Ӧ
% step(feedback(G0,1))  %ԭϵͳ��Ծ��Ӧ
% hold on;              %����ͼ��
% step(G0*uis,t)%У����ϵͳ�Ľ�Ծ��Ӧ
% title('У��ǰ��ϵͳ��Ծ��Ӧ���߶Ա�')





% 
% plot(t,ui,'b','LineWidth',0.1);
% hold on;              %����ͼ��
% plot(t,x,'r','LineWidth',0.1);
% hold on;              %����ͼ��
% axis([0 25 -2 2]);





% axis([0.5 2.5 0.5 2.5]);

% s = tf('s');  
% G0 = 1/(s*(s+1)*(0.5*s+1));  %ԭϵͳ��������
% [Kc,pm,wcg,wcp] = margin(G0);  %������̬����
% Tc = 2 * pi / wcg;   %
% Kp2 = 0.91 * Kc;   %PID����ֵP
% Ti2 = 16 * Tc;     %PID����ֵI
% Td2 = 0.32 *Tc;    %PID����ֵD
% Gc2 = Kp2 * (1+1/Ti2/s+Td2*s) %У������
% G1 = G0*Gc2          %У����ϵͳ�Ŀ�������
% Gs = feedback(G1,1); %���츺����GS
% %%
% %����ϵͳ��Ծ��Ӧ����
% %����ϵͳbodeͼ
% %����ϵͳ���켣ͼ
% figure(1);%���ƽ�Ծ��Ӧ
% step(feedback(G0,1))  %ԭϵͳ��Ծ��Ӧ
% hold on;              %����ͼ��
% step(feedback(G0*Gc2,1),t)%У����ϵͳ�Ľ�Ծ��Ӧ
% title('У��ǰ��ϵͳ��Ծ��Ӧ���߶Ա�')
% hold off;

% figure(2);%����У��ǰϵͳ��bodeͼ
% margin(G0) %����ԭϵͳBodeͼ
% title('У��ǰϵͳ��Bodeͼ')
% figure(3);%����У����ϵͳ��bodeͼ
% margin(G1) %������ϵͳBodeͼ
% title('У����ϵͳ��Bodeͼ')
% figure(4);%����У��ǰϵͳ�ĸ��켣ͼ
% rlocus(G0) %��ȡԭϵͳ���켣
% title('У��ǰϵͳ�ĸ��켣ͼ')
% figure(5);%����У����ϵͳ�ĸ��켣ͼ
% rlocus(G1) %��ȡ��ϵͳ���켣
% title('У����ϵͳ�ĸ��켣ͼ')








%PID Controller
clear, clc, close all;

m = 2;
T = 1;
t = 0:0.01:250;
k0 = 0.5;
% k = k0 + (1+m)/T;
% for i = 1:length(t)
%     u(i) = T^(1+m)/(T-t(i))^(1+m);
% end 
% x = ones(1,length(t));
% ui = -k*u.*x;


% plot(t,ui,'b','LineWidth',0.1);
% hold on;              %����ͼ��
% plot(t,x,'r','LineWidth',0.1);
% hold on;              %����ͼ��
% axis([0 250 -2 2]);





ts=0.001;        %����ʱ��=0.001s
% sys=tf(5.235e005,[1,87.35,1.047e004,0])  %�������ض��󴫵ݺ���
sys=tf(5.235e005,[1.047e004,0])  %�������ض��󴫵ݺ���
dsys=c2d(sys,ts,'z');                  %�Ѵ��ݺ�����ɢ��
[num,den]=tfdata(dsys,'v');       %  ��ɢ������ȡ���ӡ���ĸ

u_1=0.0;u_2=0.0;u_3=0.0;         %�������� �ĳ�ʼ״̬
y_1=0.0;y_2=0.0;y_3=0.0;         %����ĳ�ʼ״̬
x=[0,0,0]';      %PID��3������Kp Ki Kd��ɵ�����
error_1=0;                    %��ʼ���
S = 1;
% S=input('��ѡ�������źŵ���ʽ��1 ��Ծ�ź� 2 �����ź� 3 �����ź�\n');
for k=1:1:5000
        time(k)=k*ts;           % ����ʱ��500ms
    if S==1
        kp=1.50;ki=0.01;kd=0.01;
        yd(k)=1;                       % ָ��Ϊ��Ծ�ź�
    elseif S==2
        kp=0.50;ki=0.001;kd=0.001;
        yd(k)=sign(sin(2*2*pi*k*ts));  % ָ��Ϊ�����ź�
    elseif S==3
        kp=1.5;ki=1.0;kd=0.01;          % ָ��Ϊ�����ź�
        yd(k)=0.5*sin(2*2*pi*k*ts);
    end
    u_u(k) = T^(1+m)/(T-ts*k)^(1+m);
    u(k) =  -k*u_u(k)*x(1);
%     u(k)=kp*x(1)+kd*x(2)+ki*x(3);   % PID������??
    % ���ƿ����������
    if u(k)>=0.1
        u(k)=0.1;
    end
    if u(k)<=-0.1
        u(k)=-0.1;
    end
    % ��������ģ��
    y(k)=-den(2)*y_1+num(2)*u_1;

    error(k)=yd(k)-y(k);
    % ����pid����
    u_3=u_2;u_2=u_1;u_1=u(k);
    y_3=y_2;y_2=y_1;y_1=y(k);
    x(1)=error(k);                % ���� P
    x(2)=(error(k)-error_1)/ts;   % ���� D
    x(3)=x(3)+error(k)*ts;        % ���� I

    error_1=error(k);
end
figure(1);
set(0,'defaultfigurecolor','w') % ����ͼ�񱳾�Ϊ��ɫ
plot(time,yd,'r',time,y,'b','linewidth',2);
xlabel('time(s)');ylabel('�ź����');
legend('�����ź�','׷���ź�');









