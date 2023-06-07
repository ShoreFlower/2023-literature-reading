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
% hold on;              %保存图像
% plot(t,x,'r','LineWidth',0.1);
% hold on;              %保存图像
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
% G0 = 1/(s*(s+1)*(0.5*s+1));  %原系统开环传函
% figure(1);%绘制阶跃响应
% step(feedback(G0,1))  %原系统阶跃响应
% hold on;              %保存图像
% step(G0*uis,t)%校正后系统的阶跃响应
% title('校正前后系统阶跃响应曲线对比')





% 
% plot(t,ui,'b','LineWidth',0.1);
% hold on;              %保存图像
% plot(t,x,'r','LineWidth',0.1);
% hold on;              %保存图像
% axis([0 25 -2 2]);





% axis([0.5 2.5 0.5 2.5]);

% s = tf('s');  
% G0 = 1/(s*(s+1)*(0.5*s+1));  %原系统开环传函
% [Kc,pm,wcg,wcp] = margin(G0);  %返回稳态参数
% Tc = 2 * pi / wcg;   %
% Kp2 = 0.91 * Kc;   %PID参数值P
% Ti2 = 16 * Tc;     %PID参数值I
% Td2 = 0.32 *Tc;    %PID参数值D
% Gc2 = Kp2 * (1+1/Ti2/s+Td2*s) %校正函数
% G1 = G0*Gc2          %校正后系统的开环传函
% Gs = feedback(G1,1); %构造负反馈GS
% %%
% %绘制系统阶跃响应曲线
% %绘制系统bode图
% %绘制系统根轨迹图
% figure(1);%绘制阶跃响应
% step(feedback(G0,1))  %原系统阶跃响应
% hold on;              %保存图像
% step(feedback(G0*Gc2,1),t)%校正后系统的阶跃响应
% title('校正前后系统阶跃响应曲线对比')
% hold off;

% figure(2);%绘制校正前系统的bode图
% margin(G0) %绘制原系统Bode图
% title('校正前系统的Bode图')
% figure(3);%绘制校正后系统的bode图
% margin(G1) %绘制新系统Bode图
% title('校正后系统的Bode图')
% figure(4);%绘制校正前系统的根轨迹图
% rlocus(G0) %求取原系统根轨迹
% title('校正前系统的根轨迹图')
% figure(5);%绘制校正后系统的根轨迹图
% rlocus(G1) %求取新系统根轨迹
% title('校正后系统的根轨迹图')








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
% hold on;              %保存图像
% plot(t,x,'r','LineWidth',0.1);
% hold on;              %保存图像
% axis([0 250 -2 2]);





ts=0.001;        %采样时间=0.001s
% sys=tf(5.235e005,[1,87.35,1.047e004,0])  %建立被控对象传递函数
sys=tf(5.235e005,[1.047e004,0])  %建立被控对象传递函数
dsys=c2d(sys,ts,'z');                  %把传递函数离散化
[num,den]=tfdata(dsys,'v');       %  离散化后提取分子、分母

u_1=0.0;u_2=0.0;u_3=0.0;         %输入向量 的初始状态
y_1=0.0;y_2=0.0;y_3=0.0;         %输出的初始状态
x=[0,0,0]';      %PID的3个参数Kp Ki Kd组成的数组
error_1=0;                    %初始误差
S = 1;
% S=input('请选择输入信号的形式：1 阶跃信号 2 方波信号 3 正弦信号\n');
for k=1:1:5000
        time(k)=k*ts;           % 仿真时间500ms
    if S==1
        kp=1.50;ki=0.01;kd=0.01;
        yd(k)=1;                       % 指令为阶跃信号
    elseif S==2
        kp=0.50;ki=0.001;kd=0.001;
        yd(k)=sign(sin(2*2*pi*k*ts));  % 指令为方波信号
    elseif S==3
        kp=1.5;ki=1.0;kd=0.01;          % 指令为正弦信号
        yd(k)=0.5*sin(2*2*pi*k*ts);
    end
    u_u(k) = T^(1+m)/(T-ts*k)^(1+m);
    u(k) =  -k*u_u(k)*x(1);
%     u(k)=kp*x(1)+kd*x(2)+ki*x(3);   % PID控制器??
    % 限制控制器的输出
    if u(k)>=0.1
        u(k)=0.1;
    end
    if u(k)<=-0.1
        u(k)=-0.1;
    end
    % 近似线性模型
    y(k)=-den(2)*y_1+num(2)*u_1;

    error(k)=yd(k)-y(k);
    % 返回pid参数
    u_3=u_2;u_2=u_1;u_1=u(k);
    y_3=y_2;y_2=y_1;y_1=y(k);
    x(1)=error(k);                % 计算 P
    x(2)=(error(k)-error_1)/ts;   % 计算 D
    x(3)=x(3)+error(k)*ts;        % 计算 I

    error_1=error(k);
end
figure(1);
set(0,'defaultfigurecolor','w') % 设置图像背景为白色
plot(time,yd,'r',time,y,'b','linewidth',2);
xlabel('time(s)');ylabel('信号输出');
legend('理想信号','追踪信号');









