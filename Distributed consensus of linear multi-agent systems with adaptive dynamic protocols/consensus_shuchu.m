clear;
clc;
close all;
%ȷ��ѭ������
loop=3000; 
%ȷ������
step=0.01;
%����������
n=6;order=3;
%�ڽӾ���
A1 = [0 1 1 0 1 1 ;
     1 0 1 1 0 0 ;
     1 1 0 0 0 1 ;
     0 1 0 0 1 0;
     1 0 0 1 0 0; 
     1 0 1 0 0 0 ];
 A2 = [0 1 0 0 1 1 ;
       1 0 1 1 0 0 ;
       0 1 0 0 0 1 ;
       0 1 0 0 0 1;
       1 0 0 0 0 0; 
       1 0 1 1 0 0 ];
%ϵͳ״̬����
SA=[0 1 0;0 0 1;0 0 0];
%�������
SB=[0 0 1]';
%�������
SC=[1 0 0];
%��������
F=-[3 6.5 4.5];
%״̬x
x_state=-4*rand(order,n)+2;
%x_state=zeros(order,n);
%״̬y
y_state=SC*x_state;
%״̬v
v_state=zeros(order,n);
%����u
u_input=zeros(1,n);
%tao
tao=[1 -1;-1 1];
%L
L=-[2.1115 1.3528 0.6286]';
%״̬x������ֵ
x1_state=zeros(order,n,loop);
c_state=zeros(n,n,loop);
u1_input=zeros(loop,n);
c=[-1.5 -1 -0.5 0 0.5 1;
    -1 -0.5 0 0.5 1 1.5;
   -0.5 0 0.5 1 1.5 2;
   0 0.5 1 1.5 2 2.5;
   0.5 1 1.5 2 2.5 3; 
   1 1.5 2 2.5 3 3.5];

errorAdded = zeros(n);
%-----------ϵͳ��ʼ��--------------------------

%----------��ʼѭ��--------------------------
%---------�����������ѭ����ʵ������---------
for ld=1:loop
    x1_state(:,:,ld)=(x_state(:,:)-x_state(:,1));%����״̬��Ϣ
%     x1_state(:,:,ld)=x_state(:,:);
    c_state(:,:,ld)=c(:,:);%��������Ӧ����Ϣ
    u1_input(ld,:)=u_input;
    if mod(ld, 2) == 0
    A=A1;
    else
    A=A2;
    end
    %����cij
    for i=1:n
        for j=1:n
            c(i,j)=c(i,j)+step*A(i,j)*[(y_state(i)-y_state(j));SC*(v_state(:,i)-v_state(:,j))]'...
                    *tao*[(y_state(i)-y_state(j));SC*(v_state(:,i)-v_state(:,j))];
        end
    end
    %�������������
    for i=1:n
        errorAdded(i)=0;
        for j=1:n
            errorAdded(i)=errorAdded(i)+c(i,j)*A(i,j) *(SC*(v_state(:,i) - v_state(:,j))-(y_state(i)-y_state(j)));
        end
     %����״̬v   
%          v_state(:,i)=(step*(SA+SB*F)+eye(3))*v_state(:,i)+step*L*errorAdded(i);
           v_state(:,i)=step*((SA+SB*F)*v_state(:,i)+L*errorAdded(i))+v_state(:,i);
    end     
    for i=1:n
        u_input(i)=F*v_state(:,i);%F:1*3,v_state:3*6
    end 
    
    %----------------����������м���------------
    for i=1:n
%         x_state(:,i)=(eye(3)+step*SA)*x_state(:,i)+step*SB*u_input(i);
          x_state(:,i)=x_state(:,i)+step*(SA*x_state(:,i)+SB*u_input(i));
          y_state(i)=SC*x_state(:,i);
%         disp(y_state(i));
    end
end
% q(:,:)=x1_state(2,:,:);
% p(:,:)=c_state(2,2,:);
% plot(0:loop-1,q);


figure(1)
q1(:,:)=x1_state(1,:,:);
q2(:,:)=x1_state(2,:,:);
q3(:,:)=x1_state(3,:,:);
p1(:,:)=c_state(1,:,:);
p2(:,:)=c_state(2,:,:);
p3(:,:)=c_state(3,:,:);
plot(0:step:(loop-1)*step,q1,0:step:(loop-1)*step,q2,0:step:(loop-1)*step,q3,'LineWidth',1.5);
figure(2)
plot(0:step:(loop-1)*step,p1,0:step:(loop-1)*step,p2,0:step:(loop-1)*step,p3,'LineWidth',1.5)
figure(3)
plot(0:step:(loop-1)*step,u1_input,'LineWidth',1.5)




