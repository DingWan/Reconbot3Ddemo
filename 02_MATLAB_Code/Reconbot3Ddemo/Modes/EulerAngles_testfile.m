clc
clf
clear

beta = [];
gamma = -pi/6;
q11 = 0;
step = 0;
% for gamma = -pi/6 :180/pi: pi/6
for q11 = 0 :pi/180: pi/2
    
    step = step + 1;
    EulerAngle_output = EulerAngles_beta_gamma_q11_IK(beta, gamma, q11);
    EulerAngle_q11_theta(step,:) = EulerAngle_output(1,:) * 180/pi;
    
end
i = 1:1:step;
plot(i,EulerAngle_q11_theta(i,1),'r-'); hold on
plot(i,EulerAngle_q11_theta(i,2),'g-'); hold on
plot(i,EulerAngle_q11_theta(i,3),'b-'); hold on
plot(i,EulerAngle_q11_theta(i,4),'m-'); hold on
plot(i,EulerAngle_q11_theta(i,5),'k-'); hold on
grid on
% axis equal

if isempty(beta) == 1 && isempty(gamma) ~= 1 && isempty(q11) ~= 1 && q11 ~= pi/2 && q11 ~= -pi/2
    1
elseif isempty(beta) ~= 1 && isempty(gamma) == 1 && isempty(q11) ~= 1 && q11 ~= pi/2 && q11 ~= -pi/2
    2
elseif isempty(beta) ~= 1 && isempty(gamma) ~= 1 && isempty(q11) == 1 || q11 == pi/2 || q11 == -pi/2
    3
end