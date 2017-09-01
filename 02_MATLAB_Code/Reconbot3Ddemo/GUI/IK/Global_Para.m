
global choice;
global Mode;
global NumTP;
global x;
global y;
global z;
global p;
global theta;
global q11;
global q12;
global q21;
global q22;
global l1; 
global l2;
global SolutionRow;
global q0q1q2_HomePosition;
global p_0;
global deg;
global NumTrajPoints_num;
global Mode_Pos_Ori_TrajPoints_cell;
global q0q1q2_all;
global l1; 
global l2;
global q0q1q2_HomePosition,
global p_0, 
global deg;
global po_num;
global HomePos2SelectedEndPos_OutputData_Origin;
global HomePos2SelectedEndPos_OutputData_RePlan;
global q0q1q2_Pos_mat;
global PosOri_Output_mat;
global step;
global Tr;
global IK_FinalPos;
global FK_FinalPos;


l1 = 230.1390;
l2 = 147.7;
q0q1q2_HomePosition = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
p_0 = [0 0 208.879343162506 0 0 0, 0 0]; % p = 2 * l2 * sin(pi/4)
deg = pi/180;