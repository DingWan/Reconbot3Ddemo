clc
clear

A = [ 1 2 23 4 5 6 2 54 4 64 645 3 56 34];
B = [ -1 1 2 23 -4 -5 -6 2 54 -4 -64 645 -3 56 34];

col_B_positive = find(B>0);
col_B_negative = find(B<0);
j = 0;
for i = 1:length(col_B_positive)
    if i == 1 && col_B_positive(1)>1 
        j = j + 1;
        %seq_postive(1) = 1;
        Changepoint_postive(j) = col_B_positive(i);
    elseif i < length(col_B_positive) && col_B_positive(i+1) - col_B_positive(i) > 1
        j = j + 1;
           Changepoint_postive(j) = col_B_positive(i); 
        j = j + 1;
           Changepoint_postive(j) = col_B_positive(i+1); 
    elseif i == length(col_B_positive) && col_B_positive(length(col_B_positive))<length(B)
        %seq_postive(i) = col_B_positive(i) - col_B_positive(i-1);
        j = j + 1;
        Changepoint_postive(j) = col_B_positive(i);
    end
end

min(B(Changepoint_postive))

for i = 1:length(col_B_negative)
    if i == 1
        seq_negative(1) = 1;
    else
        seq_negative(i) = col_B_negative(i) - col_B_negative(i-1);
    end
end

Changepoint_postive = find(seq_postive > 1)
if col_B_positive(1)>1
    Changepoint_postive_final(1) = col_B_positive(1);
    Changepoint_postive_final(2:length(Changepoint_postive)+1) = Changepoint_postive;
else
    Changepoint_postive_final = Changepoint_postive;
end
if col_B_positive(length(col_B_positive)) < length(B)
    Changepoint_postive_final(length(Changepoint_postive_final)+1) = col_B_positive(length(col_B_positive));
end

B(col_B_positive(Changepoint_postive_final-1))


Changepoint_negative = find(seq_negative > 1)
B(col_B_negative(Changepoint_negative-1))