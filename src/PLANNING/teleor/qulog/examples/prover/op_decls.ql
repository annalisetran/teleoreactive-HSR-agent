
%% Operator declarations for read_op_term and write_op_term in
%% natural_deduction.qlg so users can interact with formulas using
%% these operator declarations

%% NOTE: we can't use => for implies as this is a reserved operator in Qulog.

?-op(480, fy, ~).
?-op(520, xfy, and).
?-op(530, xfy, or).
?-op(540, xfy, ==>).

