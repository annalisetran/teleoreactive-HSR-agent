%{
#include <math.h>
#include "pedro_token.h"

YY_BUFFER_STATE buffstate;
%}

%option noyywrap

DIGIT [0-9]
LOWER [a-z]
UPPER [A-Z]

%%
[0-9]+"."[0-9]+("e"|"e+"|"e-")[0-9]+  {  return DOUBLE_TOKEN; }
[0-9]+"."[0-9]+("E"|"E+"|"E-")[0-9]+  {  return DOUBLE_TOKEN; }
[0-9]+("e"|"e+"|"e-")[0-9]+  { return DOUBLE_TOKEN; }
[0-9]+("E"|"E+"|"E-")[0-9]+  { return DOUBLE_TOKEN; }
[0-9]+"."[0-9]+  { return DOUBLE_TOKEN; }

"-"[0-9]+"."[0-9]+("e"|"e+"|"e-")[0-9]+  { return DOUBLE_TOKEN; }
"-"[0-9]+"."[0-9]+("E"|"E+"|"E-")[0-9]+  { return DOUBLE_TOKEN; }
"-"[0-9]+("e"|"e+"|"e-")[0-9]+  { return DOUBLE_TOKEN; }
"-"[0-9]+("E"|"E+"|"E-")[0-9]+  { return DOUBLE_TOKEN; }
"-"[0-9]+"."[0-9]+  { return DOUBLE_TOKEN; }

[0-9]+    { return INT_TOKEN; }
"-"[0-9]+ { return INT_TOKEN; }




"("        {  return OBRA_TOKEN; }
")"        {  return CBRA_TOKEN; }
","        {  return COMMA_TOKEN; }
"["        {  return OSBRA_TOKEN; }
"]"        {  return CSBRA_TOKEN; }
"|"        {  return VBAR_TOKEN; }


[_A-Z][A-Za-z0-9_]*  { return VAR_TOKEN; }


[a-z][A-Za-z0-9_]*  { return ATOM_TOKEN; }

"'"[^'\\]*("\\".[^'\\]*)*"'"  { return  ATOM_TOKEN; }

"\""[^\"\\]*("\\".[^\"\\]*)*"\""  { return STRING_TOKEN; }

[-/+*<=>#@$\\^&~`:.?!;]+  { return ATOM_TOKEN; }

"{}"  { return ATOM_TOKEN; }

"\n" { return NEWLINE_TOKEN; }
[ \t]+

. { return ERROR_TOKEN; } 


%%

void set_buffstate(char* buff, int size)
{
    buffstate = yy_scan_buffer(buff, size);
}

void delete_buffstate()
{
    yy_delete_buffer(buffstate);
}
